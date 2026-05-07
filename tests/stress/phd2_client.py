"""
Async JSON-RPC client for PHD2's event server.

PHD2 listens on TCP port 4400 by default and speaks line-delimited JSON
(see src/event_server.cpp). Each request is a JSON object with `method`,
optional `params`, and an `id` for correlation. The server sends responses
matched by id, plus unsolicited notifications carrying an `Event` field.

Use:

    client = await PHD2Client.connect("127.0.0.1", 4400)
    state = await client.call("get_app_state")
    notif = await client.notifications.get()
    await client.close()

The client is async-safe in the single-task sense (one task driving one
client). Concurrent `call()`s from different tasks on the same client
are fine -- pending requests are correlated by id and stored in a dict.

This module deliberately has no third-party dependencies.
"""

import asyncio
import json
import logging
from typing import Any, Dict, Optional


logger = logging.getLogger("phd2.client")


class PHD2Error(Exception):
    """Raised for transport errors (closed connection, malformed message)
    and for JSON-RPC error responses returned by PHD2."""
    pass


class PHD2Client:
    """One TCP connection to PHD2's event server.

    Construct via `await PHD2Client.connect(...)`. The constructor itself
    is private-ish (takes already-opened reader/writer); use the
    classmethod for normal use.
    """

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter,
                 notif_queue_size: int = 10000):
        self._reader = reader
        self._writer = writer
        self._next_id = 1
        self._pending: Dict[int, asyncio.Future] = {}
        # Notifications stream into this queue. The user's scenario code
        # should drain it; if it doesn't, we drop OLDEST to keep memory
        # bounded under firehose conditions. PHD2 itself does not block
        # on a slow client (the event server uses non-blocking writes),
        # so backpressure is purely on our end.
        self.notifications: asyncio.Queue = asyncio.Queue(maxsize=notif_queue_size)
        self._reader_task = asyncio.create_task(self._read_loop())
        self._closed = False

    @classmethod
    async def connect(cls, host: str = "127.0.0.1", port: int = 4400,
                      notif_queue_size: int = 10000) -> "PHD2Client":
        reader, writer = await asyncio.open_connection(host, port)
        return cls(reader, writer, notif_queue_size=notif_queue_size)

    async def _read_loop(self):
        """Background task: parse one JSON object per line, dispatch to
        either a pending-request future (by id) or the notifications
        queue (by absence of id)."""
        try:
            while True:
                line = await self._reader.readline()
                if not line:
                    # Peer closed cleanly.
                    break
                try:
                    msg = json.loads(line.decode("utf-8").strip())
                except json.JSONDecodeError as e:
                    logger.warning("malformed line from PHD2: %r (%s)", line, e)
                    continue
                self._dispatch(msg)
        except asyncio.CancelledError:
            raise
        except Exception:
            # Unexpected reader-loop crash. Surface to all pending callers
            # so they don't hang forever.
            logger.exception("reader loop crashed")
        finally:
            for fut in self._pending.values():
                if not fut.done():
                    fut.set_exception(PHD2Error("connection closed"))
            self._pending.clear()

    def _dispatch(self, msg: dict):
        # Responses to requests carry the same `id` we sent. PHD2 sometimes
        # echoes id=null for parse errors etc. -- those go to the
        # notifications stream so the caller sees them.
        if "id" in msg and msg["id"] is not None:
            fut = self._pending.pop(msg["id"], None)
            if fut and not fut.done():
                if "error" in msg:
                    fut.set_exception(PHD2Error(msg["error"]))
                else:
                    fut.set_result(msg.get("result"))
            else:
                logger.debug("orphan response id=%s: %r", msg.get("id"), msg)
        else:
            try:
                self.notifications.put_nowait(msg)
            except asyncio.QueueFull:
                # Drop oldest to keep memory bounded under firehose.
                try:
                    self.notifications.get_nowait()
                    self.notifications.put_nowait(msg)
                except (asyncio.QueueEmpty, asyncio.QueueFull):
                    pass

    async def call(self, method: str, params: Optional[list] = None,
                   timeout: float = 10.0) -> Any:
        """Send a JSON-RPC request and await the response.

        Returns the `result` field on success.
        Raises PHD2Error on protocol error, timeout, or closed connection.
        """
        if self._closed:
            raise PHD2Error("client closed")
        rpc_id = self._next_id
        self._next_id += 1
        msg = {"method": method, "id": rpc_id}
        if params is not None:
            msg["params"] = params
        # PHD2 expects line-delimited JSON. We use \r\n out of habit;
        # PHD2's parser accepts either.
        line = json.dumps(msg) + "\r\n"

        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[rpc_id] = fut
        try:
            self._writer.write(line.encode("utf-8"))
            await self._writer.drain()
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending.pop(rpc_id, None)
            raise PHD2Error(f"timeout after {timeout}s calling {method}")

    async def close(self):
        if self._closed:
            return
        self._closed = True
        self._reader_task.cancel()
        try:
            await self._reader_task
        except (asyncio.CancelledError, Exception):
            pass
        try:
            self._writer.close()
            await self._writer.wait_closed()
        except Exception:
            pass
