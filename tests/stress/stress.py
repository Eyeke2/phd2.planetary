#!/usr/bin/env python3
"""
Stress harness for PHD2's event server.

Four scenarios, all driven by the asyncio JSON-RPC client in
phd2_client.py:

  storm     -- N concurrent clients hammering read-only RPCs.
               Targets connection lifecycle, RPC dispatch fairness,
               and any deadlock that scales with concurrent load.

  firehose  -- One client connects, drains notifications, never sends
               an RPC. PHD2 should keep up regardless of how slow we
               are at draining (the event server uses non-blocking
               writes; if it ever started to block, that would be a
               regression).

  mixload   -- One client drives loop/stop_capture; another hammers
               read-only RPCs concurrently. Targets RPC dispatch
               fairness during in-flight exposures. NOTE: does NOT
               actually exercise StopWorkerThread -- that path is
               only triggered by gear disconnect, not by stop_capture.

  gearcycle -- One client cycles set_connected false/true (which DOES
               trigger StopWorkerThread teardown) while another
               hammers read RPCs. This is the scenario that actually
               stresses the cleanup work in step (e). Slow because
               real ASCOM gear connect/disconnect takes 10-30s.

While running any scenario, watch PHD2's debug log for these markers:

  UI-SAFETY:        an unsafe wx-pumping op fired from inside an RPC
                    handler. After the cleanup commits this should
                    NEVER appear; if it does, we missed an RPC path.

  TELEMETRY(e.1):   the worker->main dangling-pointer race fired.
                    Worker was force-killed and a stale event was
                    discarded safely. Pre-cleanup this would have
                    crashed PHD2.

  SLOW elapsed=Nms  StopWorkerThread spent more than the 1s timeout
                    draining. Indicates a structurally-blocking
                    driver (target for the per-driver-thread refactor).

Prerequisites:
  * PHD2 running with gear connected (Sky Simulator + ASCOM Telescope/
    Camera Simulator is the recommended starting point -- it exercises
    the worker->main GUI-context capture/move pattern that is the
    primary deadlock surface).
  * Event server enabled (Tools -> Enable Server). Default port 4400.

Usage:
  python stress.py storm     --clients 10 --rate 10 --duration 30
  python stress.py firehose  --duration 60
  python stress.py mixload   --duration 30
  python stress.py gearcycle --duration 120 --settle 2

No third-party dependencies; Python 3.8+.
"""

import argparse
import asyncio
import collections
import logging
import statistics
import sys
import time

from phd2_client import PHD2Client, PHD2Error


# ---------------------------------------------------------------------------
# Scenario: connection storm
# ---------------------------------------------------------------------------

# Read-only RPCs that are safe to call at any application state. Mutating
# RPCs (set_paused, set_lock_position, etc.) are NOT in this list because
# we do not want the stress run to leave PHD2 in a weird state.
READ_ONLY_METHODS = [
    "get_app_state",
    "get_pixel_scale",
    "get_exposure",
    "get_paused",
    "get_calibrated",
    "get_connected",
    "get_lock_position",
    "get_use_subframes",
]


async def _storm_one_client(client_id: int, host: str, port: int,
                             rate_hz: float, duration_s: float, stats):
    """Single storm participant. Connects, loops a fixed cadence of
    read-only RPCs for the duration, then disconnects."""
    try:
        client = await PHD2Client.connect(host, port)
    except OSError as e:
        stats["connect_errors"] += 1
        logging.error("client %d: cannot connect: %s", client_id, e)
        return

    interval = (1.0 / rate_hz) if rate_hz > 0 else 0.0
    deadline = time.monotonic() + duration_s
    i = 0
    try:
        while time.monotonic() < deadline:
            method = READ_ONLY_METHODS[i % len(READ_ONLY_METHODS)]
            i += 1
            t0 = time.monotonic()
            try:
                await client.call(method)
                stats["calls"] += 1
                stats["latencies_ms"].append((time.monotonic() - t0) * 1000.0)
            except PHD2Error as e:
                stats["call_errors"] += 1
                logging.warning("client %d %s: %s", client_id, method, e)
            if interval > 0:
                # Account for the time the call took so we approach the
                # requested rate even when latency is non-trivial.
                slack = interval - (time.monotonic() - t0)
                if slack > 0:
                    await asyncio.sleep(slack)
    finally:
        await client.close()


async def scenario_storm(args):
    stats = {"calls": 0, "call_errors": 0, "connect_errors": 0,
             "latencies_ms": []}
    print(f"[storm] {args.clients} clients @ {args.rate:.1f} req/s each, "
          f"{args.duration:.1f}s")
    t0 = time.monotonic()
    tasks = [
        asyncio.create_task(
            _storm_one_client(i, args.host, args.port, args.rate, args.duration, stats))
        for i in range(args.clients)
    ]
    await asyncio.gather(*tasks, return_exceptions=True)
    _print_stats(stats, time.monotonic() - t0)


# ---------------------------------------------------------------------------
# Scenario: notification firehose
# ---------------------------------------------------------------------------

async def scenario_firehose(args):
    """Connect, do nothing, count notifications. To get a real firehose
    you need PHD2 to be either looping or guiding -- the user is expected
    to start that from the UI before launching this scenario."""
    client = await PHD2Client.connect(args.host, args.port)
    print(f"[firehose] connected, draining notifications for {args.duration:.1f}s")
    counts: "collections.Counter[str]" = collections.Counter()
    deadline = time.monotonic() + args.duration
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = await asyncio.wait_for(client.notifications.get(),
                                             timeout=min(remaining, 1.0))
                # Notifications carry an `Event` field per PHD2's protocol.
                # Some malformed/error responses come through with no Event;
                # bucket them under the method name or "<unknown>".
                evt = msg.get("Event") or msg.get("method") or "<unknown>"
                counts[evt] += 1
            except asyncio.TimeoutError:
                continue
    finally:
        await client.close()
    total = sum(counts.values())
    print(f"[firehose] received {total} notifications:")
    for evt, n in counts.most_common():
        print(f"  {n:8d}  {evt}")
    if total == 0:
        print("  (none -- start looping or guiding in PHD2 to get a stream)")


# ---------------------------------------------------------------------------
# Scenario: mixload (loop + concurrent reads)
# ---------------------------------------------------------------------------

async def _mixload_loop_driver(client: PHD2Client, deadline, stats):
    """Drives `loop` -> wait -> `stop_capture` cycles for the duration.
    This is the side that gets the worker into the EXPOSING activity
    state, which is what makes the dangling-pointer race possible."""
    try:
        while time.monotonic() < deadline:
            try:
                await client.call("loop")
                stats["loop_starts"] += 1
            except PHD2Error as e:
                stats["loop_errors"] += 1
                logging.warning("loop: %s", e)
                await asyncio.sleep(0.5)
                continue
            # Run for ~3s, then stop. Short enough that we cycle through
            # several start/stop pairs in a 30s run -- each stop is the
            # interesting moment.
            await asyncio.sleep(min(3.0, max(0.0, deadline - time.monotonic())))
            try:
                await client.call("stop_capture")
                stats["stops"] += 1
            except PHD2Error as e:
                stats["stop_errors"] += 1
                logging.warning("stop_capture: %s", e)
            # Brief settle before next iteration.
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        # Final cleanup so we don't leave PHD2 looping on exit.
        try:
            await client.call("stop_capture", timeout=2.0)
        except Exception:
            pass
        raise


async def _mixload_hammer(client: PHD2Client, deadline, stats):
    """Hammers read-only RPCs at ~100 Hz against PHD2 while the looper
    is running. High RPC rate increases the chance of an RPC arriving
    during stop_capture, which is the moment we want to exercise."""
    i = 0
    while time.monotonic() < deadline:
        method = READ_ONLY_METHODS[i % len(READ_ONLY_METHODS)]
        i += 1
        t0 = time.monotonic()
        try:
            await client.call(method)
            stats["calls"] += 1
            stats["latencies_ms"].append((time.monotonic() - t0) * 1000.0)
        except PHD2Error as e:
            stats["call_errors"] += 1
            logging.warning("hammer %s: %s", method, e)
        await asyncio.sleep(0.01)


async def scenario_mixload(args):
    stats = {"calls": 0, "call_errors": 0,
             "loop_starts": 0, "loop_errors": 0,
             "stops": 0, "stop_errors": 0,
             "latencies_ms": []}
    looper = await PHD2Client.connect(args.host, args.port)
    hammer = await PHD2Client.connect(args.host, args.port)
    print(f"[mixload] looping + hammering RPCs for {args.duration:.1f}s")
    deadline = time.monotonic() + args.duration
    loop_task = asyncio.create_task(_mixload_loop_driver(looper, deadline, stats))
    hammer_task = asyncio.create_task(_mixload_hammer(hammer, deadline, stats))
    try:
        await asyncio.gather(hammer_task, return_exceptions=True)
        loop_task.cancel()
        await asyncio.gather(loop_task, return_exceptions=True)
    finally:
        await hammer.close()
        await looper.close()

    print(f"  loop start/stop:   {stats['loop_starts']}/{stats['stops']}"
          f" (errors: {stats['loop_errors']}/{stats['stop_errors']})")
    _print_stats(stats, args.duration)


# ---------------------------------------------------------------------------
# Scenario: gearcycle (repeated set_connected false/true)
# ---------------------------------------------------------------------------
#
# UNLIKE mixload, this scenario actually exercises StopWorkerThread (the
# code path rewritten in step (e) of the deadlock cleanup): set_connected
# false triggers the gear teardown which calls StopWorkerThread for both
# the primary and secondary worker threads. RPCs from the hammer client
# arriving during that teardown window are exactly the situation that
# motivated the cross-process SendMessage deadlock fix.
#
# Run-time considerations:
#
#   * gear connect/disconnect is genuinely slow on real hardware (often
#     10-30s per cycle just for the ASCOM driver round-trip), and the
#     simulator can be similarly slow when emulating slow hardware. Each
#     set_connected call therefore uses a 60s RPC timeout (vs the default
#     10s) and the scenario default duration is 120s -- enough to get
#     ~3-6 full cycles in even on a slow rig.
#
#   * If the hammer's read RPC happens to land in the middle of a
#     teardown, PHD2 may legitimately return a stale result or an
#     "operation in progress" style error. We log those at DEBUG level
#     and count them in stats but do NOT treat them as failures. The
#     thing we care about is the ABSENCE of UI-SAFETY:, TELEMETRY(e.1):,
#     and SLOW markers in PHD2's debug log -- not perfect RPC fidelity.
#
#   * On exit (clean or interrupted) we attempt one final
#     set_connected:true so PHD2 is left in a usable state.

GEAR_CYCLE_TIMEOUT_S = 60.0


async def _gearcycle_driver(client: PHD2Client, deadline: float,
                             settle_s: float, stats):
    """Drives set_connected false -> settle -> set_connected true ->
    settle, in a loop until deadline. Each set_connected call uses a
    long timeout because real ASCOM drivers can take 10-30s."""
    while time.monotonic() < deadline:
        # Disconnect.
        t0 = time.monotonic()
        try:
            await client.call("set_connected", [False],
                              timeout=GEAR_CYCLE_TIMEOUT_S)
            elapsed_ms = (time.monotonic() - t0) * 1000.0
            stats["disconnect_ms"].append(elapsed_ms)
            stats["disconnects"] += 1
        except PHD2Error as e:
            stats["disconnect_errors"] += 1
            logging.warning("set_connected:false: %s", e)
            # If the disconnect itself failed, give PHD2 a moment before
            # we try again. Hammering a stuck disconnect doesn't help.
            await asyncio.sleep(2.0)

        if time.monotonic() >= deadline:
            break
        await asyncio.sleep(settle_s)

        # Reconnect.
        t0 = time.monotonic()
        try:
            await client.call("set_connected", [True],
                              timeout=GEAR_CYCLE_TIMEOUT_S)
            elapsed_ms = (time.monotonic() - t0) * 1000.0
            stats["connect_ms"].append(elapsed_ms)
            stats["connects"] += 1
        except PHD2Error as e:
            stats["connect_errors"] += 1
            logging.warning("set_connected:true: %s", e)
            await asyncio.sleep(2.0)

        if time.monotonic() < deadline:
            await asyncio.sleep(settle_s)


async def _gearcycle_hammer(client: PHD2Client, deadline: float, stats):
    """Hammers a small set of read-only RPCs. The interesting moments
    are when these land while the driver task is mid set_connected --
    that is exactly the StopWorkerThread / restart window we want to
    stress."""
    methods = ["get_app_state", "get_connected", "get_paused"]
    i = 0
    while time.monotonic() < deadline:
        method = methods[i % len(methods)]
        i += 1
        t0 = time.monotonic()
        try:
            # 15s timeout: the read may queue behind a teardown and
            # take noticeably longer than usual. We still want to
            # surface true hangs (which would indicate a deadlock),
            # so we don't disable the timeout entirely.
            await client.call(method, timeout=15.0)
            stats["calls"] += 1
            stats["latencies_ms"].append((time.monotonic() - t0) * 1000.0)
        except PHD2Error as e:
            stats["call_errors"] += 1
            # DEBUG, not WARNING: errors during the disconnect window
            # are expected and not what we are testing for.
            logging.debug("hammer %s: %s", method, e)
        await asyncio.sleep(0.05)


async def scenario_gearcycle(args):
    stats = {
        "calls": 0, "call_errors": 0,
        "connects": 0, "connect_errors": 0, "connect_ms": [],
        "disconnects": 0, "disconnect_errors": 0, "disconnect_ms": [],
        "latencies_ms": [],
    }
    # Match storm's graceful handling: a connect failure here means PHD2
    # is not running or the event server is disabled. Print a useful
    # message rather than an asyncio traceback.
    try:
        driver = await PHD2Client.connect(args.host, args.port)
        hammer = await PHD2Client.connect(args.host, args.port)
    except OSError as e:
        print(f"[gearcycle] cannot connect to PHD2 at {args.host}:{args.port}: {e}",
              file=sys.stderr)
        print("           Is PHD2 running? Is the event server enabled?",
              file=sys.stderr)
        return
    print(f"[gearcycle] cycling set_connected for {args.duration:.1f}s "
          f"(settle={args.settle:.1f}s between calls)")
    print("           NOTE: gear connect/disconnect can take 10-30s each;")
    print("                 expect only a few full cycles in a 120s run.")
    deadline = time.monotonic() + args.duration
    drv_task = asyncio.create_task(_gearcycle_driver(driver, deadline,
                                                     args.settle, stats))
    ham_task = asyncio.create_task(_gearcycle_hammer(hammer, deadline, stats))
    try:
        await asyncio.gather(drv_task, ham_task, return_exceptions=True)
    finally:
        # Best-effort: leave PHD2 connected so the next scenario can run
        # without manual intervention. Long timeout because this is the
        # connect call after a possibly-aborted cycle.
        try:
            await driver.call("set_connected", [True],
                              timeout=GEAR_CYCLE_TIMEOUT_S)
        except Exception:
            pass
        await driver.close()
        await hammer.close()

    print(f"  disconnects:    {stats['disconnects']:3d}"
          f" (errors: {stats['disconnect_errors']})")
    print(f"  connects:       {stats['connects']:3d}"
          f" (errors: {stats['connect_errors']})")
    if stats["disconnect_ms"]:
        ms = sorted(stats["disconnect_ms"])
        print(f"  disconnect time: median {ms[len(ms)//2]/1000:.1f}s"
              f"  max {max(ms)/1000:.1f}s")
    if stats["connect_ms"]:
        ms = sorted(stats["connect_ms"])
        print(f"  connect time:    median {ms[len(ms)//2]/1000:.1f}s"
              f"  max {max(ms)/1000:.1f}s")
    _print_stats(stats, args.duration)


# ---------------------------------------------------------------------------
# Stats helper
# ---------------------------------------------------------------------------

def _print_stats(stats, elapsed_s: float):
    calls = stats["calls"]
    print(f"  total calls:       {calls}")
    print(f"  call errors:       {stats['call_errors']}")
    if "connect_errors" in stats:
        print(f"  connect errors:    {stats['connect_errors']}")
    if elapsed_s > 0:
        print(f"  effective rate:    {calls / elapsed_s:.1f} req/s")
    if stats["latencies_ms"]:
        lats = sorted(stats["latencies_ms"])
        n = len(lats)
        p50 = lats[n // 2]
        p99 = lats[min(n - 1, (n * 99) // 100)]
        print(f"  latency p50/p99:   {p50:.2f} / {p99:.2f} ms")
        print(f"  latency max:       {max(lats):.2f} ms")
        if n >= 2:
            print(f"  latency mean:      {statistics.mean(lats):.2f} ms")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1",
                   help="PHD2 event-server host (default 127.0.0.1)")
    p.add_argument("--port", type=int, default=4400,
                   help="PHD2 event-server port (default 4400)")
    p.add_argument("--verbose", "-v", action="store_true",
                   help="enable DEBUG-level logging")

    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("storm", help="N clients hammering read-only RPCs")
    s.add_argument("--clients", type=int, default=10)
    s.add_argument("--rate", type=float, default=10.0,
                   help="requests per second per client (default 10)")
    s.add_argument("--duration", type=float, default=30.0)
    s.set_defaults(func=scenario_storm)

    s = sub.add_parser("firehose", help="connect once, drain notifications")
    s.add_argument("--duration", type=float, default=30.0)
    s.set_defaults(func=scenario_firehose)

    s = sub.add_parser("mixload", help="loop + concurrent RPC hammering")
    s.add_argument("--duration", type=float, default=30.0)
    s.set_defaults(func=scenario_mixload)

    s = sub.add_parser("gearcycle",
                       help="repeatedly disconnect/reconnect gear "
                            "(actually exercises StopWorkerThread)")
    # Default duration is 2 minutes because each cycle (disconnect +
    # settle + reconnect + settle) can take 30s+ on real ASCOM gear;
    # 120s gives ~3-6 full cycles which is enough to surface any
    # intermittent issue without making the test feel endless.
    s.add_argument("--duration", type=float, default=120.0,
                   help="total seconds (default 120)")
    s.add_argument("--settle", type=float, default=2.0,
                   help="seconds between disconnect and reconnect "
                        "(default 2)")
    s.set_defaults(func=scenario_gearcycle)

    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    try:
        asyncio.run(args.func(args))
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(130)


if __name__ == "__main__":
    main()
