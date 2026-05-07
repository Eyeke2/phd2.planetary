# PHD2 stress-test client

Async JSON-RPC stress harness for PHD2's event server. Built initially
to validate the deadlock cleanup commit series; intended as a general
soak-test harness for future changes.

## Prerequisites

* **Python 3.8 or later.** No third-party dependencies.
* **PHD2 running**, with the deadlock cleanup commits applied if you
  want to exercise them.
* **Gear connected.** Sky Simulator + ASCOM Telescope/Camera Simulator
  is the recommended starting configuration -- it exercises the
  worker -> main GUI-context capture/move pattern that is the primary
  deadlock surface, without needing physical hardware.
* **Event server enabled** in PHD2: Tools -> Enable Server. Default
  port is 4400; the harness defaults to 127.0.0.1:4400.

## Quick start

```
cd tests/stress
python stress.py storm --clients 5 --rate 5 --duration 10
```

Expected on a clean run: zero errors, modest latencies, no
suspicious markers in PHD2's debug log.

## What to watch for in PHD2's debug log

The cleanup commits added three log markers that surface the bug
classes we care about. Tail PHD2's debug log file in another window
while the stress runs.

Debug log location:

| OS      | Path                                  |
|---------|---------------------------------------|
| Windows | `%LOCALAPPDATA%\phd2\PHD2_DebugLog_*.txt` |
| macOS   | `~/Library/Application Support/PHD2/PHD2_DebugLog_*.txt` |
| Linux   | `~/.phd2/PHD2_DebugLog_*.txt`         |

| Marker | Meaning | What to do |
|---|---|---|
| `UI-SAFETY:` | An unsafe wx-pumping op fired from inside an RPC handler. After the cleanup, this should NEVER appear. | Find the RPC verb mentioned in the line and the source file/line. That code path needs the same kind of fix step (a)-(d) applied. |
| `UI-SAFETY: (vetted)` | Same path, but in a region annotated with `::ui_safety::VettedScope`. Expected from `MyFrame::StopWorkerThread` on `set_connected: false`. | Informational only. |
| `TELEMETRY(e.1):` | The worker -> main dangling-pointer race fired. Worker was force-killed and a stale REQUEST_*_EVENT was discarded safely. | Pre-cleanup this would have crashed. Now it's recoverable. Frequency tells you how urgent the per-driver-thread refactor is. |
| `SLOW elapsed=Nms` | `StopWorkerThread` spent more than the 1 s timeout draining. | Indicates a structurally-blocking driver. The activity name on the same log line tells you which Handle* the worker was stuck in. |

## Scenarios

### `storm` -- connection storm

N concurrent clients, each hammering read-only RPCs at a steady rate.
Targets connection lifecycle, RPC dispatch fairness, and any deadlock
that scales with concurrent load.

```
python stress.py storm --clients 20 --rate 10 --duration 60
```

* `--clients`: how many concurrent TCP connections (default 10).
* `--rate`: requests per second per client (default 10). Effective
  rate is `clients * rate` server-side.
* `--duration`: seconds (default 30).

### `firehose` -- notification firehose

Connects once, never sends an RPC, just consumes notifications.
Validates that PHD2 doesn't block when a client is a slow consumer
of its event stream.

```
# Start looping or guiding in PHD2's UI first, then:
python stress.py firehose --duration 60
```

If you don't start looping/guiding first, you'll get only the
initial state-snapshot events. That's not a bug -- there's just
nothing to stream.

### `mixload` -- loop + concurrent RPC hammering

One client cycles `loop` -> wait 3 s -> `stop_capture` while a second
client hammers read-only RPCs at ~100 Hz. Targets RPC dispatch
fairness during in-flight exposures.

```
python stress.py mixload --duration 60
```

The scenario auto-issues a final `stop_capture` on exit, so PHD2
won't be left looping if you Ctrl-C.

NOTE: contrary to what the original commit message implied, mixload
does NOT actually exercise `StopWorkerThread` -- `stop_capture` just
toggles `CaptureActive`; the worker thread keeps running idle.
`StopWorkerThread` is only triggered by gear disconnect, profile
switch, or PHD2 shutdown. For the test that actually stresses the
step (e) cleanup, use `gearcycle`.

### `gearcycle` -- repeated set_connected disconnect/reconnect

One client cycles `set_connected: false` -> settle -> `set_connected:
true` -> settle, in a loop. The second client hammers read-only RPCs
at ~20 Hz throughout. THIS is the scenario that actually exercises
`StopWorkerThread` (the bit of code rewritten in step (e) of the
deadlock cleanup): every `set_connected: false` triggers gear
teardown which stops both worker threads.

```
python stress.py gearcycle --duration 120 --settle 2
```

* `--duration`: total seconds (default 120). Each full cycle is
  typically 20-60s on real ASCOM gear, so 120s gives ~3-6 cycles.
* `--settle`: seconds between disconnect and reconnect (default 2).

Each `set_connected` call uses a 60-second RPC timeout because real
ASCOM drivers can legitimately take 10-30 seconds to connect or
disconnect, and simulators sometimes emulate that. The hammer's
read RPCs use a shorter 15-second timeout: long enough to ride out
a teardown without aborting, short enough to surface a true hang.

Errors from the hammer during a teardown window are EXPECTED and
not a failure -- the relevant pass/fail signal is the absence of
`UI-SAFETY:`, `TELEMETRY(e.1):`, and `SLOW` markers in PHD2's debug
log, NOT perfect RPC fidelity.

The scenario attempts a final `set_connected: true` on exit so PHD2
is left in a usable state.

## Reading the output

Sample healthy run:

```
[storm] 10 clients @ 10.0 req/s each, 30.0s
  total calls:       2956
  call errors:       0
  connect errors:    0
  effective rate:    98.5 req/s
  latency p50/p99:   1.43 / 12.40 ms
  latency max:       38.20 ms
  latency mean:      2.18 ms
```

Healthy looks like: zero errors; p50 latency under 5 ms; p99 under
20 ms; p99/p50 ratio under ~10x; no markers in the debug log.

If `call errors` is non-zero, re-run with `-v` to see which RPC
failed and why.

If latency spikes above 100 ms in p99, correlate the timestamp with
PHD2's debug log -- usually it's a long-running operation on the
main thread (camera connect, profile switch).

If you see `UI-SAFETY:` (without `(vetted)`), the harness has done
its job and found a regression. File a bug.

## Files

```
phd2_client.py    asyncio JSON-RPC client (no deps)
stress.py         scenarios + CLI
README.md         this file
```

## Note on intentional limitations

This harness does NOT reproduce the original `wxFindWindowAtPoint`
cross-process `SendMessage` deadlock, because that needs a *foreign
GUI process* with an HWND under the cursor and blocked in `recv()`
on PHD2 -- which a console asyncio client is not. To reproduce the
original bug pre-cleanup, you'd need a small wxPython companion that
parks a window under the cursor while issuing an RPC. That's
deliberately out of scope here; for the cleanup we care about
detection (the UI-SAFETY log markers) more than reproduction.
