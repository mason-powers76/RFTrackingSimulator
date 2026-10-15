# RF target simulator

A native Windows 11 / Python 3.13 senior-design tool for developing system software
before a TI mmWave board is integrated. It simulates **target-level output** after
onboard processing, not radio waves or raw radar samples. A local UDP/SQLite/HTTP
backend is implemented; the dashboard, LLM integration, and TI parser remain future work.

```text
               DEVELOPMENT
RC Simulator (motion/controller -> ground truth)
     |
     v
Simulated Radar Source (noise, detections, latency)
     |
     v
TargetFrame
     +----> UDP ----> Backend ----> SQLite / HTTP API ----> Dashboard (future)
     +----> Recorder (measurement JSONL)

Ground truth ----> separate validation JSONL and debug GUI

                FINAL SYSTEM
TI mmWave Radar
     |
     v
TI communication / UART / TLV parser (future)
     |
     v
Radar-source adapter
     |
     v
TargetFrame
     +----> Backend/Dashboard
     +----> Database
```

**TargetFrame is the integration boundary.** `RadarSource` is a small protocol with
`poll(current_time) -> list[TargetFrame]`. The simulated source accepts truth through
its internal `acquire()` method and releases only measured frames through `poll()`.
A TI adapter would acquire bytes independently and map board-specific units, IDs,
coordinates, timestamps, and available metadata into this normalized contract.
TI output is not assumed to match the synthetic fields automatically. Downstream
publishers, recordings, and consumers use only TargetFrame.

## Install in the project environment

Run PowerShell in the repository root. In IntelliJ IDEA, select
`.venv\Scripts\python.exe` as the project interpreter. Existing `.venv` users can skip
creation. To create it, use an installed Python **3.13** executable (the Windows
Python launcher example below applies if you have that launcher):

```powershell
py -3.13 -m venv .venv
```

All remaining Python commands explicitly use the local interpreter and do not need
activation, PYTHONPATH changes, Conda, WSL, or Docker:

```powershell
.\.venv\Scripts\python.exe -c "import sys; print(sys.executable); print(sys.version)"
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m rf_sim.main --help
```

`pip install -e .` installs the package and runtime dependencies; `.[dev]` also
installs pytest and Ruff. Runtime dependencies are NumPy, Pygame **2.6.1**,
Pydantic **2.x**, and PyYAML. Package discovery is configured for `src/rf_sim`.
After activating `.venv` with `.\.venv\Scripts\Activate.ps1`, the equivalent
short forms `python -m rf_sim.main ...`, `python -m pytest`, and `python -m ruff`
work from the root. Explicit interpreter commands avoid PowerShell activation-policy issues.

## Run

```powershell
.\.venv\Scripts\python.exe -m rf_sim.main --scenario rectangle --gui
.\.venv\Scripts\python.exe -m rf_sim.main --scenario straight_line --gui --udp
.\.venv\Scripts\python.exe -m rf_sim.main --scenario circle --gui
.\.venv\Scripts\python.exe -m rf_sim.main --scenario figure_eight --gui
.\.venv\Scripts\python.exe -m rf_sim.main --scenario two_targets --gui
.\.venv\Scripts\python.exe -m rf_sim.main --scenario two_targets_manual --gui --seed 42
.\.venv\Scripts\python.exe -m rf_sim.main --scenario manual --gui
.\.venv\Scripts\python.exe -m rf_sim.main --scenario stationary --gui --seed 42
.\.venv\Scripts\python.exe -m rf_sim.main --scenario rectangle --duration 10 --no-udp
```

Headless runs are supported; `--duration` specifies simulation seconds and omitting
it runs until Ctrl+C or window exit. `manual` and `two_targets_manual` require `--gui`. Scripted cars
start at the configured initial position, then approach the path; straight_line
stops at its endpoint. Each target has its own controller and vehicle parameters.

`two_targets` supplies two cars (IDs 1 and 2) starting at opposite rectangle corners
and following the same path half a lap apart. This preset replaces the YAML `targets`
list; room, common vehicle, radar, and scenario settings still apply. Custom waypoints
are supported, with the second car starting its waypoint sequence halfway through.

`two_targets_manual` supplies exactly two independently controlled cars at opposite
rectangle corners. Target 1 has its reflector enabled and uses W/S/A/D; Target 2
has no reflector and uses the arrow keys. Hold either or both sets of keys to drive
simultaneously. Both inherit the common vehicle constraints. SPACE stops both,
R resets both positions/headings, and P pauses/resumes the simulation.

| Target | Forward | Brake/reverse | Steer left | Steer right |
| --- | --- | --- | --- | --- |
| 1 | W | S | A | D |
| 2 | Up Arrow | Down Arrow | Left Arrow | Right Arrow |

To transmit both targets through the existing UDP measurement pipeline:

```powershell
.\.venv\Scripts\python.exe -m rf_sim.main --scenario two_targets_manual --gui --udp --seed 42
```

| Key | Action |
| --- | --- |
| W | Accelerate forward |
| S | Brake, then accelerate in reverse |
| A / D | Steer left / right |
| SPACE | Emergency stop (also in scripted scenarios) |
| R | Reset vehicle positions and controllers |
| P | Pause/unpause |
| ESC | Exit |

Reset preserves the frame sequence, simulation clock, pending radar frames, and
RNG sequence. Start a new process to reproduce a run from its beginning.

The GUI is a debug tool: green circles and heading lines show truth; orange rings
and crosses show measured positions. Both have ten-second trails. Blue marks the
radar. The panel shows the first configured target in detail; all targets are drawn.
A missing measurement removes its current marker but leaves its recent trail.
For `two_targets_manual`, the panel shows truth and radar details for both targets.

## Configuration and motion

Built-in defaults allow running from any working directory after installation.
`--config config/default.yaml` loads optional YAML overrides; missing fields inherit
defaults and unknown/invalid settings fail validation. CLI seed and network flags
override YAML. UDP is disabled by default; `--no-udp` overrides enabled YAML.

The default room is 2 by 2 metres, origin bottom-left, +x right, +y up. Simulation positions
are always metres; only GUI drawing converts to pixels. Angles are radians internally
and degrees in the GUI. The kinematic bicycle uses forward Euler at 50 Hz:

```text
dx/dt = speed*cos(heading)
dy/dt = speed*sin(heading)
dheading/dt = speed/wheelbase*tan(steering)
dspeed/dt = acceleration
```

Speed, acceleration/braking and steering are clamped. A circular footprint clamps
the centre at walls and stops speed in either direction without bouncing. Default
wheelbase 0.25 m, radius 0.12 m, forward/reverse limits 2/1 m/s, acceleration/braking
2.5/3 m/s², and steering 35° are **configurable placeholders**, not measurements
of the eventual RC vehicle. Steering changes instantaneously; tyre slip, inertia,
motor dynamics and car-to-car collisions are not modeled.

`scenario` supports `cruise_speed_mps`, `waypoint_tolerance_m`, `loop`, and optional
`waypoints: [[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]`. Circle and figure-eight use dense
waypoints; heading control is proportional and slows for turns/near waypoints.
`targets` is a list with unique IDs below 1,000,000, name, initial_x, initial_y,
initial_heading (radians), reflector_enabled, reflector_gain_db, and optional
nested `vehicle` parameters for that target. Common vehicle parameters are inherited
when the nested vehicle section is omitted. Initial centres must have wall clearance.

## Radar and timing

Radar operates at 20 Hz independently of the 50 Hz physics and approximately 60 FPS
GUI. The wall-clock accumulator never discards physics ticks when rendering slows.
Sensor opportunities occur on the first physics tick at/after their deadlines;
this gives alternating 40/60 ms acquisition intervals at default rates. Initial
acquisition occurs at the first physics tick. Latency is also released on physics
ticks: a nominal 30 ms delay becomes 40 ms at 50 Hz. Timestamp and simulation_time
refer to **acquisition**, not release. Pausing freezes simulation time and latency.
Radar rates above the physics rate are rejected to avoid sampling the same state
multiple times. Pending frames at exit are not forcibly released early.

Each in-range truth target gets an independent detection trial. Detected x/y and
vx/vy receive independent Gaussian noise; range and azimuth are computed from
**measured** coordinates relative to the configurable sensor. Reported speed is
the nonnegative magnitude of measured velocity; heading is its direction (None
for zero velocity), rather than a hidden ground-truth body heading. Noisy measurements
can extend beyond walls/range gates; true visibility is gated before noise.

The default arena is exactly 2 × 2 m (floor X/Y both 0–2 m). The radar is at
world (1, 1, 3) m, directly above its center. Vehicle centers with the default
0.12 m radius clamp to [0.12, 1.88] on both floor axes, without wrapping.
`two_targets_manual` starts at quarter-room coordinates (0.5, 0.5) and
(1.5, 1.5), with headings 0 and pi and reflectors on and off respectively.

Synthetic track geometry now reports `horizontal_range_m = hypot(dx, dy)` and
`slant_range_m = hypot(dx, dy, target_z - sensor_height_m)` using nominal
track target_z = 0. `range_m` now defaults to slant range; this is a documented
semantic change from the historical horizontal range. Set
`radar.legacy_horizontal_range: true` for legacy consumers requiring horizontal
`range_m`. Explicit ranges remain available in either mode, and frame metadata
identifies the mode. Range gating, SNR and confidence use slant range in both modes.
Old recordings without explicit ranges remain readable; their historical
`range_m` values are not reinterpreted or converted.

Track `azimuth_rad` retains its legacy meaning: floor bearing `atan2(dy, dx)`
from +world X, counterclockwise. It is not TI sensor azimuth. TI point geometry
remains sensor (X, down-Y, Z), mapping floor offsets to (dx, height-z, dy);
point azimuth is atan2(sensor X, sensor Y), and elevation is
atan2(sensor Z, hypot(sensor X, sensor Y)). Scatterers intentionally occupy
the configured object height; track centers stay nominally on the floor.

Defaults: position noise 0.04 m, velocity noise 0.05 m/s, detection probability
0.98, latency 30 ms, false-alarm probability 0.0. False alarms optionally create
at most one uniformly distributed room candidate per frame, subject to range gating.
They get IDs starting at 1,000,000 to avoid collisions, but carry no truth flag.
These IDs are a simulator convention: downstream classifiers must not use the
numeric range as a real-world authenticity test. Clutter IDs are transient.

Synthetic SNR proxy: `30 - 2*range_m + reflector_gain_db + Gaussian noise`.
Reflectors add 6 dB by default; this implementation changes SNR/confidence, not
detection probability. Confidence is a bounded logistic SNR score with a mild
range penalty (see radar.py). Neither is a radar-equation/RCS model. There are no
friendly/enemy labels. All stochastic effects use one seeded NumPy generator.
Same seed, configuration, library versions and control inputs produce the same
measurement sequence; Unix wall timestamps naturally differ between processes.

## UDP and normalized schema

Open two terminals in the root:

```powershell
# Terminal 1
.\.venv\Scripts\python.exe tools/udp_receiver.py
# Terminal 2
.\.venv\Scripts\python.exe -m rf_sim.main --scenario rectangle --gui --udp --seed 42
```

Each validated frame is UTF-8 JSON in one UDP datagram, default `127.0.0.1:5005`.
Change destination using `--udp-host` and `--udp-port`. The receiver accepts
`--host`, `--port`, optional `--count` and `--timeout`. It validates every frame
and reports timestamp, count, positions, velocities, range, azimuth, confidence
and SNR. UDP has no delivery/order guarantees and payloads above 65,507 bytes
are rejected; this is intended for small local target frames.

Example contract (values illustrative):

```json
{
  "schema_version": "1.0",
  "source": "rf_simulator",
  "frame_id": 1042,
  "timestamp": 1790801200.123,
  "simulation_time": 52.1,
  "targets": [{
    "target_id": 1, "x": 3.18, "y": 1.72,
    "vx": 0.54, "vy": 0.12, "speed": 0.553,
    "heading_rad": 0.219, "range_m": 1.729, "azimuth_rad": 1.467,
    "confidence": 0.94, "snr_db": 21.2
  }]
}
```

Pydantic rejects nonfinite numbers, invalid confidence and extra fields.
`targets` may be empty. Truth, reflector settings, controller state, and names
are never included in TargetFrame. `timestamp` is Unix seconds; simulation_time
is elapsed acquisition time. Measurements use room coordinates, with azimuth
relative to +x, counterclockwise in radians. Hardware coordinate transforms must
be handled by the future source adapter.

## Recording and replay

```powershell
.\.venv\Scripts\python.exe -m rf_sim.main --scenario rectangle --gui --udp --record test01
.\.venv\Scripts\python.exe -m rf_sim.replay recordings/test01_measurements.jsonl
.\.venv\Scripts\python.exe -m rf_sim.replay recordings/test01_measurements.jsonl --speed 2.0
```

Creates `recordings/test01_measurements.jsonl` (released frames only) and
`recordings/test01_ground_truth.jsonl` (truth at each physics tick), relative to the
working directory. Existing names are rejected to preserve experiments. Files
are flushed after each write and closed on normal exit/Ctrl+C. Truth timestamps
permit matching a measurement to its acquisition state despite latency.

Replay streams and validates measurement lines, uses acquisition-time intervals
with absolute wall-clock deadlines, and preserves original IDs, source, timestamps
and payloads. It requires no truth file. The first frame sends immediately; replay
does not recreate the initial latency delay. `--no-realtime` sends as fast as
possible; destination flags are `--udp-host` and `--udp-port`. Corrupt or backward
time lines fail with file/line context. Old timestamps intentionally remain old.

## Structure and checks

```text
config/default.yaml          editable example defaults
src/rf_sim/
  config.py, models.py        validated settings and public contract
  motion.py, controllers.py  bicycle physics and controls
  scenarios.py, simulator.py paths and fixed-step orchestration
  radar.py                   sensor and RadarSource protocol
  transport.py               JSON/UDP boundary
  gui.py                     debug visualization/input
  recorder.py, replay.py      separate logs and normalized replay
  main.py                    CLI and wall-clock loop
tools/udp_receiver.py         standalone network validation
tests/                       unit and process integration tests
recordings/                  experiment files (ignored by Git)
```

```powershell
.\.venv\Scripts\python.exe -m pytest
.\.venv\Scripts\python.exe -m ruff check .
.\.venv\Scripts\python.exe -m ruff format --check .
```

Tests cover motion/limits/walls, scenario progress, range/azimuth, noise/dropouts,
seed reproducibility, reflector gain, clutter, multiple targets, latency,
serialization, actual loopback UDP, recording/replay timing and transmission,
coordinate conversion, and CLI/receiver processes. Automated GUI CLI smoke uses
SDL's dummy driver; native desktop launch should also be checked on your workstation.

## Local backend and database

The backend is a separate `rf_backend` package. It imports the existing simulator
contract only in `adapter.py`; state, SQLite, and HTTP depend on backend models.
The standalone `tools/udp_receiver.py` remains a display/debug tool.

```text
Simulator / recording replay -- UDP TargetFrame JSON --> adapter.parse_packet
                                                        |
Future TI bytes --> future TI parser / coordinate adapter|
                         |                              |
                         +----------> RadarFrame <------+
                                         |
                                  RadarStore.ingest
                                         |
                         SQLite frames + detections
                                         |
                                   HTTP JSON API
                                         |
                               Future dashboard / LLM
```

### Start a complete development session

Run these commands in the project root, in separate PowerShell terminals:

```powershell
# Terminal 1: use a new stream name for each new simulator process or replay
.\.venv\Scripts\python.exe -m rf_backend.main --stream-id manual-001

# Terminal 2: the existing simulator and manual controls
.\.venv\Scripts\python.exe -m rf_sim.main --scenario two_targets_manual --gui --udp --seed 42

# Terminal 3: read the latest persisted frame
Invoke-RestMethod http://127.0.0.1:8000/api/latest | ConvertTo-Json -Depth 6
Invoke-RestMethod http://127.0.0.1:8000/api/targets/1 | ConvertTo-Json -Depth 6
```

Stop the backend with Ctrl+C. Default database: `data/radar.sqlite3`, relative to
the working directory. SQLite WAL/SHM companion files are normal. No packages
were added. Existing editable installation exposes both packages under `src`.
For a fresh setup, use the installation commands above.

Backend options: `--db`, required `--stream-id`, `--udp-host`, `--udp-port`,
`--http-host`, `--http-port`. The default UDP bind is `127.0.0.1:5005`; HTTP is
`127.0.0.1:8000`. Do not run the debug receiver on the same UDP port as the backend.
To change ports, match backend `--udp-port` with simulator/replay `--udp-port`.

For headless compatibility testing, replace Terminal 2 with:

```powershell
.\.venv\Scripts\python.exe -m rf_sim.main --scenario two_targets --udp --duration 5 --seed 42
```

For replay, start the backend with a new stream ID and run:

```powershell
.\.venv\Scripts\python.exe -m rf_sim.replay recordings/test01_measurements.jsonl
```

Replay keeps the original acquisition timestamps. Restarting the backend with
the same stream ID resumes queries for that run; a new simulator process needs
a new stream ID because its frame IDs restart at zero. Stored streams remain in
SQLite, and the API reads the stream selected at backend startup. One stream has
one source. This explicit identity avoids guessing whether a late packet is a
sensor restart. Concurrent sources would each need a separate ingestion instance.

### Contract and validation

Existing wire fields remain unchanged: `schema_version` (only `"1.0"`), `source`,
`frame_id`, Unix acquisition `timestamp`, elapsed acquisition `simulation_time`,
and `targets`. Each target has `target_id`, `x`, `y`, `vx`, `vy`, `speed`,
`heading_rad`, `range_m`, `azimuth_rad`, `confidence`, and `snr_db` as shown above.
The simulator wire adapter requires the current fields and validates types,
finite numbers, nonnegative IDs/times/range/speed, confidence bounds, and unknown
fields. It additionally rejects duplicate target IDs and IDs beyond SQLite's
signed 64-bit range. Invalid datagrams are logged and skipped.

Backend `RadarFrame` keeps those names except the simulator schema version.
Positions, identity, source, and acquisition timestamp are required; velocity,
speed, heading, range, azimuth, confidence, SNR, and simulation time can be null.
This is a proposed internal contract, not a claim about TI output. The current
adapter copies measured values without recomputing them. In the simulator,
speed/heading derive from noisy velocity and range/azimuth from noisy position;
confidence and SNR are synthetic. Simulation time is simulator-specific.
`stream_id` and `received_at` are new backend metadata outside the measurement.
Receipt time never replaces acquisition time.

A future TI adapter must map verified hardware data into `RadarFrame`, transform
coordinates into the documented room convention, establish timestamp semantics,
and leave unavailable optional measurements null. It may supply frames directly
to `RadarStore.ingest` or replace the injected UDP parser. No TI format, target
identity stability, confidence meaning, or hardware clock behavior is assumed.

### State and schema

`frames`: integer database primary key `id`, `stream_id`, `source`, source
`frame_id`, acquisition `timestamp`, backend `received_at`, nullable
`simulation_time`; unique `(stream_id, frame_id)` and a stream/time index.

`detections`: foreign key `frame_pk`, `target_id`, and the eleven measurement
fields listed in the contract (including target ID). Primary key
`(frame_pk, target_id)` and a target-history index. Optional fields are nullable.
Each frame and all its detections commit atomically. Raw JSON and simulator truth
are not duplicated into the database. Empty frames are preserved.

Latest state orders by acquisition timestamp, breaking ties by frame ID. Late
frames enter history without replacing a newer frame. Identical duplicates are
ignored; conflicting reused frame IDs are rejected. Gaps in IDs are allowed:
UDP can lose packets. Missing detections disappear from the current frame;
last-seen data remains queryable. There is no extrapolation, re-identification,
or assertion that a last-seen target is still present. Acquisition and receipt
times let a dashboard display stale data when the source stops.

### HTTP interface

These read-only JSON endpoints use the selected stream:

| Endpoint | Result |
| --- | --- |
| `GET /api/latest` | Latest frame, stream ID, receive time; 404 before any frame |
| `GET /api/targets/1` | Target 1's last detection, timestamp, frame ID, receive time, and whether detected in the latest frame |
| `GET /api/targets/1?at=100.5` | Most recent detection at or before Unix time 100.5; latest-frame flag still refers to the current frame |
| `GET /api/frames?start=100&end=110&limit=1000` | Frames in the inclusive acquisition-time interval, ascending, including empty frames |

Unknown endpoints/targets return 404; invalid queries return 400. History defaults
to 1,000 frames and permits at most 10,000. For longer intervals, split requests
into smaller time windows; full boundaries are inclusive. APIs do not translate
Unix timestamps to relative replay time. Example latest response (illustrative):

```json
{
  "stream_id": "manual-001",
  "received_at": 1790801200.163,
  "frame": {
    "source": "rf_simulator", "frame_id": 1042,
    "timestamp": 1790801200.123, "simulation_time": 52.1,
    "targets": [{
      "target_id": 1, "x": 3.18, "y": 1.72,
      "vx": 0.54, "vy": 0.12, "speed": 0.553,
      "heading_rad": 0.219, "range_m": 1.729, "azimuth_rad": 1.467,
      "confidence": 0.94, "snr_db": 21.2
    }]
  }
}
```

The database supports last-seen/as-of positions and period detection lists.
Average reported speed can be calculated with `AVG(d.speed)` over a join of
`detections d` and `frames f ON d.frame_pk=f.id`, filtering stream, target, and
acquisition interval. This is a sample average, not a time-weighted average.
Distance can later be estimated by summing displacement between ordered measured
positions or integrating available speed. Noise, missed detections, and identity
changes affect these estimates; stored measurements are not ground truth.
No analytics or LLM endpoint is implemented in this phase.

### Backend files, tests, and next step

```text
src/rf_backend/models.py    normalized internal models
src/rf_backend/adapter.py   existing wire validation and mapping
src/rf_backend/store.py     transactional persistence and state/history queries
src/rf_backend/udp.py       UDP input, injected parser
src/rf_backend/api.py       read-only HTTP queries
src/rf_backend/main.py      CLI and component lifecycle
tests/test_backend.py       boundary, database, HTTP, UDP, simulator process tests
```

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m ruff check .
.\.venv\Scripts\python.exe -m ruff format --check .
```

The recommended next step is a small dashboard that polls `/api/latest` every
100–200 ms, draws measured targets in room coordinates, and shows acquisition
age and last-seen status. Choose dashboard technology with the team; keep UDP
and hardware packet knowledge in the backend adapters. The HTTP server is a
simple local development interface. Authentication, deployment, cross-origin
browser access, streaming/WebSockets, retention policy, and TI integration remain
future work. Serving a future dashboard from the same origin would avoid a CORS
requirement. The simulator, GUI controls, recording, and replay are unchanged.

## Modeling scope

This DOES model 2D RC-car motion/trajectory, target-level radar positions and
velocities, Gaussian position/velocity error, missed detections, latency, optional
false detections, synthetic confidence/SNR, radar-to-target geometry, UDP transmission,
and recording/replay.

This DOES NOT model electromagnetic fields, GHz signals, radar chirps, antenna
radiation/beam patterns, ADC sampling, Doppler/range/angle FFTs, CFAR, TI firmware,
multipath, true radar cross section, real mmWave noise characteristics, or actual
TI detection performance. There is no occlusion/FOV model and real-target IDs remain
stable rather than undergoing hardware tracker association errors. Numerical defaults
are synthetic software-testing parameters. The goal is to develop system software
before hardware integration, not to predict radar performance.
# IWR6843ISK-ODS compatibility

Processed TI UART encoding and binary captures are now available as an additional
output tool. See [the encoder developer guide](docs/ti_processed_output.md) for
format, quantization, association timing, reference validation, and limitations.

```powershell
.\.venv\Scripts\python.exe -m rf_sim.ti_capture recordings/synthetic_ti_uart.bin --frames 100
.\.venv\Scripts\python.exe -B tools/validate_ti_reference.py recordings/synthetic_ti_uart.bin
```

No virtual COM/config UART is implemented. This is processed-output emulation;
hardware-capture verification remains false.

The default world contains two RC cars. Ground truth (`Vehicle`/`CarState`),
spherical radar detections (`RadarPoint`), normalized floor tracks (`Detection`),
and JSON/TI binary encoding are separate representations. Point and track counts
are variable and independent of the number of cars.

The transport-independent application contract is `rf_common.frame.RadarFrame`.
Its `targets` field retains the existing name for **tracks**, while `detections`
contains sensor points. Frame `timestamp` is host generation/receive Unix time;
`device_time_value` is separate and explicitly labeled `cpu_cycles` for TI.
Optional associations, presence, heights, diagnostics and source metadata are
retained in SQLite frame payloads and exposed through the existing frame API.
Existing databases acquire an additive `frame_details` table on opening.

The synthetic source uses Poisson scatterer counts and uniform scatterers in a
car's heading-aligned length/width/height volume. Configure those dimensions and
`scattering_gain_db` per target; configure `points_per_object`,
`point_detection_probability`, spherical measurement noise, mounting height and
SNR parameters under `radar`. Reflectors add signal gain. Positive Doppler means
increasing sensor distance. Points contain radial velocity, never Cartesian vx/vy
or a ground-truth object ID. The SNR formula is a configurable test heuristic,
not a calibrated RF model.

The existing noisy center/velocity track model is preserved as a synthetic
tracking surrogate, explicitly labeled in source metadata. It does not implement
TI's tracker or infer Cartesian velocity from one Doppler sample. Its random
stream is independent of point generation. Associations are unavailable for this
independent surrogate. Disabling `point_cloud_enabled` leaves track behavior
unchanged. Manual controls, scheduling, latency, recording and UDP remain intact.

`rf_common.ti.decode_packet` decodes complete IWR6843 overhead-demo packets;
`adapt_packet` produces the same application frame. Example:

```python
from rf_common.ti import decode_packet, adapt_packet
from rf_sim.transport import serialize_frame

native = decode_packet(uart_packet_bytes)
frame = adapt_packet(
    native,
    frame_id=application_sequence,
    received_at=host_receive_unix_time,
    floor_origin=(3.0, 2.0),
    previous_device_frame_number=previous_received_device_frame,
)
udp_payload = serialize_frame(frame)
```

The unrotated overhead transform uses sensor X/Z as floor x/y with configurable
origin. Different installation orientations require a corresponding transform.
Target heights retain native sensor Z estimates rather than being presented as
floor heights. Track speed/heading are derived from projected velocity. Native
`tid`, position, velocity, acceleration, `ec[16]`, `g`, and `confidenceLevel` remain
under `source_metadata.ti_targets`. The error covariance is in
range/azimuth/elevation/Doppler coordinates; native confidence is not mapped to a
probability. Track source and timestamp are supplied by the enclosing frame.

TI target-index entries refer to the previous point cloud, including native
253/254/255 reason codes. The adapter preserves those codes and requires the
caller to supply the actual previous device frame number; absent that number the
reference stays unknown. It never assigns those indices to the current points.

Protocol references: [TI Overhead 3D People Tracking guide](https://dev.ti.com/tirex/explore/content/radar_toolbox_4_00_00_05/source/ti/examples/Industrial_and_Personal_Electronics/People_Tracking/Overhead_3D_People_Tracking/docs/overhead_3d_people_tracking_user_guide.html)
and the locally installed Radar Toolbox 4.00.00.05 Applications Visualizer
`common/gui_parser.py`, `parseFrame.py`, `parseTLVs.py`, `tlv_defines.py`, plus
overhead-demo `mss_main.c` and `height_detection.h`. TI files were only read.
The decoder follows firmware/Visualizer payload-only TLV lengths, unsigned
range/SNR, and uint32 height IDs where the HTML descriptions differ.

The existing JSON schema accepts additive frame details and older recordings;
external strict consumers must accept the new optional fields. Live serial port
configuration, installation calibration and
validation against captured hardware packets remain future integration work.

