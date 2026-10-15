# TI Applications Visualizer UART milestone

This is **HOST SOFTWARE EMULATION ONLY** for processed xWR6843 UART output.
Applications Visualizer operation still requires the manual test below.
No virtual COM driver is installed/configured by this project. No TI source or
configuration is modified. There is no physical radar, RF, ADC, chirp, FFT, CFAR,
LVDS or DCA1000 emulation. Configuration acknowledgements do not apply RF settings.

## Source inspection completed before implementation

Installation: `C:\ti\radar_toolbox_4_00_00_05`.
The following exact paths were read relative to that installation:

| TI source path | Relevant implementation/evidence |
| --- | --- |
| `tools/visualizers/Applications_Visualizer/Industrial_Visualizer/gui_main.py` | Industrial entry point; imports shared `Window` |
| `tools/visualizers/Applications_Visualizer/common/gui_core.py` | `Window.onChangeDevice/onChangeDemo`, `Core.connectCom`, `parseCfg`, `sendCfg`, `startApp`, `stopSensor`, `parseData`, `updateResetButton` |
| `tools/visualizers/Applications_Visualizer/common/gui_parser.py` | `UARTParser.connectComPorts`, `sendCfg`, `sendLine`, `readAndParseUartDoubleCOMPort` |
| `tools/visualizers/Applications_Visualizer/common/Baud_Rates_Manager.py` | `BaudRatesManager`, input/output rate settings |
| `tools/visualizers/Applications_Visualizer/common/demo_defines.py` | `DEVICE_DEMO_DICT['xWR6843']`, `DEMO_3D_PEOPLE_TRACKING` |
| `tools/visualizers/Applications_Visualizer/common/parseFrame.py` | `parseStandardFrame`, TLV dispatch |
| `tools/visualizers/Applications_Visualizer/common/parseTLVs.py` | `parseCompressedSphericalPointCloudTLV`, `parseTrackTLV`, `parseTargetIndexTLV`, `parseTrackHeightTLV` |
| `tools/visualizers/Applications_Visualizer/common/tlv_defines.py` | IDs 1020, 1010, 1011, 1012 |
| `tools/visualizers/Applications_Visualizer/common/gui_common.py` | `sphericalToCartesianPointCloud` |
| `tools/visualizers/Applications_Visualizer/common/gui_threads.py` | `parseUartThread.run`, `updateQTTargetThread3D.run/drawTrack` |
| `tools/visualizers/Applications_Visualizer/common/Common_Tabs/plot_3d.py` | `Plot3D.parseSensorPosition`, `updatePointCloud`, boundary boxes |
| `tools/visualizers/Applications_Visualizer/common/Common_Tabs/plot_1d.py` | `Plot1D.setRangeValues`; separate range plot initialization (not used for our 3D output) |
| `tools/visualizers/Applications_Visualizer/common/Demo_Classes/people_tracking.py` | `PeopleTracking.updateGraph`, `parseTrackingCfg`, point persistence |
| `tools/visualizers/Applications_Visualizer/common/graph_utilities.py` | `eulerRot`, track geometry |
| `source/ti/examples/Industrial_and_Personal_Electronics/People_Tracking/Overhead_3D_People_Tracking/chirp_configs/pt_6843_3d_ods_overhead_3m_radial.cfg` | Full ODS overhead config, `sensorPosition 2.9 0 90`, 120 ms frame interval |

Command names were also enumerated from the sibling `pt_6843_3d_ods_overhead*.cfg`
files, covering low-bandwidth, static-retention and 8 m/60-degree variants.

### Ports, CLI handshake and start/stop findings

For xWR6843 (IWR6843 family), `gui_core.py:360` selects CLI **115200 baud**
and DATA **921600 baud**. `Core.connectCom` passes both to `connectComPorts`.
Both use `serial.Serial(..., parity=PARITY_NONE, stopbits=STOPBITS_ONE,
timeout=0.6)`. TI omits data bits (pyserial defaults to eight), and omits flow
control (pyserial defaults off). Our ports explicitly use 8N1, no flow control.
TI calls `dataCom.reset_output_buffer()` after opening; this is not an input purge.

`UARTParser.sendCfg` removes blank `\n` lines and lines beginning `%`, adds
`\n` where missing, sleeps **30 ms before each command**, writes its text,
then calls `readline()` twice. For xWR6843 it does not send character-by-character
or read the two additional lines used by the low-power device path.

There is **no parsed success terminator**: TI does not compare either line
against `Done`, echo, a prompt or an error string. An empty **first** response
sets `comError`; an empty second response is only printed. Thus command echo
is not semantically checked, but two newline-terminated response lines avoid
the timeout. The emulator returns exactly:

```text
<received command>\r\n
Done\r\n
```

There is no initial banner and **no prompt**, because neither is required by
the inspected host and unsolicited lines could shift acknowledgement pairing.
An unknown command returns echo plus `Error: unsupported command\r\n`.
TI prints this error but does not recognize it as a command failure; inspect
the emulator log. This is a limited host handshake, not a complete mmWave CLI.

The accepted configuration command names are:

```text
flushCfg dfeDataOutputMode channelCfg adcCfg adcbufCfg lowPower profileCfg
chirpCfg frameCfg dynamicRACfarCfg staticRACfarCfg dynamicRangeAngleCfg
dynamic2DAngleCfg staticRangeAngleCfg antGeometry0 antGeometry1 antPhaseRot
fovCfg compRangeBiasAndRxChanPhase staticBoundaryBox boundaryBox sensorPosition
gatingParam stateParam allocationParam maxAcceleration trackingCfg
presenceBoundaryBox fineMotionCfg
```

All configuration arguments are logged and acknowledged **without changing
simulation or RF parameters**. Set simulation settings using the bridge's YAML
`--config`, baud flags and seed; use matching visualizer metadata. The emulator
does not implement `baudRate`, device reset, calibration, firmware queries, or
other demos. Unsupported names receive the error above.

`sensorStart` (optionally `0` or `1`) sets RUNNING; `sensorStop` (optionally `0`
or `1`) sets STOPPED. Repeated commands are idempotent. CR, LF, CRLF, partial
reads and multiple commands per read are supported. Input is limited to 4096
bytes per line/read; oversized lines are discarded through their delimiter and
receive a two-line error. Commands are processed on the main loop before physics.

TI's config ends with `sensorStart`. After `sendCfg` finishes, TI waits another
30 ms, resets CLI input, and `Core.sendCfg` starts `parseTimer` at `frameTime`.
`Start without Send Configuration` also starts this timer, **but sends no
sensorStart**. `frameCfg` sets this polling interval to half the configured
frame period (25 ms for the supplied 50 ms host config).

The timer runs `parseData`, starts `parseUartThread`, and dispatches to
`readAndParseUartDoubleCOMPort`. The reader scans one byte at a time for the
8-byte magic, reads version and length, then reads `totalPacketLen - 16` bytes
into a per-packet bytearray. pyserial/driver input buffering holds bytes between
reads; this path does not use a separate Python stream accumulation queue.
If DATA stops, its read times out and magic scanning continues/logs errors.

**TI 4.00.00.05 stop caveat:** `updateResetButton` hides the stop button for
all devices. `Core.stopSensor` would send `sensorStop 0` without a newline,
and `sendLine` does not append one. The emulator intentionally waits for a line
delimiter. Use the supplied **stop-only config** through the normal config
sender to test stopping without modifying TI. Do not use board reset: it invokes
TI's external XDS reset utility, which is irrelevant to a virtual UART.

## Coordinate audit: retain native coordinates

The correct host path is **A: native radar-local points and targets, then
visualizer installation rotation/height**. There is no separate Overhead demo
entry or automatic 90-degree rotation inferred from a board type. Select
**xWR6843 → 3D People Tracking**; the configuration supplies the installation.
The existing world-to-sensor adapter is retained unchanged.

For an arbitrary synthetic scattering point `(xw,yw,zw)` and radar `(1,1,3)`:

```text
world -> native sensor:  X = xw - 1, Y = 3 - zw, Z = yw - 1
native -> TLV 1020:      r = sqrt(X²+Y²+Z²)
                        azimuth = atan2(X,Y), elevation = atan2(Z,sqrt(X²+Y²))
TI parser -> Cartesian: (r sin(az) cos(el), r cos(az) cos(el), r sin(el))
TI eulerRot at (elevation=90°, azimuth=0°): (X,Z,-Y)
TI height translation:  (X,Z,3-Y) = (xw-1,yw-1,zw)
```

Tracks take the same path from native `posX/posY/posZ` in TLV 1010, without the
spherical conversion. `Plot3D.updatePointCloud` rotates points and adds height;
`PeopleTracking.updateGraph` does the same to **track positions** in its 3D tab.
The GUI does not rotate track velocities/accelerations in this path. Its thread
plots the resulting XYZ positions directly. Sensor mesh rotation is just the
mount drawing; it does not supply a second data rotation. The x843 form of
`sensorPosition` has **height, azimuth tilt, elevation tilt**, with no XY offsets.
Consequently the display is centered beneath the radar; it does not show our
floor-origin coordinates 0..2 on X/Y. Positive world x/y remain positive plot X/Y.

| Nominal world floor XY | Native track XYZ (track z=0) | Display XYZ with host config |
| --- | --- | --- |
| Center `(1,1)` | `(0,3,0)` | `(0,0,0)` |
| Target 1 initial `(0.5,0.5)` | `(-0.5,3,-0.5)` | `(-0.5,-0.5,0)` |
| Target 2 initial `(1.5,1.5)` | `(0.5,3,0.5)` | `(0.5,0.5,0)` |

Measurements have noise and quantization; point clouds occupy the synthetic
car surface/height (default 0.12 m), while planar track centroids use z=0.
The unchanged TI config uses height **2.9**, so its corresponding display Z is
`zw - 0.1` with our 3 m radar. This offset is predictable, not an encoder error.
Applying the mounting transform in the encoder as well would rotate twice.
Height TLV 1012 remains a native sensor-Z bound and is not a calibrated human
height; fall detection should stay off. The simulator does not synthesize that
TLV or associations unless the common frame explicitly supplies coherent data.
The x843 GUI delays point display by one frame for tracking association, so a
small motion lag is expected even without serial latency. Use SNR coloring first.

## Transport and pacing

```text
one Simulator (existing physics + existing pygame inputs)
  -> common RadarFrame -> TIFrameEncoder/frame_to_ti -> encode_packet
  -> TIDataUART (synchronous, monotonic-paced) -> configured DATA endpoint
configured CLI endpoint -> TICLIEmulator -> TIUARTSession start/stop gate
```

`rf_common.ti_uart` imports no simulator or GUI. Its serial protocol allows test
doubles; pyserial is imported only by the port opener. `rf_sim.ti_uart_bridge`
owns the existing simulator and GUI. Normal simulator/UDP/backend/capture/replay
entry points are unchanged; this additional entry point only emits UART.

At 20 Hz, DATA write start times are separated by at least 50 ms. The clock is
`time.monotonic`, independent of packet timestamps/Unix epoch. One frame is
encoded and written synchronously before the next can be submitted. The bridge
advances the existing fixed-step physics on monotonic deadlines, with no wall
time catch-up backlog on delays. This slows simulated time under sustained
backpressure. Physics, keyboard controls and measurement acquisition continue
while sensor transmission is stopped; resume emits subsequent live frames,
without replaying the stopped interval. P toggles physics pause in the GUI.

**Backpressure policy: block, no application frame queue, no frame drops while
running.** A packet is limited to 1 MiB; CLI has 4096-byte line/read bounds.
The OS/virtual driver has its own finite buffers, outside this application's
control. There is no background producer or unbounded Python queue. Short writes
are completed in order. Each pyserial write has a 100 ms timeout and the overall
write loop a 250 ms deadline (a last call may exceed that deadline by up to its
100 ms timeout). No-progress writes, disconnects or write exceptions stop the
bridge and close both ports, because continuing after a partial packet would
corrupt framing. Shutdown never calls an unbounded `flush()`.

CLI polling and physics share a thread. Stop processing can wait for the current
pacing/write operation; an in-flight packet and bytes already buffered by the
driver may still arrive. Once a stop is processed, no new DATA packets are
submitted. Frame counters may visibly drain briefly after stop; TI's read loop
may continue printing DATA timeouts. Ctrl-C, Escape or closing pygame closes
the bridge ports. The bridge logs command/response status, state changes,
frame counts every five seconds and errors, without logging radar points.

At 921600 baud with 8N1 the theoretical wire budget is **92160 bytes/s**, or
4608 bytes/frame at 20 Hz. Allow headroom. Virtual drivers may model baud timing
differently; matching baud settings does not prove electrical UART behavior.

## Windows topology and startup

Install project dependencies in the existing environment:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

The added dependency is `pyserial>=3.5,<4`. Choose/configure a Windows virtual
serial package yourself. Code is independent of the package. Create **two
independent pairs**, one for each direction's logical UART:

| Pair | Visualizer opens (example) | Emulator opens (example) |
| --- | --- | --- |
| A: CLI | COM10 | COM11 |
| B: DATA | COM12 | COM13 |

`COM10 ↔ COM11` and `COM12 ↔ COM13` are examples only. Reverse sides if desired,
but do not open the same endpoint in both applications or leave a serial terminal
holding an endpoint. A virtual pair is not a single shared COM port.

From the repository root, substituting your actual emulator endpoints:

```powershell
.\.venv\Scripts\python.exe -m rf_sim.ti_uart_bridge --cli-port COM11 --data-port COM13 --scenario two_targets_manual --seed 42
```

Manual scenarios enable the existing pygame GUI automatically: Target 1 WASD,
Target 2 arrow keys; Space stops both, R resets cars, P pauses physics, Escape
exits. **Keep pygame focused when driving**; keys pressed in TI's window are
not global controls. For autonomous/headless output, use `--scenario two_targets
--no-gui`. Optional flags: `--config path.yaml`, `--cli-baud 115200`,
`--data-baud 921600`, `--duration 30` (wall seconds, including stopped time).
Ports are arbitrary names, not restricted to any example COM numbers.

Startup reports scenario, both endpoints/baud, radar rate and **STOPPED**.
Frames begin only when a newline-terminated `sensorStart` arrives.

## Exact manual Applications Visualizer test

1. Create two virtual COM pairs using your chosen Windows tool.
2. Record all four endpoints and assign one pair to CLI and one to DATA.
3. Start the bridge with the simulator endpoints and seed 42 using the command
   above. Confirm pygame opens and the console reports STOPPED.
4. Launch `C:\ti\radar_toolbox_4_00_00_05\tools\visualizers\Applications_Visualizer\Industrial_Visualizer\Industrial_Visualizer.exe`.
5. Select device **xWR6843** (the IWR6843 family entry).
6. Select demo **3D People Tracking**, not 6844, Out of Box or another demo.
7. Enter the opposite CLI endpoint in the GUI's CLI COM field.
8. Enter the opposite DATA endpoint in its DATA COM field. **On Windows these
   fields take the numeric suffix**, e.g. `10` and `12`; source prepends `COM`.
   Confirm Settings → Baud Rates has input 115200 and output 921600; connect.
9. Click **Select Configuration** and load this repository's
   `configs\ti_uart_host_only.cfg`. It supplies a 3 m overhead mount and 50 ms
   period with enough track slots for IDs 1 and 2. It is deliberately not a
   physical radar config. Then click **Start and Send Configuration**.
10. Observe each received command in the emulator console, starting with
    `sensorStop`, `flushCfg`, then host config metadata.
11. Confirm echo plus `Done` acknowledgements in TI output and emulator logs.
    Compatibility ACKs explicitly state that no RF settings are applied.
12. Confirm `sensorStart` and **Sensor state: RUNNING** in the emulator.
13. Confirm the periodic DATA transmitted frame count increases.
14. In TI's **3D Plot**, confirm Frame increments and no frame/TLV parser errors.
15. Confirm point-cloud detections appear. Start with **Color Points By: SNR**;
    tracking association coloring requires additional association data.
16. Confirm two tracked objects usually appear. Synthetic missed detections are
    possible; this is not a persistent hardware tracker. Check Targets count.
17. Focus pygame and drive Target 1 with WASD.
18. Confirm its corresponding TI target moves; nominal start is (-0.5,-0.5,0).
19. Drive Target 2 with arrow keys in pygame.
20. Confirm independent TI target motion from nominal (0.5,0.5,0). Keep fall
    detection off. Cloud noise and the one-frame point delay are expected.
21. Stop through TI: **Select Configuration** → `configs\ti_uart_stop_only.cfg`
    → **Start and Send Configuration**. Despite the button name, this file sends
    only `sensorStop`. The inspected version hides the separate stop button.
22. Confirm **STOPPED** and a stable emulator DATA count after the current write.
    TI may finish buffered frames and then log read timeouts; this alone is not
    failure. Reload/send the host config to resume, then Ctrl-C to close ports.

Optional source-config comparison: load/send the unchanged installed
`pt_6843_3d_ods_overhead_3m_radial.cfg` from the full path in the inspection table.
The emulator accepts all its commands. It uses a 120 ms hardware frame period
and a 2.9 m height: the simulator stays at its configured 20 Hz/3 m because RF
commands are ACKs only. The supplied host config aligns the display metadata
and polling rate; the original is useful for handshake comparison, not for
matching our measurement timing or height. Other ODS configs exist:
`pt_6843_3d_ods_overhead_3m_radial_low_bw.cfg`,
`pt_6843_3d_ods_overhead_3m_radial_staticRetention.cfg`, and
`pt_6843_3d_ods_overhead_8m_60degtilt_staticRetention.cfg`. The 8 m tilted config
does not match this synthetic mounting. AOP siblings are not the ODS selection.

Record date, config, driver/pair assignment, device/demo, bauds, observed
point/track counts, both manual motions and stop/resume results. Only a successful
manual visualizer session can advance `applications_visualizer_verified`.

## Troubleshooting

| Symptom | Check/action |
| --- | --- |
| CLI opens but configuration times out | Verify opposite endpoints of one pair, CLI 115200 8N1, newline-terminated commands, bridge console input and two response lines; no terminal may hold the endpoint. No prompt is needed. |
| Wrong side of virtual pair | Write down all four ports; TI and emulator each open different endpoints within each assigned pair. Confirm the pairs actually connect in the chosen driver UI. |
| CLI works but no DATA | Verify the second pair and 921600 baud, RUNNING state and rising bridge count; use Start and Send Configuration. Start without Send Configuration starts parsing only. |
| DATA arrives but parser errors | Select xWR6843 / 3D People Tracking, binary 8N1 with no text/newline translation or flow control; inspect write errors and exact reference-parser validation. Restart both ends after a partial packet. |
| Wrong baud | CLI 115200, DATA 921600 on both ends; GUI Settings may retain another device's rate. Some virtual drivers ignore baud, so apparent success alone does not validate rates. |
| Port already in use / access denied | Close other bridge, terminal and visualizer instances holding the endpoint; verify no endpoint is shared. |
| Cloud but no tracks | Use 3D Plot and correct demo; confirm bridge frames have common targets/1010 and `trackingCfg` initialized at least three slots (IDs are 1 and 2). Disable fall detection; distinguish transient synthetic misses from persistent absence. |
| Tracks mirrored | Confirm `sensorPosition 3 0 90`, zero azimuth tilt and floor-centered coordinate expectations. Check camera viewpoint before editing mappings. |
| Tracks rotated 90 degrees | Verify overhead elevation 90 and azimuth 0; another config can rotate data again. |
| Axes swapped | Sensor Y is down and sensor Z follows world y; displayed Y comes from sensor Z. Use the coordinate table, not raw native Y. |
| Elevation inverted | Check positive 90-degree elevation tilt, not -90; TI maps native Y to negative display Z before adding height. |
| Points above/below region | Height must be 3, not original config's 2.9; cloud car height/noise differs from planar tracks. Check boundary boxes and double-applied rotations. |
| Visualizer freezes | Check bridge count/logs and correct DATA pair. TI's magic-byte loop can keep waiting/logging while stopped. Restart TI after disconnect/truncated packets, reduce point persistence and keep 3D Plot selected. Do not invoke XDS board reset. |
| Writer cannot keep up | Budget is ~4608 bytes/frame at 20 Hz before headroom. Lower `radar.points_per_object` or `radar.measurement_rate_hz` in YAML and match host frameCfg metadata; keep actual endpoint rates matched. Backpressure blocks/slows simulation; timeout terminates with an error. No unbounded queue is created. |
| Stop button missing / unresponsive | It is hidden in this TI source and its callback lacks a newline. Send the stop-only config as above. |

## Automated validation and limits

Run `python -m pytest tests/test_ti_uart.py -q` for fake-port CLI, DATA, bridge,
shutdown, timing, GUI/manual input and stop/start/stop tests. They require no
driver or COM endpoints. The exact-byte test runs real simulated frames through
the transport and compares the captured byte stream with a separate direct
`TIFrameEncoder`, then decodes each packet to check frame ordering. This proves
the software abstraction, not a real UART or Applications Visualizer session.

`python -B tools/validate_ti_uart.py` produces an in-memory demonstration and
validates the resulting packets against the locally installed, unmodified TI
reference parser and its `eulerRot` function. No GUI/COM port is opened and no
bytecode or config is written in the TI installation.

| Validation level | State |
| --- | --- |
| synthetic_only | TRUE |
| published_spec_verified | TRUE |
| ti_reference_parser_verified | TRUE |
| applications_visualizer_verified | **FALSE** — awaits your manual test |
| hardware_capture_verified | **FALSE** — no hardware capture comparison |

Known limits: no RF processing, CLI prompt/firmware query/reset emulation,
hardware timing or electrical validation; no guaranteed global keyboard input;
configuration ACKs do not synchronize simulator settings; no range-profile TLV
is generated, and the Range Plot tab is outside this test. The minimal host cfg
and complete GUI session remain manually unverified. Existing capture/replay,
UDP/backend and normal simulator modes retain their separate entry points.
