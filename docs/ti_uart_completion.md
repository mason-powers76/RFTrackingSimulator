# UART milestone completion report

Coding and software validation completed 2026-10-14. Manual Applications
Visualizer operation has not been tested. No physical or virtual COM endpoint
was opened during validation. No driver was installed/configured and no TI
installation file was modified.

1. **Files added:**
   - `src/rf_common/ti_uart.py`: serial protocol, port opener, bounded writes,
     paced DATA transport, CLI emulator, session gate.
   - `src/rf_sim/ti_uart_bridge.py`: additional CLI entry point using existing
     physics, radar source and pygame controls.
   - `configs/ti_uart_host_only.cfg`: host-only 3 m/20 Hz display configuration.
   - `configs/ti_uart_stop_only.cfg`: host-only stop command workaround.
   - `tests/test_ti_uart.py`: 53 deterministic fake-UART test cases.
   - `tools/validate_ti_uart.py`: reproducible in-memory/reference-source audit.
   - `docs/ti_uart_bridge.md`: inspection, coordinates, topology, manual test,
     troubleshooting and limitations.
   - `docs/ti_uart_completion.md`: this report.

2. **Files modified:** `pyproject.toml`, `requirements.txt` (pyserial dependency),
   `README.md` (bridge link), `docs/ti_processed_output.md` (milestone links and
   explicit Applications Visualizer validation state). No existing simulator,
   encoder, parser, UDP/backend or replay implementation was changed.

3. **Exact TI sources inspected:** installation root
   `C:\ti\radar_toolbox_4_00_00_05`. Under
   `tools\visualizers\Applications_Visualizer\common\`:
   `gui_core.py`, `gui_parser.py`, `parseFrame.py`, `parseTLVs.py`,
   `tlv_defines.py`, `gui_common.py`, `gui_threads.py`, `demo_defines.py`,
   `Baud_Rates_Manager.py`, `graph_utilities.py`, `Common_Tabs\plot_3d.py`,
   `Common_Tabs\plot_1d.py`, `Demo_Classes\people_tracking.py`.
   Also `tools\visualizers\Applications_Visualizer\Industrial_Visualizer\gui_main.py`
   and
   `source\ti\examples\Industrial_and_Personal_Electronics\People_Tracking\Overhead_3D_People_Tracking\chirp_configs\pt_6843_3d_ods_overhead_3m_radial.cfg`.
   Command names were enumerated from sibling `pt_6843_3d_ods_overhead*.cfg`
   files. [Inspection table and relevant functions](ti_uart_bridge.md#source-inspection-completed-before-implementation).

4. **xWR6843 CLI baud:** **115200**, selected by TI `Window.onChangeDemo` and
   passed by `Core.connectCom` to `UARTParser.connectComPorts`.

5. **xWR6843 DATA baud:** **921600**, same path. TI explicitly selects parity
   none and one stop bit; it leaves data bits at pyserial's eight-bit default.
   Emulator explicitly configures 8N1/no flow control, with finite I/O timeouts.

6. **Discovered CLI handshake:** TI `sendCfg` adds LF, waits 30 ms before each
   command and reads two lines. Empty first response triggers `comError`;
   neither line's text is parsed for success/error. No prompt is required.
   At completion TI waits another 30 ms and resets CLI input. The minimum
   successful host exchange is two nonempty newline-terminated response lines.

7. **Supported commands:** `sensorStart`, `sensorStop`, and compatibility ACKs
   for `flushCfg`, `dfeDataOutputMode`, `channelCfg`, `adcCfg`, `adcbufCfg`,
   `lowPower`, `profileCfg`, `chirpCfg`, `frameCfg`, `dynamicRACfarCfg`,
   `staticRACfarCfg`, `dynamicRangeAngleCfg`, `dynamic2DAngleCfg`,
   `staticRangeAngleCfg`, `antGeometry0`, `antGeometry1`, `antPhaseRot`,
   `fovCfg`, `compRangeBiasAndRxChanPhase`, `staticBoundaryBox`, `boundaryBox`,
   `sensorPosition`, `gatingParam`, `stateParam`, `allocationParam`,
   `maxAcceleration`, `trackingCfg`, `presenceBoundaryBox`, `fineMotionCfg`.
   Configuration arguments do not alter RF or simulator settings.

8. **Implemented response:** `command\r\nDone\r\n`, without banner/prompt.
   Unknown commands: `command\r\nError: unsupported command\r\n`. Invalid
   sensor arguments/overlong commands also receive two-line errors. TI prints
   error strings but does not treat their content as a failed acknowledgement.

9. **sensorStart:** sets RUNNING; subsequent common measurement frames are
   encoded/transmitted. Optional argument 0 or 1 is accepted. Repeats are
   idempotent. TI's start-without-config button only starts host parsing.

10. **sensorStop:** sets STOPPED and pauses new DATA submissions. Physics/manual
    controls continue. In-flight/driver-buffered bytes may finish arriving.
    TI hides its stop button; send the stop-only cfg to exercise this command.
    The hidden callback's missing newline is documented, not worked around by
    guessing when partial text has ended.

11. **DATA architecture:** one `Simulator` → common `RadarFrame` → existing
    `TIFrameEncoder/frame_to_ti/encode_packet` → independent `TIDataUART` →
    arbitrary serial endpoint. CLI/session gating is separate from physics.
    No second motion implementation or tight serial coupling was introduced.

12. **Pacing:** monotonic clock, at least 50 ms between frame write starts at
    20 Hz; no Unix-time sleeps or catch-up backlog. Synthetic clock/sequence
    fields remain owned by the existing encoder.

13. **Backpressure:** synchronous block, no application frame queue; preserve
    order and slow simulation under sustained pressure. Max packet 1 MiB,
    CLI line/read 4096 bytes, per-write timeout 100 ms, total write-loop deadline
    250 ms plus at most one final call's timeout. Errors stop/close both ports.
    OS/virtual-driver buffering is external; no unbounded Python queue exists.

14. **Coordinate findings:** native sensor points/tracks are required for this
    host path. TI rotates both positions with `eulerRot`, then adds sensor
    height. At `sensorPosition 3 0 90`, `(X,Y,Z)` displays `(X,Z,3-Y)`.
    x843 sensorPosition supplies no XY translation. TI plots track positions
    in the 3D tab; it does not rotate track velocity/acceleration in this path.

15. **Existing transform retained:** `(xw-1,3-zw,yw-1)` already produces display
    `(xw-1,yw-1,zw)`, so changing it would risk double mounting rotation.
    Nominal center displays `(0,0,0)`; Target 1 `(-0.5,-0.5,0)`; Target 2
    `(0.5,0.5,0)`. Noise/quantization and car-surface heights affect actual
    points. Original TI height 2.9 would put the nominal floor at -0.1 m.

16. **Recommended demo/config:** Industrial Visualizer → **xWR6843 → 3D People
    Tracking**, with `configs/ti_uart_host_only.cfg` (HOST SOFTWARE EMULATION
    ONLY). It sets height 3, azimuth 0, elevation 90 and 50 ms frame metadata.
    The installed ODS radial cfg above can be sent unchanged for handshake
    comparison, but its 2.9 m/120 ms settings do not match our simulator.

17. **Start command**, from repository root with your actual simulator endpoints:

    ```powershell
    .\.venv\Scripts\python.exe -m rf_sim.ti_uart_bridge --cli-port COM11 --data-port COM13 --scenario two_targets_manual --seed 42
    ```

    Manual scenarios open pygame automatically. Target 1 WASD, Target 2 arrows;
    focus pygame when driving. Headless autonomous mode: `--scenario two_targets
    --no-gui`. Both port names and baud rates are configurable.

18. **Example topology only:** CLI pair COM10 ↔ COM11; DATA pair COM12 ↔ COM13.
    TI opens COM10/COM12, bridge opens COM11/COM13. Never share endpoints.
    Driver selection and pair creation remain your manual work.

19. **Visualizer sequence:** create/identify pairs → start emulator → launch
    installed Industrial Visualizer → select xWR6843/3D People Tracking → enter
    opposite CLI/DATA port numbers (Windows GUI prepends COM) → confirm bauds →
    Connect → Select Configuration/host-only cfg → Start and Send Configuration
    → inspect command ACKs/RUNNING → confirm frame counter/cloud/tracks in 3D
    Plot → focus pygame/move both cars independently → verify TI motion →
    select/send stop-only cfg → confirm STOPPED and stable emulator frame count.
    [Exact 22-step guide and troubleshooting](ti_uart_bridge.md#exact-manual-applications-visualizer-test).

20. **New tests:** 53 cases cover partial/full commands, all line endings,
    batching, state changes, every allowed config name, exact replies, unknown/
    oversized input, byte identity/order, monotonic pacing, slow producer,
    stop/start/stop/resume, packet bounds, short/zero-progress writes, deadlines,
    disconnect/read/write failures, port settings/errors, cleanup/interrupts,
    real physics with fake ports and both existing pygame manual controls.
    Tests open no COM endpoints and existing TI encoder/parser tests remain intact.

21. **Complete pytest:** `159 passed in 13.64s` (106 existing + 53 new).
    New UART suite separately: `53 passed in 6.29s`.
    In-memory audit: four frames, **1792 bytes**, byte-for-byte identical to
    direct encoder output; all four accepted by TI's reference parser; 0.15 s
    simulated pacing span. TI's actual `eulerRot` confirmed all three expected
    display positions. Captured stream SHA-256:
    `8011c7fe81264a95ab02e6fcf50aefd4eddc20adc500f6cb67a6bf4e8225cefd`.
    Reproduce with `python -B tools/validate_ti_uart.py`.

22. **Ruff/format:** `ruff check .` → all checks passed;
    `ruff format --check .` → 50 files already formatted.
    `git diff --check` passed. pyserial 3.5 was installed in the project venv;
    no other dependency/driver was installed.

23. **Remaining manual work:** create two virtual pairs; execute the linked
    visualizer test, record results/config/bauds and both motions/stop/resume.
    Actual COM transport and GUI session operation remain unverified.

24. **Known limits:** configuration commands are host ACKs; no RF processing,
    hardware electrical/timing guarantee, full firmware CLI/prompt/reset/query
    emulation, global keyboard hook or range-profile output. Use 3D Plot/SNR
    coloring and leave fall detection off. Synthetic misses/noise are expected;
    native-Z height metadata is not calibrated human height. GUI point display
    has TI's one-frame association delay. Serial backpressure can slow physics;
    fatal write errors require restarting the session. Existing normal GUI,
    UDP/backend, binary capture/replay and seed modes are preserved.

25. **Validation state:**

    ```text
    synthetic_only = TRUE
    published_spec_verified = TRUE
    ti_reference_parser_verified = TRUE
    applications_visualizer_verified = FALSE
    hardware_capture_verified = FALSE
    ```
