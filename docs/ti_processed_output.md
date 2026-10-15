# Processed TI UART output

This milestone emulates **processed TI UART output**, not raw radar/ADC data.
This capture/encoder milestone adds no COM ports. The subsequent
[UART bridge mode](ti_uart_bridge.md) adds CLI and paced DATA transports.
Neither mode emulates ADC, chirps, FFT, CFAR, LVDS, or DCA1000.
JSON/UDP/backend output remains available.

```text
Simulator or recorded common RadarFrame
    -> frame_to_ti (coordinate/metadata/association adapter)
    -> TIOutput / TITarget (native sensor representation)
    -> encode_packet (pure binary encoder)
    -> bytes -> binary capture / future UART transport
```

`TIFrameEncoder` owns only sequence/clock/previous-cloud state. `encode_frames`
accepts any iterable of common frames; `write_capture` concatenates packets and
creates files exclusively. Failed encoding does not advance encoder state.
The original project had decoding and TI-to-common adaptation, but no encoder.

## Format and supported TLVs

All encoder structs explicitly use little endian (`<`). Magic bytes:
`02 01 04 03 06 05 08 07`. The 40-byte header is magic plus eight uint32 fields:
version, totalPacketLen, platform, frameNumber, timeCpuCycles, numDetectedObj,
numTLVs, subFrameNumber. TLV headers are `<II`; length counts **payload only**.
Packet length is rounded up to a multiple of 32; padding is deterministic zeros
(0-31 bytes), with no extra block when already aligned. Both counts and lengths
come from the encoded data. numDetectedObj counts current radar points, not tracks.

| Type | Payload | Emission rule |
| --- | --- | --- |
| 1020 | `<5f` units + `<bbhHH` records | Always, including zero points |
| 1010 | `<I27f` complete 112-byte target records | When tracks exist |
| 1011 | Unsigned byte per previous-cloud point | Only explicitly supplied, valid associations |
| 1012 | `<I2f`: tid, maximumZ, minimumZ | Only explicit coherent native sensor-Z bounds for an existing track |

1021 presence and unknown TLVs are **not encoded**; common presence is omitted.
The existing decoder's previously verified uint32 presence support is retained.
No new presence definition is introduced. Unknown received TLVs remain recorded
by the decoder; their payloads are not replayed by this encoder.

Target records retain position, velocity, acceleration, **all 16 covariance
values**, g, and native confidenceLevel. If `source_metadata.ti_targets` exists,
it is preserved and checked against common target IDs. Otherwise floor tracks
become planar synthetic native targets: acceleration zero (unavailable),
g=1 and confidenceLevel=0 placeholders. Common confidence is never copied into
native confidence. The synthetic covariance is row-major diagonal
`diag(0.04, 0.0004, 0.0004, 0.01)` in range/azimuth/elevation/Doppler coordinates,
not measured covariance or XYZ covariance. Missing common velocity uses zero
as a documented synthetic placeholder.

## Quantization

`PointUnits` is configurable. Defaults below are serialized as float32; the
encoder quantizes using those transmitted float32 values to avoid scale drift.

| Field | Unit | Integer | Approximate representable range |
| --- | --- | --- | --- |
| Elevation | 0.01 rad | int8 | -1.28 to 1.27 rad |
| Azimuth | 0.01 rad | int8 | -1.28 to 1.27 rad |
| Doppler | 0.001 m/s | int16 | -32.768 to 32.767 m/s |
| Range | 0.001 m | uint16 | 0 to 65.535 m |
| SNR | 0.1 native units | uint16 | 0 to 6553.5 |

Rounding is nearest integer with ties to even. Error is at most half a
transmitted unit. Out-of-range inputs, including negative SNR, are explicitly
rejected before rounding; there is no silent clamping. Units must fit positive
finite float32. Synthetic dB-like signal quality is numerically placed in the
native SNR field as a test heuristic; this does not establish calibration or
equivalence to hardware SNR. Doppler always comes from radial_velocity_mps;
Cartesian track velocities are never used as point Doppler.

## Coordinates

Points already use TI sensor spherical coordinates. They are compressed directly:

```text
X = range * sin(azimuth) * cos(elevation)
Y = range * cos(azimuth) * cos(elevation)
Z = range * sin(elevation)
```

The default unrotated overhead radar is at world `(1,1,3)` metres. Sensor X
corresponds to world x, Y points down, and Z corresponds to world y. The adapter
uses `(X,Y,Z) = (world_x-x0, mounting_height-world_z, world_y-y0)` and transforms
velocity accordingly. The encoder sees only native coordinates. Track world-z
defaults to the simulator's floor-plane surrogate at zero; this is not an
estimated car centroid height. Floor-track azimuth is never reused as TI azimuth.

New simulator frames include sensor_pose metadata, preserving configured mounting
position in recordings. Older recordings use the explicit adapter SensorPose
or the default `(1,1,3)`; pass the original `--config` when replaying older captures
with a different mount. Arbitrary tilted mounts need a future rigid-transform
adapter. Target heights must declare `height_coordinates: native_sensor_Z`.
Those values are not automatically physical vertical heights in this overhead
mount. The current simulator has no coherent tracked native Z-bound estimates,
so its captures omit 1012. Explicit heights are supported and independently tested.

## Association timing and clocks

Common PointAssociations already contained point_frame_id and
device_point_frame_number. The synthetic tracker is independent of scatterer
generation, so it still supplies no associations; this milestone does not invent
truth-ID associations. For supplied indices, the sequence adapter requires an
explicit reference to the previous **emitted** point-cloud frame and exactly
that frame's point count. References to current points, unknown references, or
missing previous context are rejected. Frames may have different point counts.
Codes 253/254/255 are preserved unchanged. A normalized None requires an explicit
native reason code, since it cannot distinguish weak SNR, boundary, or noise.
TI bytes contain no association frame number; callers decoding a stream must
retain the actual previous device number, including across rollover and gaps.

Application frame_id and host timestamp are unchanged. Device frameNumber starts
at zero independently, increments modulo uint32, or preserves an explicitly
provided device_frame_number. Supplied numbers must advance modulo uint32
(forward differences less than 2^31). Recordings may contain application-ID gaps.
The synthetic device clock is **200 MHz**, configurable through TIFrameEncoder:
`round(simulation_time * cpu_clock_hz) modulo 2^32`. It wraps about every 21.475 s.
If simulation time is absent, elapsed host time since the first encoded frame
is used. Explicit uint32 cpu_cycles are preserved; other device-time units are
rejected. Unix epoch is never written as cycles. subFrameNumber defaults to zero;
explicit native header version/platform/subframe metadata is preserved. Defaults
version `0x03050004` and platform `0xA6843` are synthetic output identifiers,
not a claim that the simulator runs TI firmware.

## Capture and independent validation

```powershell
.\.venv\Scripts\python.exe -m rf_sim.ti_capture recordings/synthetic_ti_uart.bin --frames 100
.\.venv\Scripts\python.exe -m rf_sim.ti_capture recordings/ten_seconds.bin --seconds 10 --seed 42
.\.venv\Scripts\python.exe -m rf_sim.ti_capture recordings/replayed.bin --input recordings/test01_measurements.jsonl --frames 100
.\.venv\Scripts\python.exe -B tools/validate_ti_reference.py recordings/synthetic_ti_uart.bin
```

No serial port is required. `--seconds` covers acquisition time from the first
frame, not wall-clock running time; generation runs as fast as possible. `--frames`
is a maximum for finite recordings. Common JSONL and existing schema-1.0 simulator
recordings are supported. Existing output files are rejected. A failed capture
may leave a partial file; complete packets remain inspectable. `recordings/` is
already ignored by Git. TIStreamDecoder accepts arbitrary byte chunks and skips
garbage before magic while using the existing strict packet parser; finish()
detects truncated frames. It does not recover automatically from malformed
complete frames; its length cap prevents trusting unbounded declared sizes.

The validator imports the installed Applications Visualizer common/parseFrame.py,
which calls the original parseTLVs.py and gui_common.py; no TI source is copied,
modified, or stubbed. It defaults to Radar Toolbox 4.00.00.05 under C:/ti;
use `--common-dir` for another installation. Tests also accept
`TI_VISUALIZER_COMMON`. TI uses native struct byte order, so this development
validation requires a little-endian host. NumPy is sufficient for the installed
parser dependency chain; it imported successfully without launching the GUI.
The script disables bytecode writes and reports SHA256 hashes of all five
reference source files, packet fields, and comparison results.

TI reference validation compares frame acceptance/length alignment, point count,
Cartesian conversion, Doppler/SNR, track ID/position/velocity/acceleration/g/native
confidence, association bytes, and heights. TI's display parser discards ec[16];
covariance therefore receives a separate native I27f layout check and a full
common-to-project-parser round-trip test, **not** a claim that TI exposes EC.
The TI integration test skips only when reference sources are absent; import or
validation failures with installed sources fail the test.

Protocol evidence: [TI's Overhead 3D People Tracking guide](https://dev.ti.com/tirex/explore/content/radar_toolbox_4_00_00_05/source/ti/examples/Industrial_and_Personal_Electronics/People_Tracking/Overhead_3D_People_Tracking/docs/overhead_3d_people_tracking_user_guide.html),
installed Applications Visualizer gui_parser.py, parseFrame.py, parseTLVs.py,
tlv_defines.py, gui_common.py, and overhead-demo mss_main.c / height_detection.h.
Published descriptions differ on range/SNR signedness and height ID type;
the encoder follows actual reference structs and firmware unsigned fields.

Validation levels are cumulative evidence: generated data remains synthetic,
while format evidence can exceed our own round trips. Current classification:

| Classification | Status | Meaning |
| --- | --- | --- |
| synthetic_only | TRUE (data origin) | All inputs are synthetic; no hardware measurements |
| published_spec_verified | TRUE | Published frame/TLV layouts reviewed with firmware/parser corrections |
| ti_reference_parser_verified | TRUE | Local unmodified parser accepts generated packets, including all four TLVs |
| applications_visualizer_verified | **FALSE** | UART bridge awaits a manual TI GUI session |
| hardware_capture_verified | **FALSE** | No real IWR6843 UART capture has been compared |

The strongest achieved format-validation tier is ti_reference_parser_verified.
This does not establish visualizer session operation, real tracking performance,
hardware SNR/confidence calibration, or real-device timing. The UART bridge now
provides paced output/backpressure and a source-inspected CLI handshake; a
complete visualizer data session still requires the manual test. Before
hardware: capture the exact board/firmware stream, compare it byte-for-byte at
the schema level, establish mounting/calibration, clock/reset and packet-loss
behavior, and validate genuine associations/heights/SNR/confidence meanings.

## Verified example from this milestone

The seed-42 default capture command with `--frames 3` generated an ignored
1408-byte file at `recordings/synthetic_ti_uart.bin`. Its first packet was accepted
by the installed TI parseStandardFrame with error=0: **448 bytes**, **16 points**,
**2 tracks**, TLVs **1020, 1010**, and **20 zero padding bytes**. The integration
test also independently validates a packet containing all four supported TLVs.

The first 40 bytes decode as follows:

| Header field | Value |
| --- | --- |
| magicWord bytes | 02 01 04 03 06 05 08 07 |
| version | 0x03050004 (50659332) |
| totalPacketLen | 448 |
| platform | 0xA6843 (682051) |
| frameNumber | 0 |
| timeCpuCycles | 4000000 (acquired at simulation time 0.02 s) |
| numDetectedObj | 16 |
| numTLVs | 2 |
| subFrameNumber | 0 |

```text
02 01 04 03 06 05 08 07 04 00 05 03 c0 01 00 00
43 68 0a 00 00 00 00 00 00 09 3d 00 10 00 00 00
02 00 00 00 00 00 00 00
```

Validation was run on the local Radar Toolbox 4.00.00.05 parser. Reference
parseFrame.py SHA256:
`3b93c4924ee78b23d8696a81b2dd2a7e44fd32e56d3b9b2a09c9ea7af09dcdb2`.
The tool prints all five reference-file hashes for reproducibility. All
**106 pytest tests passed**, including the reference integration test (no skips).
