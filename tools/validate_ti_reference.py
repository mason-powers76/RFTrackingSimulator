"""Development-only comparison with unmodified, locally installed TI parser.

Run with the project interpreter; TI source is never copied or modified.
Pass --common-dir for installations other than Radar Toolbox 4.00.00.05.
TI's parser uses host-native struct formats: require a little-endian host.
"""

import argparse
import hashlib
import importlib
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

from rf_common.ti import decode_packet

DEFAULT_COMMON = Path(
    "C:/ti/radar_toolbox_4_00_00_05/tools/visualizers/Applications_Visualizer/common"
)


def validate_reference(data: bytes, common_dir: Path) -> dict:
    """Invoke actual parseStandardFrame and compare all fields it exposes.

    TI intentionally discards covariance. Independently unpack its exact I27f
    record layout to check ec[16]; this is a layout check, not a TI EC decoder.
    Run in a separate development/test process to isolate TI module globals.
    """
    if sys.byteorder != "little":
        raise RuntimeError("TI reference parser requires a little-endian host")
    common_dir = common_dir.resolve()
    if not (common_dir / "parseFrame.py").is_file():
        raise FileNotFoundError(f"TI parseFrame.py not found in {common_dir}")
    sys.path.insert(0, str(common_dir))
    sys.dont_write_bytecode = True
    reference = importlib.import_module("parseFrame")
    if Path(reference.__file__).resolve().parent != common_dir:
        raise RuntimeError("a different parseFrame module was already imported")
    parsed = reference.parseStandardFrame(data)
    expected = decode_packet(data)
    if parsed.get("error") != 0:
        raise ValueError(f"TI reference parser returned error {parsed.get('error')}")
    if parsed["frameNum"] != expected.header.frameNumber:
        raise ValueError("TI reference frame number mismatch")
    if parsed.get("numDetectedPoints", 0) != len(expected.points):
        raise ValueError("TI reference point count mismatch")
    xyz = []
    for point in expected.points:
        r, az, el = point.range_m, point.azimuth_rad, point.elevation_rad
        xyz.append(
            [
                r * math.sin(az) * math.cos(el),
                r * math.cos(az) * math.cos(el),
                r * math.sin(el),
                point.radial_velocity_mps,
                point.signal_quality,
            ]
        )
    np.testing.assert_allclose(
        parsed["pointCloud"][:, :5], np.array(xyz).reshape(-1, 5), atol=1e-12
    )
    if parsed.get("numDetectedTracks", 0) != len(expected.targets):
        raise ValueError("TI reference track count mismatch")
    if expected.targets:
        rows = [
            [
                t.tid,
                t.posX,
                t.posY,
                t.posZ,
                t.velX,
                t.velY,
                t.velZ,
                t.accX,
                t.accY,
                t.accZ,
                t.g,
                t.confidenceLevel,
            ]
            for t in expected.targets
        ]
        np.testing.assert_allclose(parsed["trackData"][:, :12], rows, atol=0)
    if expected.target_indices is not None:
        np.testing.assert_array_equal(parsed["trackIndexes"], expected.target_indices)
    if expected.heights:
        rows = [[h.target_id, h.maximum_m, h.minimum_m] for h in expected.heights]
        np.testing.assert_allclose(parsed["heightData"], rows, atol=0)
    cursor, types, covariance_count = 40, [], 0
    for _ in range(expected.header.numTLVs):
        kind, length = struct.unpack_from("<II", data, cursor)
        cursor += 8
        types.append(kind)
        if kind == 1010:
            records = struct.iter_unpack("I27f", data[cursor : cursor + length])
            for record, target in zip(records, expected.targets, strict=True):
                np.testing.assert_array_equal(record[10:26], target.ec)
                covariance_count += 1
        cursor += length
    source_hashes = {
        name: hashlib.sha256((common_dir / name).read_bytes()).hexdigest()
        for name in (
            "parseFrame.py",
            "parseTLVs.py",
            "tlv_defines.py",
            "gui_common.py",
            "gui_parser.py",
        )
    }
    return {
        "validation_scope": "generated processed TI UART capture; no hardware evidence",
        "synthetic_only": True,
        "published_spec_verified": True,
        "ti_reference_parser_verified": True,
        "hardware_capture_verified": False,
        "reference_common_dir": str(common_dir),
        "reference_sha256": source_hashes,
        "header": expected.header.model_dump(),
        "magic_word": data[:8].hex(" "),
        "first_40_bytes": data[:40].hex(" "),
        "byte_length": len(data),
        "tlv_types": types,
        "points": len(expected.points),
        "tracks": len(expected.targets),
        "padding_bytes": len(data) - cursor,
        "covariance_layout_checks": covariance_count,
        "covariance_reference_parser_exposes_values": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--common-dir", type=Path, default=DEFAULT_COMMON)
    args = parser.parse_args()
    try:
        # Validate the first packet; full capture framing is tested separately.
        with args.capture.open("rb") as stream:
            header = stream.read(40)
            if len(header) != 40:
                raise ValueError("capture has no complete header")
            size = struct.unpack_from("<I", header, 12)[0]
            if not 40 <= size <= 1_048_576:
                raise ValueError("invalid capture packet length")
            data = header + stream.read(size - 40)
        print(json.dumps(validate_reference(data, args.common_dir), indent=2))
    except (ImportError, OSError, ValueError, RuntimeError, AssertionError) as error:
        parser.exit(2, f"TI reference validation failed: {type(error).__name__}: {error}\n")


if __name__ == "__main__":
    main()
