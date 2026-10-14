"""The only backend module that depends on the simulator's wire contract."""

from rf_backend.models import RadarFrame
from rf_common.wire import TargetFrame


def parse_packet(payload: bytes) -> RadarFrame:
    wire = TargetFrame.model_validate_json(payload, strict=True)
    values = wire.model_dump()
    values.pop("schema_version")
    return RadarFrame.model_validate(values)
