# Copyright 2026 Avidbots Corp.
# Offline calibration toolkit for Flatland's context-conditioned noise models.
#
# Fits the `parametric_linear` model (see docs/noise_model_format.md) from
# labeled sensor/odom residual data and exports the versioned JSON consumed by
# flatland_server/noise_model.cpp. Pure stdlib (no numpy/pandas) so it mirrors
# scenario_gen and runs in CI without heavyweight ML deps.
"""noise_cal: calibrate -> export -> load round trip for Flatland noise models."""

from .schema import SCHEMA_VERSION, MODEL_TYPE, FEATURES, validate_model
from .exporter import build_model, write_model
from .fit import fit_rows, fit_channel, generate_synthetic, load_csv

__all__ = [
    "SCHEMA_VERSION",
    "MODEL_TYPE",
    "FEATURES",
    "validate_model",
    "build_model",
    "write_model",
    "fit_rows",
    "fit_channel",
    "generate_synthetic",
    "load_csv",
]
