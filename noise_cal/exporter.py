# Copyright 2026 Avidbots Corp.
# Assemble + write the versioned noise-model JSON from fitted per-channel coefs.
#
# Output is byte-loadable by flatland_server/noise_model.cpp. Keys/structure
# follow docs/noise_model_format.md exactly.
"""Build and serialize the parametric_linear noise-model file."""

import json

from .schema import SCHEMA_VERSION, MODEL_TYPE, FEATURES, validate_model


def _coef_block(coefs):
    """Drop exact-zero features so the file stays compact (C++ defaults to 0)."""
    return {f: round(float(coefs[f]), 9) for f in FEATURES if abs(coefs.get(f, 0.0)) > 0.0}


def _surface_block(offsets):
    return {
        str(int(k)): round(float(v), 9) for k, v in offsets.items() if abs(v) > 0.0
    }


def build_channel(fit):
    """Turn a single fit_channel() result into a schema channel object."""
    ch = {
        "base_std": round(float(fit["base_std"]), 9),
        "base_mean": round(float(fit["base_mean"]), 9),
    }
    std_coef = _coef_block(fit["std_coef"])
    if std_coef:
        ch["std_coef"] = std_coef
    mean_coef = _coef_block(fit["mean_coef"])
    if mean_coef:
        ch["mean_coef"] = mean_coef
    surf_std = _surface_block(fit["surface_std_offset"])
    if surf_std:
        ch["surface_std_offset"] = surf_std
    surf_mean = _surface_block(fit["surface_mean_offset"])
    if surf_mean:
        ch["surface_mean_offset"] = surf_mean
    return ch


def build_model(channel_fits, metadata=None):
    """Assemble the full model dict from {channel_name: fit_channel result}."""
    model = {
        "schema_version": SCHEMA_VERSION,
        "model_type": MODEL_TYPE,
        "metadata": metadata or {},
        "channels": {name: build_channel(fit) for name, fit in channel_fits.items()},
    }
    return validate_model(model)


def write_model(model, path):
    """Validate and write a model dict to `path` as pretty JSON."""
    validate_model(model)
    with open(path, "w") as fh:
        json.dump(model, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return path
