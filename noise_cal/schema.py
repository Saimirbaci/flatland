# Copyright 2026 Avidbots Corp.
# Schema constants + validation for the context-conditioned noise model file.
#
# This is the Python side of the contract documented in
# docs/noise_model_format.md. The C++ loader (flatland_server/noise_model.cpp)
# must accept exactly what build_model() here emits. Keep SCHEMA_VERSION in sync
# with NoiseModel::kSchemaVersion.
"""Versioned schema definition for the parametric_linear noise model."""

SCHEMA_VERSION = 1
MODEL_TYPE = "parametric_linear"

# Context features the linear model regresses on (besides the per-surface
# offset). `darkness` is derived from lighting as (1 - lighting) on the C++ side
# and on the calibration side identically.
FEATURES = ("speed", "age", "darkness")


class SchemaError(ValueError):
    """Raised when a model dict does not satisfy the schema contract."""


def _check_coef_block(name, block):
    if block is None:
        return
    if not isinstance(block, dict):
        raise SchemaError("%s must be an object" % name)
    for k, v in block.items():
        if k not in FEATURES:
            raise SchemaError("%s has unknown feature '%s'" % (name, k))
        if not isinstance(v, (int, float)):
            raise SchemaError("%s.%s must be numeric" % (name, k))


def _check_surface_block(name, block):
    if block is None:
        return
    if not isinstance(block, dict):
        raise SchemaError("%s must be an object" % name)
    for k, v in block.items():
        try:
            int(k)
        except (TypeError, ValueError):
            raise SchemaError("%s key '%s' must be an integer id" % (name, k))
        if not isinstance(v, (int, float)):
            raise SchemaError("%s['%s'] must be numeric" % (name, k))


def validate_model(model):
    """Validate a model dict against schema_version 1.

    Mirrors the checks NoiseModel::FromFile performs so a model rejected here
    would also be rejected by the C++ loader (and vice versa). Returns the model
    unchanged on success; raises SchemaError otherwise.
    """
    if not isinstance(model, dict):
        raise SchemaError("model must be a JSON object")
    if model.get("schema_version") != SCHEMA_VERSION:
        raise SchemaError(
            "schema_version must be %d, got %r"
            % (SCHEMA_VERSION, model.get("schema_version"))
        )
    if model.get("model_type") != MODEL_TYPE:
        raise SchemaError(
            "model_type must be %r, got %r" % (MODEL_TYPE, model.get("model_type"))
        )
    channels = model.get("channels", {})
    if not isinstance(channels, dict):
        raise SchemaError("channels must be an object")
    for cname, ch in channels.items():
        if not isinstance(ch, dict):
            raise SchemaError("channel '%s' must be an object" % cname)
        for key in ("base_std", "base_mean"):
            if key in ch and not isinstance(ch[key], (int, float)):
                raise SchemaError("channel '%s'.%s must be numeric" % (cname, key))
        _check_coef_block("channel '%s'.std_coef" % cname, ch.get("std_coef"))
        _check_coef_block("channel '%s'.mean_coef" % cname, ch.get("mean_coef"))
        _check_surface_block(
            "channel '%s'.surface_std_offset" % cname, ch.get("surface_std_offset")
        )
        _check_surface_block(
            "channel '%s'.surface_mean_offset" % cname, ch.get("surface_mean_offset")
        )
    return model
