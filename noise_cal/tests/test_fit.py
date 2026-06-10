# Copyright 2026 Avidbots Corp.
# Tests for the noise_cal calibration toolkit: synthetic round trip, recovery of
# known coefficients, schema validity, and the fit -> export -> reload contract.
"""Unit tests for noise_cal (pure stdlib + pytest)."""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from noise_cal import (  # noqa: E402
    build_model,
    fit_rows,
    generate_synthetic,
    load_csv,
    validate_model,
    write_model,
)
from noise_cal.fit import SYNTHETIC_TRUTH, fit_channel  # noqa: E402
from noise_cal.schema import SchemaError  # noqa: E402


def _make_rows(tmp_path, n=6000, seed=11):
    csv_path = os.path.join(str(tmp_path), "synth.csv")
    generate_synthetic(csv_path, n=n, seed=seed)
    return load_csv(csv_path)


def test_synthetic_generation_and_load(tmp_path):
    rows = _make_rows(tmp_path, n=200)
    assert rows
    channels = {r["channel"] for r in rows}
    assert channels == set(SYNTHETIC_TRUTH.keys())
    for r in rows:
        assert r["surface_id"] in (0, 1, 2)
        assert 0.0 <= r["lighting"] <= 1.0


def test_recovers_known_std_coefficients(tmp_path):
    rows = _make_rows(tmp_path, n=8000, seed=3)
    fits = fit_rows(rows)
    truth = SYNTHETIC_TRUTH["range"]
    f = fits["range"]
    # Base std and the dominant coefficients should be recovered to a loose
    # tolerance (statistical fit, not exact).
    assert f["base_std"] == pytest.approx(truth["base_std"], abs=0.012)
    assert f["std_coef"]["darkness"] == pytest.approx(
        truth["std_coef"]["darkness"], abs=0.012
    )
    assert f["surface_std_offset"][2] == pytest.approx(
        truth["surface_std_offset"][2], abs=0.012
    )
    # Mean is ~zero for the synthetic data.
    assert abs(f["base_mean"]) < 0.01


def test_std_increases_with_surface_roughness(tmp_path):
    rows = _make_rows(tmp_path, n=8000, seed=5)
    f = fit_rows(rows)["range"]
    off = f["surface_std_offset"]
    # Surface 2 (slipperiest/roughest) must add more std than surface 1.
    assert off[2] > off[1] > -0.005


def test_export_is_schema_valid_and_reloads(tmp_path):
    rows = _make_rows(tmp_path, n=3000, seed=9)
    model = build_model(fit_rows(rows), metadata={"fit_source": "synthetic"})
    validate_model(model)
    out = os.path.join(str(tmp_path), "model.json")
    write_model(model, out)
    with open(out) as fh:
        reloaded = json.load(fh)
    assert reloaded["schema_version"] == 1
    assert reloaded["model_type"] == "parametric_linear"
    assert "range" in reloaded["channels"]


def test_validate_rejects_bad_version():
    with pytest.raises(SchemaError):
        validate_model({"schema_version": 99, "model_type": "parametric_linear",
                        "channels": {}})


def test_validate_rejects_unknown_feature():
    bad = {
        "schema_version": 1,
        "model_type": "parametric_linear",
        "channels": {"range": {"std_coef": {"temperature": 1.0}}},
    }
    with pytest.raises(SchemaError):
        validate_model(bad)


def test_single_surface_channel_has_no_dummies():
    rows = [
        {"channel": "x", "surface_id": 0, "speed": s, "lighting": 1.0,
         "age": 0.0, "residual": 0.01 * s}
        for s in [0.0, 0.5, 1.0, 1.5, 0.2, 0.8]
    ]
    f = fit_channel(rows)
    assert set(f["surface_std_offset"].keys()) == {0}
