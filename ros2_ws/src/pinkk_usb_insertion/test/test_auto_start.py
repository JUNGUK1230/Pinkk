import pytest

from pinkk_usb_insertion.control.auto_start import (
    AutoStartSample,
    evaluate_auto_start,
)


def _samples(*, u: float = 320.0, depth_step: float = 0.0):
    return [
        AutoStartSample(
            timestamp=index * 0.1,
            center_u=u + (index % 3 - 1),
            center_v=240.0 + (index % 2),
            depth_m=0.20 + depth_step * index,
            axis_deg=2.0 + (index % 2) * 0.2,
        )
        for index in range(51)
    ]


def _evaluate(samples):
    return evaluate_auto_start(
        samples,
        now=5.0,
        stable_duration_seconds=5.0,
        minimum_samples=30,
        image_center_u_px=320.0,
        image_center_v_px=240.0,
        center_tolerance_px=40.0,
        maximum_center_spread_px=5.0,
        maximum_depth_spread_m=0.005,
        maximum_yaw_spread_deg=2.0,
        maximum_sample_gap_seconds=0.3,
    )


def test_centered_stable_port_is_ready():
    result = _evaluate(_samples())
    assert result.ready
    assert result.center_error_px < 2.0


def test_port_outside_center_is_not_ready():
    result = _evaluate(_samples(u=380.0))
    assert not result.ready
    assert result.reason == '포트가 영상 중앙 밖'


def test_depth_motion_is_not_ready():
    result = _evaluate(_samples(depth_step=0.0003))
    assert not result.ready
    assert result.reason == '깊이 흔들림'


def test_short_observation_is_not_ready():
    result = _evaluate(_samples()[:20])
    assert not result.ready
    assert result.reason in ('표본 부족', '관찰 시간 부족')
