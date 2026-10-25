"""The envelopes against TowerPro's lettered dimensions."""

import pytest
from mechanism import servos


@pytest.mark.parametrize("spec", [servos.MG90D, servos.SG90])
def test_overall_height_is_dimension_a(spec):
    box = servos.servo(spec, cable=False).bounding_box()
    assert box.size.Z == pytest.approx(spec.overall, abs=0.01)


@pytest.mark.parametrize("spec", [servos.MG90D, servos.SG90])
def test_the_tabs_span_dimension_e(spec):
    box = servos.servo(spec, cable=False).bounding_box()
    assert box.size.X == pytest.approx(spec.tab_span, abs=0.01)


def test_the_mg90d_reaches_deeper_below_its_tabs_than_the_sg90():
    """The likely reason it does not fit the generic 9 g kits: 19.8 against ~15.9 mm."""
    assert servos.MG90D.tab_underside - servos.SG90.tab_underside > 3.5


def test_a_margin_grows_the_envelope_on_every_side():
    plain = servos.servo(cable=False).bounding_box().size
    grown = servos.servo(margin=0.5, cable=False).bounding_box().size
    assert grown.X == pytest.approx(plain.X + 1.0, abs=0.01)
    assert grown.Y == pytest.approx(plain.Y + 1.0, abs=0.01)
