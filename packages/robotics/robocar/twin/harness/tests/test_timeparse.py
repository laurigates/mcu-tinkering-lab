import pytest
from robocar_twin.timeparse import TimeParseError, format_ms, parse_ms


@pytest.mark.parametrize(
    ("text", "ms"),
    [
        ("900ms", 900),
        ("15s", 15000),
        ("15.6s", 15600),
        ("2m", 120000),
        ("1m30s", 90000),
        ("1h", 3600000),
        (" 30 s ", 30000),
        (0, 0),
    ],
)
def test_parse_ms_accepts(text, ms):
    assert parse_ms(text) == ms


@pytest.mark.parametrize(
    "bad", ["", "15", 15, "15x", "s15", "1.0005s", True, None, "1s junk"]
)
def test_parse_ms_rejects(bad):
    with pytest.raises(TimeParseError):
        parse_ms(bad, field="at")


def test_error_names_field():
    with pytest.raises(TimeParseError, match=r"world\[2\]\.at"):
        parse_ms("15", field="world[2].at")


def test_format_ms():
    assert format_ms(15600) == "15.6s"
