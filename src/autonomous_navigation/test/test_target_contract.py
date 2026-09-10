import pytest

from autonomous_navigation.target_contract import (
    decode_target_request,
    encode_target_request,
)


@pytest.mark.parametrize(
    "mission,name,marker",
    [
        (1, "coordinate", None), (3, "ar1", 1), (4, "ar2", 2),
        (5, "ar3", 3), (7, "ar0", 0), (9, "ar42", 42),
        (2, "mallet", None), (6, "bottle", None),
    ],
)
def test_target_request_round_trip(mission, name, marker):
    encoded = encode_target_request(mission, name)
    assert decode_target_request(encoded) == (mission, name, marker)


@pytest.mark.parametrize(
    "value",
    [
        "", "ar1", "x:ar1", "0:ar1", "1:", "1:arx", "1:ar-2", "1:a:b",
    ],
)
def test_target_request_rejects_malformed_values(value):
    with pytest.raises(ValueError):
        decode_target_request(value)


def test_target_request_encoder_strips_names_and_rejects_bad_fields():
    assert encode_target_request(3, " ar1 ") == "3:ar1"
    with pytest.raises(ValueError):
        encode_target_request(0, "ar1")
    with pytest.raises(ValueError):
        encode_target_request(1, "ar:1")
