import json

import pytest

from kit_robot.controller_model import (
    build_components,
    validate_command,
)


def make_command(items):
    return json.dumps({
        "kit_type": "earthquake",
        "items": items,
    })


def test_merge_and_assign_slots():
    command = validate_command(
        make_command([
            {"name": "컵라면", "qty": 1},
            {"name": "마스크", "qty": 1},
            {"name": "컵라면", "qty": 1},
        ]),
        {"컵라면", "마스크"},
    )

    components, expected = build_components(
        command, ["slot_1", "slot_2", "slot_3"],
    )

    assert [(c.name, c.index, c.slot) for c in components] == [
        ("컵라면", 0, "slot_1"),
        ("컵라면", 1, "slot_2"),
        ("마스크", 2, "slot_3"),
    ]

    components[0].status = "FAILED"
    assert expected == {"컵라면": 2, "마스크": 1}
    assert components[0].attempts is not components[1].attempts


@pytest.mark.parametrize("qty", [True, 0, -1, "1", 1.5])
def test_reject_invalid_quantity(qty):
    with pytest.raises(ValueError):
        validate_command(
            make_command([{"name": "마스크", "qty": qty}]),
            {"마스크"},
        )


def test_reject_insufficient_slots():
    command = validate_command(
        make_command([{"name": "마스크", "qty": 2}]),
        {"마스크"},
    )

    with pytest.raises(ValueError, match="배치 공간 부족"):
        build_components(command, ["slot_1"])
