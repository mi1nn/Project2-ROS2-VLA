"""Tests for the MVP inventory decrement policy."""

import pytest

from kit_db.inventory_policy import should_decrement_inventory


@pytest.mark.parametrize(
    'attempts',
    [
        [{'result': 'SUCCESS', 'release': {'success': True}}],
        [{'status': 'SUCCESS', 'release': {'result': 'SUCCESS'}}],
        [{'result': 'SUCCESS'}],
    ],
)
def test_successful_component_with_successful_or_absent_release_decrements(
    attempts,
):
    assert should_decrement_inventory('SUCCESS', attempts) is True


@pytest.mark.parametrize(
    'status,attempts',
    [
        ('FAILED', [{'release': {'success': True}}]),
        ('SKIPPED', [{'release': {'success': True}}]),
        ('SUCCESS', [{'release': {'success': False}}]),
        ('SUCCESS', [{'release': {'result': 'FAILED'}}]),
        ('SUCCESS', []),
    ],
)
def test_unsuccessful_component_or_release_does_not_decrement(
    status, attempts
):
    assert should_decrement_inventory(status, attempts) is False
