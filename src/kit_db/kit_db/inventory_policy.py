"""Rules for applying component results to PostgreSQL inventory."""


def should_decrement_inventory(status, attempts):
    """Return whether one inventory item should be consumed."""
    if status != 'SUCCESS' or not attempts:
        return False

    final_attempt = attempts[-1]
    attempt_status = final_attempt.get('result', final_attempt.get('status'))
    if attempt_status != 'SUCCESS':
        return False

    release = final_attempt.get('release')
    if release is None:
        return True

    release_status = release.get('success', release.get('result'))
    return release_status is True or release_status == 'SUCCESS'
