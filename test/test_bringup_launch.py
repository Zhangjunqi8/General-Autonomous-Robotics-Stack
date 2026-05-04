from pathlib import Path


def test_navigation_public_bringup_exists():
    bringup = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    assert bringup.exists()
