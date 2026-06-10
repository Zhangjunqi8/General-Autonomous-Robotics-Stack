"""Manifest-level dependency contracts for hanmole_navigation."""

from pathlib import Path
import xml.etree.ElementTree as ET


def test_package_manifest_declares_rpp_and_navfn_runtime_dependencies():
    manifest_path = Path(__file__).resolve().parents[1] / 'package.xml'
    root = ET.fromstring(manifest_path.read_text(encoding='utf-8'))

    exec_dep_names = [element.text for element in root.findall('exec_depend')]

    assert 'nav2_regulated_pure_pursuit_controller' in exec_dep_names
    assert 'nav2_navfn_planner' in exec_dep_names
