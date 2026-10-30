#!/usr/bin/env python3
"""Derive the optional CAD visual from the current robot and lake world."""

from pathlib import Path
import xml.etree.ElementTree as ET


def synchronize(package=None):
    """Keep CAD physics, sonar mounts and terrain identical to the default."""
    package = Path(package) if package else Path(__file__).resolve().parents[1] / 'src/pobs_gazebo'
    tree = ET.parse(package / 'models/pobs/model.sdf')
    model = tree.getroot().find('model')
    model.set('name', 'pobs_cad')
    visual = model.find("link[@name='base_link']/visual[@name='visual']")
    visual.clear()
    visual.set('name', 'visual')
    ET.SubElement(visual, 'pose').text = (
        '-0.4042445 -0.2838934 -0.1204642 1.57079632679 0 1.57079632679')
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = 'model://pobs_cad/meshes/Chassis.STL'
    ET.SubElement(mesh, 'scale').text = '0.001 0.001 0.001'
    ET.indent(tree, space='  ')
    tree.write(package / 'models/pobs_cad/model.sdf', encoding='unicode', xml_declaration=True)
    world = ET.parse(package / 'worlds/lake.sdf')
    for include in world.getroot().findall('world/include'):
        if include.findtext('name') == 'pobs':
            include.find('uri').text = 'model://pobs_cad'
    ET.indent(world, space='  ')
    world.write(package / 'worlds/lake_cad.sdf', encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    synchronize()
