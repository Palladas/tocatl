import copy
import json
from pathlib import Path

import pytest

from tocatl_kinematics.geometry import geometry_from_dict
from tocatl_kinematics.joint_map import joint_map_from_dict

REPO = Path(__file__).resolve().parents[3]


@pytest.fixture
def geometry_data():
    return json.loads((REPO / 'config' / 'robot-geometry.json').read_text())


@pytest.fixture
def geometry(geometry_data):
    return geometry_from_dict(geometry_data)


@pytest.fixture
def joint_map_data():
    return json.loads((REPO / 'config' / 'joint-map.json').read_text())


@pytest.fixture
def joint_map(joint_map_data):
    return joint_map_from_dict(joint_map_data)


@pytest.fixture
def wide_geometry(geometry_data):
    """Seeded geometry with joint limits opened, for testing the math independently of limits."""
    data = copy.deepcopy(geometry_data)
    data['defaults']['limits_rad'] = {'hip_yaw': [-3.2, 3.2], 'hip_pitch': [-3.2, 3.2], 'knee': [-3.2, 3.2]}
    return geometry_from_dict(data)
