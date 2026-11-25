"""Check staging geometry independently of the detector."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[2]
CONFIG=yaml.safe_load((ROOT/'ur_task_planner/config/scene_depth.yaml').read_text())['/**']['ros__parameters']

@pytest.mark.parametrize('filename',['ur_table.sdf','ur_table_depth.sdf'])
def test_eight_staging_slots_fit_table_and_robot_reach(filename):
    world=ET.parse(ROOT/'ur_simulation_gz/world'/filename).getroot().find('world')
    names=[f'{c}_{i}' for c in ('red','yellow') for i in range(1,5)]
    positions=[]
    for i,name in enumerate(names):
        xyz=list(map(float,world.find(f"model[@name='{name}']/pose").text.split()))[:3]
        zone=CONFIG['zones'][f'D{i+1}']
        assert xyz==pytest.approx([*zone['position'][:2],.826])
        assert xyz==CONFIG['objects'][name]['position']
        x,y,_=xyz
        assert x+.02 < -.1125  # entirely outside board backing
        radius=math.hypot(.04,.04)/2
        reach=math.hypot(x+.35,y)
        assert .12+radius < reach < .56-radius
        assert -.5 < x-.02 < x+.02 < .5
        assert -.4 < y-.02 < y+.02 < .4
        positions.append(xyz)
    assert len(set(p[0] for p in positions))==1
    assert all(b[1]-a[1]==pytest.approx(-.055) for a,b in zip(positions,positions[1:]))
    assert len(CONFIG['zones'])==17

def test_camera_footprint_includes_all_slot_corners():
    world=ET.parse(ROOT/'ur_simulation_gz/world/ur_table_depth.sdf').getroot().find('world')
    camera=world.find("model[@name='depth_camera']")
    x,y,z,*_=map(float,camera.findtext('pose').split())
    hfov=float(camera.findtext('link/sensor/camera/horizontal_fov'))
    half_width=(z-.846)*math.tan(hfov/2)
    for zone in CONFIG['zones'].values():
        assert abs(zone['position'][0]-x)+.0275 < half_width
        assert abs(zone['position'][1]-y)+.0275 < half_width*.75


def test_camera_collision_is_above_the_arm_and_gripper():
    world = ET.parse(ROOT/'ur_simulation_gz/world/ur_table_depth.sdf').getroot().find('world')
    camera = world.find("model[@name='depth_camera']")
    x, y, z, roll, pitch, yaw = map(float, camera.findtext('pose').split())
    assert (roll, pitch, yaw) == pytest.approx((0, math.pi/2, math.pi/2))
    collision = camera.find("link/collision[@name='camera_body_collision']")
    local_x, *_ = map(float, collision.findtext('pose').split())
    local_length, local_width, local_height = map(float, collision.findtext('geometry/box/size').split())
    # The mounting rotation maps the box's X/Y/Z axes to world -Z/-X/+Y.
    padded_box = CONFIG['obstacles']['depth_camera']
    assert 'depth_camera' in CONFIG['obstacle_names']
    assert padded_box['position'] == pytest.approx((x, y, z-local_x))
    assert all(padded >= physical for padded, physical in
               zip(padded_box['size'], (local_width, local_height, local_length)))

    kinematics = yaml.safe_load(
        (ROOT/'ur_onrobot/config/ur3e/default_kinematics.yaml').read_text())['kinematics']
    arm_length = sum(math.hypot(joint['x'], joint['y'], joint['z'])
                     for joint in kinematics.values() if isinstance(joint, dict))
    # Include the 0.20 m gripper and an additional 0.05 m for collision geometry.
    highest_reach = CONFIG['robot_base_position'][2] + arm_length + .25
    assert padded_box['position'][2] - padded_box['size'][2]/2 > highest_reach + .10
