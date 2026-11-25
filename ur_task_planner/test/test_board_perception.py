"""Use rendered RGB-D geometry, including four separate pieces of each colour."""
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from board_perception import BoardPerception
from plan_validator import PlanValidationError, PlanValidator, SceneState
from scene_geometry import TableWorkspace

LAYOUT = {'A1':'red', 'A2':'yellow', 'A3':'red', 'B1':'yellow',
          'B2':'empty', 'B3':'red', 'C1':'yellow', 'C2':'red', 'C3':'yellow'}


def render(layout=None):
    board = BoardPerception()
    layout = layout or LAYOUT
    rotation, translation = np.diag([1.,-1.,-1.]), np.array([0.,0.,1.4])
    focal = 640/(2*math.tan(1.1/2))
    intrinsic = [focal,0.,320.,0.,focal,240.,0.,0.,1.]
    rgb = np.full((480,640,3), 130, dtype=np.uint8)
    depth = np.full((480,640), 1.4-.8)

    def polygon(center, side, height):
        points = np.array([[center[0]+dx*side/2,center[1]+dy*side/2,height]
                           for dx,dy in ((-1,-1),(1,-1),(1,1),(-1,1))])
        optical = (points-translation)@rotation
        pixels = optical[:,:2]/optical[:,2,None]*focal + [320,240]
        mask = np.zeros(depth.shape,dtype=np.uint8)
        cv2.fillConvexPoly(mask,np.rint(pixels).astype(np.int32),1)
        return mask == 1

    surface = polygon((0,0),.225,.806)
    depth[surface] = 1.4-.806
    for cell,color in layout.items():
        if color == 'empty':
            continue
        top = polygon(board.centers[cell],.04,.846)
        rgb[top] = [255,0,0] if color == 'red' else [255,255,0]
        depth[top] = 1.4-.846
    return rgb,depth,intrinsic,rotation,translation,polygon


def test_all_eight_pieces_of_repeated_colours_are_detected():
    board = BoardPerception()
    *args,_ = render()
    state,poses = board.observe(*args)
    assert state['cells'] == LAYOUT
    assert state['complete']
    assert state['counts'] == {'red':4,'yellow':4,'empty':1,'unknown':0,'invalid':0}
    assert set(poses) == {f'{color}_{i}' for color in ('red','yellow') for i in range(1,5)}
    assert poses['red_1'][0] == pytest.approx([-.055,.055,.826],abs=.002)


@pytest.mark.parametrize('occlusion', ['nan','robot'])
def test_hidden_cell_is_unknown_instead_of_empty(occlusion):
    board = BoardPerception()
    rgb,depth,k,rot,trans,polygon = render()
    hidden = polygon(board.centers['B2'],.055,.806)
    depth[hidden] = np.nan if occlusion == 'nan' else .35
    state,_ = board.observe(rgb,depth,k,rot,trans)
    assert state['cells']['B2'] == 'unknown'
    assert not state['complete']


def test_tracks_follow_a_piece_moved_into_the_empty_cell():
    board = BoardPerception()
    *args,_ = render()
    board.observe(*args)
    moved = dict(LAYOUT,A1='empty',B2='red')
    *args,_ = render(moved)
    state,poses = board.observe(*args)
    assert state['cells'] == moved
    assert poses['red_1'][0][:2] == pytest.approx([0,0],abs=.002)


def test_invalid_depth_never_confirms_empty_board():
    board = BoardPerception()
    rgb,depth,k,rot,trans,_ = render()
    depth[:] = np.nan
    state,poses = board.observe(rgb,depth,k,rot,trans)
    assert set(state['cells'].values()) == {'unknown'}
    assert poses == {}


def test_cell_capacity_and_adjacent_zone_validation():
    board = BoardPerception()
    zones = {cell:{'position':[*xy,.806],'size':[.055,.055]} for cell,xy in board.centers.items()}
    workspace = TableWorkspace([-.25,.2,-.34,.34],[-.35,0,.81],zones,
                               {'red_1':[.04]*3,'yellow_1':[.04]*3},board_mode=True)
    state = SceneState(locations={'red_1':'A1','yellow_1':'A2'},
                       positions={'red_1':[-.055,.055,.826],'yellow_1':[0,.055,.826]},observed=True)
    assert state.occupants('A2',workspace) == ['yellow_1']
    assert state.occupants('B2',workspace) == []
    validator = PlanValidator(workspace.sizes,zones,workspace=workspace)
    with pytest.raises(PlanValidationError,match='occupied'):
        validator.validate({'plan':[{'skill':'pick','object':'red_1'},
                                   {'skill':'place','object':'red_1','zone':'A2'}]},state)
    assert validator.validate({'plan':[{'skill':'pick','object':'red_1'},
                                      {'skill':'place','object':'red_1','zone':'B2'}]},state)


def staging_frame():
    rgb,depth,k,rot,trans,polygon=render({cell:'empty' for cell in LAYOUT})
    depth[:]=1.4-.806
    board=BoardPerception(staging_first=(-.17,.1925), staging_direction=(0.,-1.))
    for i in range(8):
        mask=polygon(board.centers[f'D{i+1}'],.04,.846)
        rgb[mask]=[255,0,0] if i<4 else [255,255,0]
        depth[mask]=1.4-.846
    return board,rgb,depth,k,rot,trans,polygon


def test_staging_initial_arrays_and_detected_seventeen_slots():
    board,rgb,depth,k,rot,trans,_=staging_frame()
    state,poses=board.observe(rgb,depth,k,rot,trans)
    assert state['complete']
    assert state['d_occupancy']==[1]*8
    assert state['d_valid']==[True]*8
    assert len(state['position_order'])==17
    assert all(state['cells'][cell]=='empty' for cell in LAYOUT)
    for i,name in enumerate(board.piece_names):
        assert state['piece_locations'][name]==[int(j==9+i) for j in range(17)]
        assert state['piece_valid'][name]


def test_staging_piece_movement_and_occlusion_preserve_identity_and_validity():
    board,rgb,depth,k,rot,trans,polygon=staging_frame()
    board.observe(rgb,depth,k,rot,trans)
    mask=polygon(board.centers['D1'],.045,.846)
    depth[mask]=1.4-.806
    rgb[mask]=130
    target=polygon(board.centers['B2'],.04,.846)
    rgb[target]=[255,0,0];depth[target]=1.4-.846
    state,_=board.observe(rgb,depth,k,rot,trans)
    assert state['d_occupancy']==[0]+[1]*7
    assert state['piece_locations']['red_1'][4]==1
    depth[:]=np.nan
    hidden,_=board.observe(rgb,depth,k,rot,trans)
    assert hidden['d_occupancy']==state['d_occupancy']
    assert hidden['d_valid']==[False]*8
    assert not any(hidden['piece_valid'].values())
