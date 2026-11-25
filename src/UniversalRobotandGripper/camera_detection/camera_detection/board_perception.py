"""Measure board occupancy from RGB-D only, without simulator object poses.

Rows A/B/C run from +Y to -Y; columns 1/2/3 run from -X to +X.
Same-colour pieces are interchangeable: IDs are camera tracks, not visual tags.
"""
import itertools
import math

import cv2
import numpy as np


COLORS = {'red': ((0, 10), (170, 179)), 'yellow': ((20, 38),)}


class BoardPerception:
    def __init__(self, center=(0., 0.), cell_size=.055, surface_z=.806, piece_size=.04,
                 staging_first=None, staging_spacing=.055, staging_direction=(1., 0.)):
        if (len(center) != 2 or not all(math.isfinite(v) for v in center)
                or not all(math.isfinite(v) and v > 0 for v in (cell_size, surface_z, piece_size))
                or not piece_size < cell_size
                or not 2 * piece_size**2 > cell_size**2):
            raise ValueError('cells must fit one piece and have less area than two piece footprints')
        self.center = tuple(center)
        self.cell_size, self.surface_z, self.piece_size = cell_size, surface_z, piece_size
        self.centers = {
            f'{letter}{col+1}': (center[0] + (col-1)*cell_size, center[1] + (1-row)*cell_size)
            for row, letter in enumerate('ABC') for col in range(3)
        }
        self.last_positions = {}
        if staging_first is not None:
            self.centers.update({f'D{i+1}': (staging_first[0]+i*staging_spacing*staging_direction[0],
                                           staging_first[1]+i*staging_spacing*staging_direction[1])
                                 for i in range(8)})
        self.position_order = list(self.centers)
        self.piece_names = [f'{color}_{i}' for color in COLORS for i in range(1, 5)]
        self.d_occupancy = [1]*8
        self.piece_locations = {name: [int(cell == f'D{i+1}') for cell in self.position_order]
                                for i, name in enumerate(self.piece_names)}
        if staging_first is not None:
            self.last_positions = {name: np.array([*self.centers[f'D{i+1}'], surface_z+piece_size/2])
                                   for i,name in enumerate(self.piece_names)}
        self.calibration = None
        self.world_rays = None
        self.cell_rois = None

    def update_projection(self, shape, intrinsic, rotation, translation):
        calibration = (shape, tuple(intrinsic), tuple(rotation.ravel()), tuple(translation))
        if calibration == self.calibration:
            return
        v, u = np.indices(shape)
        rays = np.stack(((u-intrinsic[2])/intrinsic[0], (v-intrinsic[5])/intrinsic[4],
                         np.ones(shape)), axis=-1)
        self.world_rays = rays @ rotation.T
        self.cell_rois = {
            cell: self._roi(shape, center, self.cell_size-.002, self.surface_z,
                            intrinsic, rotation, translation)
            for cell, center in self.centers.items()
        }
        self.calibration = calibration

    @staticmethod
    def _roi(shape, center, side, height, intrinsic, rotation, translation):
        corners = np.array([[center[0] + dx*side/2, center[1] + dy*side/2, height]
                            for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        optical = (corners - translation) @ rotation
        result = np.zeros(shape, dtype=np.uint8)
        if np.any(optical[:, 2] <= 0):
            return result.astype(bool)
        pixels = optical[:, :2] / optical[:, 2, None]
        pixels[:, 0] = pixels[:, 0]*intrinsic[0] + intrinsic[2]
        pixels[:, 1] = pixels[:, 1]*intrinsic[4] + intrinsic[5]
        # A cell outside the image is not confirmed empty from a cropped view.
        if (np.any(pixels < 0) or np.any(pixels[:, 0] >= shape[1])
                or np.any(pixels[:, 1] >= shape[0])):
            return result.astype(bool)
        cv2.fillConvexPoly(result, np.rint(pixels).astype(np.int32), 1)
        return result.astype(bool)

    def observe(self, rgb, depth, intrinsic, rotation, translation):
        if (rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.shape[:2] != depth.shape
                or len(intrinsic) != 9 or intrinsic[0] <= 0 or intrinsic[4] <= 0
                or not np.isfinite(intrinsic).all()
                or not np.isfinite(rotation).all() or not np.isfinite(translation).all()):
            raise ValueError('invalid registered RGB-D frame, camera intrinsics or transform')
        self.update_projection(depth.shape, intrinsic, rotation, translation)
        with np.errstate(invalid='ignore'):
            world = self.world_rays * depth[:, :, None] + translation
        valid = np.isfinite(world).all(axis=-1) & np.isfinite(depth) & (depth > 0)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        top_z = self.surface_z + self.piece_size
        on_top = valid & (abs(world[:, :, 2] - top_z) < .006)
        candidates = {cell: [] for cell in self.centers}
        uncertain = set()
        top_masks = {}

        for color, ranges in COLORS.items():
            color_mask = np.zeros(depth.shape, dtype=np.uint8)
            for lo, hi in ranges:
                color_mask |= cv2.inRange(hsv, (lo, 90, 60), (hi, 255, 255))
            color_mask[~on_top] = 0
            top_masks[color] = color_mask > 0
            count, labels, stats, _ = cv2.connectedComponentsWithStats(color_mask)
            for component in range(1, count):
                if stats[component, cv2.CC_STAT_AREA] < 30:
                    continue
                points = world[labels == component]
                (x, y), (sx, sy), degrees = cv2.minAreaRect(points[:, :2].astype(np.float32))
                yaw = math.remainder(math.radians(degrees), math.pi/2)
                half_extent = self.piece_size/2 * (abs(math.cos(yaw)) + abs(math.sin(yaw)))
                touched = [cell for cell, (cx, cy) in self.centers.items()
                           if abs(x-cx) < half_extent + self.cell_size/2
                           and abs(y-cy) < half_extent + self.cell_size/2]
                containing = [cell for cell in touched
                              if abs(x-self.centers[cell][0]) + half_extent <= self.cell_size/2 + .001
                              and abs(y-self.centers[cell][1]) + half_extent <= self.cell_size/2 + .001]
                full_face = self._roi(depth.shape, (x, y), self.piece_size, top_z,
                                      intrinsic, rotation, translation)
                if (len(containing) != 1
                        or not all(.8*self.piece_size <= side <= 1.15*self.piece_size for side in (sx, sy))
                        or full_face.sum() < 30 or len(points) < .65*full_face.sum()):
                    uncertain.update(touched)
                    continue
                candidates[containing[0]].append({
                    'color': color, 'position': [float(x), float(y), float(np.median(points[:, 2])-self.piece_size/2)],
                    'yaw': yaw,
                })

        states, detections = {}, []
        board_surface = valid & (abs(world[:, :, 2]-self.surface_z) < .004)
        any_top = top_masks['red'] | top_masks['yellow']
        for cell, roi in self.cell_rois.items():
            pixels = int(roi.sum())
            visible = int(np.count_nonzero(roi & (board_surface | any_top)))
            entries = candidates[cell]
            if len(entries) > 1:
                states[cell] = 'invalid'
            elif cell in uncertain or pixels < 30 or visible < .6*pixels:
                states[cell] = 'unknown'
            elif entries:
                states[cell] = entries[0]['color']
                detections.append(dict(entries[0], cell=cell))
            elif np.count_nonzero(roi & board_surface) >= .85*pixels:
                states[cell] = 'empty'
            else:
                states[cell] = 'unknown'

        # A fifth piece of the same colour is inconsistent with the eight-piece set.
        for color in COLORS:
            if sum(d['color'] == color for d in detections) > 4:
                for detection in detections:
                    if detection['color'] == color:
                        states[detection['cell']] = 'invalid'
        detections = [d for d in detections if states[d['cell']] in COLORS]
        poses = self._track(detections)
        occupants = {cell: (None if value == 'empty' else value)
                     for cell,value in states.items()}
        positions_valid = {}
        for name in self.piece_names:
            positions_valid[name] = name in poses
            if name in poses:
                xy = poses[name][0][:2]
                cell = min(self.centers, key=lambda c: math.dist(xy, self.centers[c]))
                occupants[cell] = name
                self.piece_locations[name] = [int(c == cell) for c in self.position_order]
        d_valid = []
        for i in range(8):
            value = states.get(f'D{i+1}', 'unknown')
            valid_slot = value in ('red','yellow','empty')
            d_valid.append(valid_slot)
            if valid_slot:
                self.d_occupancy[i] = int(value != 'empty')
        return {
            'cells': states,
            'complete': all(state in ('red', 'yellow', 'empty') for state in states.values()),
            'counts': {state: sum(value == state for value in states.values())
                       for state in ('red', 'yellow', 'empty', 'unknown', 'invalid')},
            'position_order': self.position_order,
            'occupants': occupants,
            'd_occupancy': list(self.d_occupancy), 'd_valid': d_valid,
            'piece_locations': {k:list(v) for k,v in self.piece_locations.items()},
            'piece_valid': positions_valid,
        }, poses

    def _track(self, detections):
        poses = {}
        for color in COLORS:
            current = sorted((d for d in detections if d['color'] == color), key=lambda d: d['cell'])
            names = [f'{color}_{i}' for i in range(1, 5)]
            # At most 4! assignments; reuse the nearest previous same-colour tracks.
            def cost(assignment):
                return sum(np.linalg.norm(np.array(d['position'][:2]) - self.last_positions[name][:2])
                           if name in self.last_positions else 1.
                           for name, d in zip(assignment, current))
            assignment = min(itertools.permutations(names, len(current)), key=cost)
            for name, detection in zip(assignment, current):
                position = detection['position']
                self.last_positions[name] = np.array(position)
                poses[name] = (position, detection['yaw'])
        return poses
