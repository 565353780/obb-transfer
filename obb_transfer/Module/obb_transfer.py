#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import numpy as np
import open3d as o3d
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

from noc_transform.Data.obb import OBB

from obb_transfer.Method.path import createFileFolder, renameFile
from obb_transfer.Method.render import renderSceneWithOBB, renderObjectWithOBB


class OBBTransfer(object):

    def __init__(self):
        self.scene_pcd = None
        self.obb_dict = {}
        return

    def reset(self):
        self.scene_pcd = None
        self.obb_dict = {}
        return True

    def loadScenePCD(self, scene_pcd_file_path):
        assert os.path.exists(scene_pcd_file_path)

        self.scene_pcd = o3d.io.read_point_cloud(scene_pcd_file_path)
        return True

    def loadOBB(self, obb_label_file_path, print_progress=False):
        assert os.path.exists(obb_label_file_path)

        with open(obb_label_file_path, 'r') as f:
            obb_label_dict_list = json.load(f)

        for_data = obb_label_dict_list
        if print_progress:
            print("[INFO][OBBTransfer::loadOBB]")
            print("\t start load obb labels...")
            for_data = tqdm(for_data)
        for obb_label_dict in for_data:
            obj_id = obb_label_dict['obj_id']
            obj_type = obb_label_dict['obj_type']
            pose_dict = obb_label_dict['psr']

            position_dict = pose_dict['position']
            rotation_dict = pose_dict['rotation']
            scale_dict = pose_dict['scale']

            position = np.array(
                [position_dict['x'], position_dict['y'], position_dict['z']])

            rotation = -np.array(
                [rotation_dict['x'], rotation_dict['y'], rotation_dict['z']])

            scale = np.array(
                [scale_dict['x'], scale_dict['y'], scale_dict['z']])

            min_point = -0.5 * scale
            max_point = 0.5 * scale
            obb = OBB.fromABBPoints(min_point, max_point)

            r = R.from_euler('xyz', rotation)
            rotate_matrix = r.as_matrix()

            points = obb.points
            rotate_points = points @ rotate_matrix
            trans_points = rotate_points + position
            obb.points = trans_points

            self.obb_dict[obj_id] = {'class': obj_type, 'obb': obb}
        return True

    def generateObjectPCD(self, print_progress=False):
        for_data = self.obb_dict.items()
        if print_progress:
            print("[INFO][OBBTransfer::generateObjectPCD]")
            print("\t start generate pcd for objects...")
            for_data = tqdm(for_data)
        for obb_label, obb_info in for_data:
            obb = obb_info['obb']
            o3d_obb = o3d.geometry.OrientedBoundingBox.create_from_points(
                o3d.utility.Vector3dVector(obb.points))
            #  self.obb_dict[obb_label]['o3d_obb'] = o3d_obb

            object_pcd = self.scene_pcd.crop(o3d_obb)
            self.obb_dict[obb_label]['object_pcd'] = object_pcd
        return True

    def saveAll(self, save_folder_path):
        return True

    def generateAll(self,
                    pcd_file_path,
                    obb_label_file_path,
                    save_folder_path=None,
                    render=False,
                    print_progress=False):
        self.reset()

        self.loadOBB(obb_label_file_path, print_progress)
        self.loadScenePCD(pcd_file_path)

        self.generateObjectPCD(print_progress)

        if render:
            renderSceneWithOBB(self.scene_pcd, self.obb_dict)
            renderObjectWithOBB(self.obb_dict)

        if save_folder_path is not None:
            self.saveAll(save_folder_path)
        return True
