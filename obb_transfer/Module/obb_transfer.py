#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
from copy import deepcopy

import numpy as np
import open3d as o3d
from noc_transform.Data.obb import OBB
from noc_transform.Module.transform_generator import TransformGenerator
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from obb_transfer.Method.path import createFileFolder, removeFile, renameFile
from obb_transfer.Method.render import (renderNOCObjectWithOBB,
                                        renderObjectWithOBB,
                                        renderSceneWithOBB)


class OBBTransfer(object):

    def __init__(self):
        self.transform_generator = TransformGenerator()

        self.scene_pcd = None
        self.obb_dict = {}
        self.layout_obb_dict = {}
        return

    def reset(self):
        self.scene_pcd = None
        self.obb_dict = {}
        self.layout_obb_dict = {}
        return True

    def loadScenePCD(self, scene_pcd_file_path):
        assert os.path.exists(scene_pcd_file_path)

        self.scene_pcd = o3d.io.read_point_cloud(scene_pcd_file_path)
        return True

    def getOBBDict(self, obb_label_file_path, print_progress=False):
        assert os.path.exists(obb_label_file_path)

        obb_dict = {}

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

            noc_trans_matrix = self.transform_generator.getNOCTransform(obb)

            obb_dict[obj_id] = {
                'class': obj_type,
                'obb': obb,
                'noc_trans_matrix': noc_trans_matrix,
            }
        return obb_dict

    def loadObjectOBB(self, obb_label_file_path, print_progress=False):
        self.obb_dict = self.getOBBDict(obb_label_file_path, print_progress)
        return True

    def loadLayoutOBB(self, layout_obb_label_file_path, print_progress=False):
        self.layout_obb_dict = self.getOBBDict(layout_obb_label_file_path,
                                               print_progress)
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

    def saveObjectPCD(self, obb_label, save_object_file_path):
        assert obb_label in self.obb_dict.keys()

        obb_info = self.obb_dict[obb_label]

        tmp_save_object_file_path = save_object_file_path[:-4] + "_tmp.pcd"
        createFileFolder(tmp_save_object_file_path)

        object_pcd = obb_info['object_pcd']
        o3d.io.write_point_cloud(tmp_save_object_file_path,
                                 object_pcd,
                                 write_ascii=True)

        renameFile(tmp_save_object_file_path, save_object_file_path)
        return True

    def saveOBBTransMatrix(self, obb_dict, obb_label,
                           save_obb_trans_matrix_file_path):
        assert obb_label in obb_dict.keys()

        obb_info = obb_dict[obb_label]

        noc_trans_matrix = obb_info['noc_trans_matrix']

        tmp_save_obb_trans_matrix_file_path = save_obb_trans_matrix_file_path[:
                                                                              -5] + "_tmp.json"
        createFileFolder(tmp_save_obb_trans_matrix_file_path)

        noc_trans_matrix_dict = {'noc_trans_matrix': noc_trans_matrix.tolist()}
        with open(tmp_save_obb_trans_matrix_file_path, 'w') as f:
            json.dump(noc_trans_matrix_dict, f, indent=4)

        renameFile(tmp_save_obb_trans_matrix_file_path,
                   save_obb_trans_matrix_file_path)
        return True

    def saveObjectOBBTransMatrix(self, obb_label,
                                 save_obb_trans_matrix_file_path):
        return self.saveOBBTransMatrix(self.obb_dict, obb_label,
                                       save_obb_trans_matrix_file_path)

    def saveLayoutOBBTransMatrix(self, obb_label,
                                 save_obb_trans_matrix_file_path):
        return self.saveOBBTransMatrix(self.layout_obb_dict, obb_label,
                                       save_obb_trans_matrix_file_path)

    def saveObjectOBB(self, obb_label, save_folder_path):
        assert obb_label in self.obb_dict.keys()

        save_object_file_path = save_folder_path + "object/" + obb_label + ".pcd"
        save_obb_trans_matrix_file_path = save_folder_path + "obb_trans_matrix/" + obb_label + ".json"

        if os.path.exists(save_object_file_path) and os.path.exists(
                save_obb_trans_matrix_file_path):
            return True

        removeFile(save_object_file_path)
        removeFile(save_obb_trans_matrix_file_path)

        self.saveObjectPCD(obb_label, save_object_file_path)
        self.saveLayoutOBBTransMatrix(obb_label,
                                      save_obb_trans_matrix_file_path)
        return True

    def saveLayoutOBB(self, obb_label, save_folder_path):
        assert obb_label in self.layout_obb_dict.keys()

        save_layout_file_path = save_folder_path + "layout/" + obb_label + ".pcd"
        save_obb_trans_matrix_file_path = save_folder_path + "layout_obb_trans_matrix/" + obb_label + ".json"

        if os.path.exists(save_layout_file_path) and os.path.exists(
                save_obb_trans_matrix_file_path):
            return True

        removeFile(save_layout_file_path)
        removeFile(save_obb_trans_matrix_file_path)

        self.saveLayoutOBBTransMatrix(obb_label,
                                      save_obb_trans_matrix_file_path)
        return True

    def saveAllObjects(self, save_folder_path, print_progress=False):
        os.makedirs(save_folder_path, exist_ok=True)

        for_data = self.obb_dict.keys()
        if print_progress:
            print("[INFO][OBBTransfer::saveAllObjects]")
            print("\t start save all obb info...")
            for_data = tqdm(for_data)
        for obb_label in for_data:
            self.saveObjectOBB(obb_label, save_folder_path)
        return True

    def saveAllLayouts(self, save_folder_path, print_progress=False):
        os.makedirs(save_folder_path, exist_ok=True)

        for_data = self.layout_obb_dict.keys()
        if print_progress:
            print("[INFO][OBBTransfer::saveAllLayouts]")
            print("\t start save all obb info...")
            for_data = tqdm(for_data)
        for obb_label in for_data:
            self.saveLayoutOBB(obb_label, save_folder_path)
        return True

    def generateAll(self,
                    pcd_file_path,
                    obb_label_file_path,
                    layout_obb_label_file_path,
                    save_folder_path=None,
                    render=False,
                    print_progress=False):
        self.reset()

        self.loadObjectOBB(obb_label_file_path, print_progress)
        self.loadLayoutOBB(layout_obb_label_file_path, print_progress)
        self.loadScenePCD(pcd_file_path)

        self.generateObjectPCD(print_progress)

        if render:
            renderSceneWithOBB(self.scene_pcd, self.obb_dict)
            renderObjectWithOBB(self.obb_dict)
            renderNOCObjectWithOBB(self.obb_dict)

        if save_folder_path is not None:
            self.saveAllObjects(save_folder_path, print_progress)
            self.saveAllLayouts(save_folder_path, print_progress)
        return True
