#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d
from copy import deepcopy

from noc_transform.Method.transform import transPoints


def getOBBPCD(obb, color=None):
    lines = [[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4],
             [0, 4], [1, 5], [2, 6], [3, 7]]

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(
        obb.points),
                                    lines=o3d.utility.Vector2iVector(lines))

    if color is not None:
        colors = np.array([color
                           for i in range(len(lines))], dtype=float) / 255.0
        line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def renderSceneWithOBB(scene_pcd, obb_dict):
    render_list = [scene_pcd]

    for obb_label, obb_info in obb_dict.items():
        obb = obb_info['obb']
        obb_pcd = getOBBPCD(obb, [255, 0, 0])
        render_list.append(obb_pcd)

    o3d.visualization.draw_geometries(render_list)
    return True


def renderObjectWithOBB(obb_dict):
    render_list = []

    for obb_label, obb_info in obb_dict.items():
        object_pcd = obb_info['object_pcd']
        render_list.append(object_pcd)

        obb = obb_info['obb']
        obb_pcd = getOBBPCD(obb, [255, 0, 0])
        render_list.append(obb_pcd)

    o3d.visualization.draw_geometries(render_list)
    return True


def renderNOCObjectWithOBB(obb_dict):
    render_list = []

    obb_num = len(list(obb_dict.keys()))
    row_num = np.sqrt(obb_num)

    row_idx = 0
    col_idx = 0
    delta_translate = 1
    for obb_label, obb_info in obb_dict.items():
        object_pcd = deepcopy(obb_info['object_pcd'])
        obb = obb_info['obb']
        noc_trans_matrix = obb_info['noc_trans_matrix']

        obb_pcd = getOBBPCD(obb, [255, 0, 0])
        noc_obb_pcd = getOBBPCD(noc_obb, [0, 255, 0])

        points = np.array(object_pcd.points)
        points = transPoints(points, noc_trans_matrix)
        object_pcd.points = o3d.utility.Vector3dVector(points)

        points = np.array(obb_pcd.points)
        points = transPoints(points, noc_trans_matrix)
        obb_pcd.points = o3d.utility.Vector3dVector(points)

        delta_translate = [
            row_idx * delta_translate, col_idx * delta_translate, 0
        ]
        object_pcd.translate(delta_translate)
        obb_pcd.translate(delta_translate)

        render_list.append(object_pcd)
        render_list.append(obb_pcd)

        row_idx += 1
        if row_idx == row_num:
            row_idx = 0
            col_idx += 1

    o3d.visualization.draw_geometries(render_list)
    return True
