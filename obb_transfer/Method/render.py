#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d


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

def renderSceneWithOBB(scene_pcd, obb_list):
    render_list = [scene_pcd]

    for obb in obb_list:
        obb_pcd = getOBBPCD(obb, [255, 0, 0])
        render_list.append(obb_pcd)

    o3d.visualization.draw_geometries(render_list)
    return True
