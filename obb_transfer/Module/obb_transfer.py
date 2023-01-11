#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import open3d as o3d

from obb_transfer.Method.path import createFileFolder, renameFile


class OBBTransfer(object):

    def __init__(self):
        return

    def generateAll(self, pcd_file_path, obb_label_file_path):
        assert os.path.exists(pcd_file_path)

        pcd = o3d.io.read_point_cloud(pcd_file_path)
        o3d.visualization.draw_geometries([pcd])
        return True
