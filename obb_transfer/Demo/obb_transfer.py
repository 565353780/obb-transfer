#!/usr/bin/env python
# -*- coding: utf-8 -*-

from obb_transfer.Module.obb_transfer import OBBTransfer


def demo():
    pcd_file_path = "/home/chli/chLi/auto-scan2cad/1314/data/obb/lidar/final_result.pcd"
    obb_label_file_path = "/home/chli/chLi/auto-scan2cad/1314/data/obb/label/final_result.json"

    obb_transfer = OBBTransfer()
    obb_transfer.generateAll(pcd_file_path, obb_label_file_path)
    return True
