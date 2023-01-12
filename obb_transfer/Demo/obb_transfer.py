#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

sys.path.append("../noc-transform")

from obb_transfer.Module.obb_transfer import OBBTransfer


def demo():
    pcd_file_path = "/home/chli/chLi/auto-scan2cad/1314/data/obb/lidar/final_result.pcd"
    obb_label_file_path = "/home/chli/chLi/auto-scan2cad/1314/data/obb/label/final_result.json"
    layout_obb_label_file_path = "/home/chli/chLi/auto-scan2cad/1314/data/obb/label/layout.json"
    save_folder_path = "/home/chli/chLi/auto-scan2cad/1314/obb_info/"
    render = False
    print_progress = True

    obb_transfer = OBBTransfer()
    obb_transfer.generateAll(pcd_file_path, obb_label_file_path,
                             layout_obb_label_file_path, save_folder_path,
                             render, print_progress)
    return True
