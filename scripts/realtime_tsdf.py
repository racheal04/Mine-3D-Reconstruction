#!/usr/bin/env python3
"""
FAST-LIO2 实时密集点云输出
不做网格重建，直接保存下采样后的强度着色点云 —— 最稳健

用法:
    python3 realtime_tsdf.py _interval:=15 _voxel:=0.1
"""

import rospy
import numpy as np
import open3d as o3d
import os
import time
import threading
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class RealtimeCloud:
    def __init__(self, interval=15, voxel=0.1, output_dir="/tmp/meshes"):
        self.interval = interval
        self.voxel = voxel
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.points = []      # xyz
        self.intensities = [] # intensity
        self.lock = threading.Lock()
        self.last_time = rospy.Time.now()
        self.count = 0

        self.sub = rospy.Subscriber("/cloud_registered", PointCloud2,
                                     self.cloud_callback, queue_size=100)
        rospy.loginfo(f"RealtimeCloud: interval={interval}s, voxel={voxel}m")

    def cloud_callback(self, msg):
        pts = []; its = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            pts.append([p[0], p[1], p[2]])
            its.append(p[3] if len(p) > 3 else 0)
        if not pts:
            return

        with self.lock:
            self.points.append(np.array(pts))
            self.intensities.append(np.array(its))

        if (rospy.Time.now() - self.last_time).to_sec() > self.interval:
            self.rebuild()

    def rebuild(self):
        with self.lock:
            if len(self.points) < 3:
                return
            all_pts = np.vstack(self.points)
            all_i = np.concatenate(self.intensities)
            self.last_time = rospy.Time.now()

        t0 = time.time()
        rospy.loginfo(f"[Cloud] {len(all_pts):,} points")

        # Downsample with intensity preserved
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts)
        pcd = pcd.voxel_down_sample(self.voxel)

        if len(pcd.points) < 100:
            return

        pts_ds = np.asarray(pcd.points)

        # Map intensity to color
        i_min, i_max = all_i.min(), all_i.max()
        if i_max - i_min < 1:
            i_max = i_min + 1
        # For downsampled points, find nearest intensity (approximate)
        i_norm = np.clip((all_i - i_min) / (i_max - i_min), 0, 1)
        # Downsample intensity the same way as points (take mean per voxel)
        from scipy.spatial import cKDTree
        tree = cKDTree(all_pts)
        _, idx = tree.query(pts_ds, k=1)
        i_ds = i_norm[idx]

        colors = np.zeros((len(pts_ds), 3))
        for j, t in enumerate(i_ds):
            if t < 0.25:
                colors[j] = [0, t*4, 1]
            elif t < 0.5:
                colors[j] = [0, 1, 2-t*4]
            elif t < 0.75:
                colors[j] = [t*4-2, 1, 0]
            else:
                colors[j] = [1, 2-t*4, 0]
        pcd.colors = o3d.utility.Vector3dVector(colors)

        self.count += 1
        out_path = os.path.join(self.output_dir, f"cloud_{self.count:04d}.ply")
        o3d.io.write_point_cloud(out_path, pcd)
        rospy.loginfo(f"  saved: {out_path} ({os.path.getsize(out_path)/1024**2:.1f} MB, {time.time()-t0:.1f}s)")
        rospy.loginfo(f"  {len(pts_ds):,} colored points")


if __name__ == "__main__":
    rospy.init_node("realtime_cloud")
    mesher = RealtimeCloud(
        interval=rospy.get_param("~interval", 15),
        voxel=rospy.get_param("~voxel", 0.1),
        output_dir=rospy.get_param("~output_dir", "/tmp/meshes"))
    rospy.spin()
