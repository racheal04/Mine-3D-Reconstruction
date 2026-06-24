#!/usr/bin/env python3
"""
FAST-LIO2 实时三维建模节点
订阅 /cloud_registered → 累积点云 → 定期 Poisson 重建 → 保存 PLY

用法:
    python3 realtime_mesher.py --interval 30 --voxel 0.1 --depth 9
"""

import rospy
import numpy as np
import open3d as o3d
import os
import time
import threading
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class RealtimeMesher:
    def __init__(self, interval=30, voxel=0.1, depth=9, output_dir="/tmp/meshes"):
        self.interval = interval
        self.voxel = voxel
        self.depth = depth
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.points = []           # accumulated world-frame points
        self.lock = threading.Lock()
        self.last_mesh_time = rospy.Time.now()
        self.mesh_count = 0

        self.sub = rospy.Subscriber("/cloud_registered", PointCloud2,
                                     self.cloud_callback, queue_size=100)
        rospy.loginfo(f"RealtimeMesher: interval={interval}s, voxel={voxel}m, depth={depth}")
        rospy.loginfo(f"  output: {output_dir}/mesh_XXXX.ply")

    def cloud_callback(self, msg):
        # Extract xyz from PointCloud2 (fast, no PCL overhead)
        pts = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)])

        if len(pts) == 0:
            return

        with self.lock:
            self.points.append(pts)

        # Check if it's time to rebuild mesh
        if (rospy.Time.now() - self.last_mesh_time).to_sec() > self.interval:
            self.rebuild_mesh()

    def rebuild_mesh(self):
        with self.lock:
            if len(self.points) == 0:
                return
            all_pts = np.vstack(self.points)
            n_total = len(all_pts)
            self.last_mesh_time = rospy.Time.now()

        t0 = time.time()
        rospy.loginfo(f"[Mesher] Rebuilding from {n_total:,} points...")

        # Build point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts)

        # Downsample
        pcd_ds = pcd.voxel_down_sample(self.voxel)
        rospy.loginfo(f"  voxel downsample: {n_total:,} -> {len(pcd_ds.points):,}")

        # Estimate normals
        radius = self.voxel * 3
        pcd_ds.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        pcd_ds.orient_normals_consistent_tangent_plane(k=30)

        # Poisson reconstruction
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd_ds, depth=self.depth, width=0, scale=1.1, linear_fit=False)
        rospy.loginfo(f"  poisson: {len(mesh.triangles):,} faces")

        # Density-based pruning
        densities = np.asarray(densities)
        thresh = np.quantile(densities, 0.05)
        mesh.remove_vertices_by_mask(densities < thresh)

        # Save
        self.mesh_count += 1
        out_path = os.path.join(self.output_dir, f"mesh_{self.mesh_count:04d}.ply")
        o3d.io.write_triangle_mesh(out_path, mesh)
        file_size = os.path.getsize(out_path) / 1024**2

        elapsed = time.time() - t0
        rospy.loginfo(f"  saved: {out_path} ({file_size:.1f} MB, {elapsed:.1f}s)")
        rospy.loginfo(f"  vertices: {len(mesh.vertices):,}, faces: {len(mesh.triangles):,}")


if __name__ == "__main__":
    rospy.init_node("realtime_mesher")

    interval = rospy.get_param("~interval", 30)   # seconds between rebuilds
    voxel = rospy.get_param("~voxel", 0.1)         # downsample resolution
    depth = rospy.get_param("~depth", 9)            # Poisson depth
    output = rospy.get_param("~output_dir", "/tmp/meshes")

    mesher = RealtimeMesher(interval, voxel, depth, output)
    rospy.spin()
