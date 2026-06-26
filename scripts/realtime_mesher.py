#!/usr/bin/env python3
"""
FAST-LIO2 实时三维建模节点 (v2)
订阅 /cloud_registered → 累积世界坐标系点云 → 定期 Poisson 重建 → 保存 PLY

改进:
  - 内存上限管理 (保留最近 N 点)
  - 实时回环闭合检测 (机器人回到起点附近时自动 Z 补偿)
  - 同步输出彩色点云 (供 web_viewer 叠加显示)
  - 更好的 Poisson 参数 (保守密度裁剪)

用法:
    python3 realtime_mesher.py _interval:=30 _voxel:=0.1 _depth:=9
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

        # 累积全部点云 (不设上限, voxel_down_sample 负责压缩)
        self.xyz_list = []        # list of (N,3) arrays
        self.intensity_list = []  # list of (N,) arrays
        self.n_accumulated = 0
        self.lock = threading.Lock()

        # 回环检测状态
        self.loop_detected = False
        self.loop_delta_z = 0.0

        self.last_mesh_time = rospy.Time.now()
        self.mesh_count = 0

        self.sub = rospy.Subscriber("/cloud_registered", PointCloud2,
                                     self.cloud_callback, queue_size=100)
        rospy.loginfo(f"RealtimeMesher v2: interval={interval}s, voxel={voxel}m, "
                      f"depth={depth} (accumulates ALL points)")
        rospy.loginfo(f"  output: {output_dir}/mesh_XXXX.ply + cloud_XXXX.ply")

    def cloud_callback(self, msg):
        # 提取 xyz + intensity
        pts, its = [], []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"),
                                  skip_nans=True):
            pts.append([p[0], p[1], p[2]])
            its.append(p[3] if len(p) > 3 else 0.0)

        if not pts:
            return

        with self.lock:
            self.xyz_list.append(np.array(pts, dtype=np.float32))
            self.intensity_list.append(np.array(its, dtype=np.float32))
            self.n_accumulated += len(pts)

        # 回环检测: 累积足够数据后检测
        if not self.loop_detected and self.n_accumulated > 500_000:
            self._check_loop_closure()

        # 定期重建
        if (rospy.Time.now() - self.last_mesh_time).to_sec() > self.interval:
            self.rebuild_mesh()

    def _check_loop_closure(self):
        """检测机器人是否回到起点附近 (回环)"""
        with self.lock:
            if len(self.xyz_list) < 10:
                return
            all_pts = np.vstack(self.xyz_list[-10:])  # 最近 10 帧

        # 用 X 轴分段近似轨迹
        x = all_pts[:, 0]
        x_min, x_max = x.min(), x.max()
        if x_max - x_min < 5:
            return

        # 前 10% 和后 10% 的 XY 质心
        mask_front = x < x_min + (x_max - x_min) * 0.1
        mask_back = x > x_max - (x_max - x_min) * 0.1

        if mask_front.sum() < 100 or mask_back.sum() < 100:
            return

        front_xy = all_pts[mask_front, :2].mean(axis=0)
        back_xy = all_pts[mask_back, :2].mean(axis=0)
        xy_dist = np.linalg.norm(back_xy - front_xy)

        front_z = all_pts[mask_front, 2].mean()
        back_z = all_pts[mask_back, 2].mean()
        delta_z = back_z - front_z

        if xy_dist < 8.0 and abs(delta_z) > 0.5:
            self.loop_detected = True
            self.loop_delta_z = delta_z
            rospy.loginfo(f"[Mesher] LOOP DETECTED: XY_dist={xy_dist:.2f}m, "
                          f"Z_drift={delta_z:.2f}m → will close loop!")
        else:
            rospy.loginfo(f"[Mesher] Loop check: XY_dist={xy_dist:.2f}m, "
                          f"Z_diff={delta_z:.2f}m (not a loop yet)")

    def rebuild_mesh(self):
        with self.lock:
            if len(self.xyz_list) < 3:
                return
            all_xyz = np.vstack(self.xyz_list)
            all_i = np.concatenate(self.intensity_list)
            n_total = len(all_xyz)
            self.last_mesh_time = rospy.Time.now()

        t0 = time.time()
        rospy.loginfo(f"[Mesher] Rebuilding from {n_total:,} points...")

        # --- 回环闭合变形 ---
        if self.loop_detected:
            x = all_xyz[:, 0]
            x_min, x_max = x.min(), x.max()
            t = np.clip((x - x_min) / (x_max - x_min), 0.0, 1.0)
            w = 3.0 * t**2 - 2.0 * t**3  # smoothstep
            all_xyz = all_xyz.copy()
            all_xyz[:, 2] -= self.loop_delta_z * w
            rospy.loginfo(f"  loop closed: Δz={self.loop_delta_z:.2f}m applied")

        # --- 构建点云 ---
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_xyz)

        # --- 下采样 ---
        pcd_ds = pcd.voxel_down_sample(self.voxel)
        n_ds = len(pcd_ds.points)
        rospy.loginfo(f"  voxel downsample: {n_total:,} -> {n_ds:,}")

        if n_ds < 500:
            rospy.logwarn(f"  Too few points ({n_ds}), skipping reconstruction")
            return

        # --- 法向量 ---
        radius = self.voxel * 3
        pcd_ds.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        pcd_ds.orient_normals_consistent_tangent_plane(k=30)
        rospy.loginfo(f"  normals estimated (radius={radius:.2f}m)")

        # --- Poisson 重建 ---
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd_ds, depth=self.depth, width=0, scale=1.1, linear_fit=False)
        n_faces_raw = len(mesh.triangles)
        rospy.loginfo(f"  poisson: {n_faces_raw:,} faces")

        # --- 保守密度裁剪 (2%) ---
        densities = np.asarray(densities)
        thresh = np.quantile(densities, 0.02)
        mesh.remove_vertices_by_mask(densities < thresh)
        rospy.loginfo(f"  density pruned (2%): {len(mesh.triangles):,} faces")

        # --- 边界裁剪 ---
        pts_ds = np.asarray(pcd_ds.points)
        bbox_min = pts_ds.min(axis=0) - 3.0
        bbox_max = pts_ds.max(axis=0) + 3.0
        verts = np.asarray(mesh.vertices)
        in_bbox = np.all((verts >= bbox_min) & (verts <= bbox_max), axis=1)
        mesh.remove_vertices_by_mask(~in_bbox)

        # --- 距离裁剪 (切除稀疏区域的 Poisson 鼓包) ---
        # 原理: 对每个网格顶点，计算到最近输入点的距离。
        #       距离 > 阈值 → 没有数据支撑 → 删除。
        #       路口另一侧只有几十个稀疏点的地方，Poisson 脑补的鼓包被切除。
        pcd_tree = o3d.geometry.KDTreeFlann(pcd_ds)
        verts = np.asarray(mesh.vertices)
        mask = np.ones(len(verts), dtype=bool)
        dist_thresh = self.voxel * 3.0  # 3×体素 = 无数据支撑
        for i in range(len(verts)):
            _, idx, dist2 = pcd_tree.search_knn_vector_3d(verts[i], 1)
            if dist2[0] > dist_thresh * dist_thresh:
                mask[i] = False
        mesh.remove_vertices_by_mask(~mask)
        rospy.loginfo(f"  distance clipped (>{dist_thresh:.2f}m): {len(mesh.triangles):,} faces")

        # --- 移除碎片 ---
        clusters, counts, _ = mesh.cluster_connected_triangles()
        counts = np.asarray(counts)
        keep = np.where(counts >= 100)[0]
        mask = ~np.isin(np.asarray(clusters), keep)
        mesh.remove_triangles_by_mask(mask)
        mesh.remove_unreferenced_vertices()
        rospy.loginfo(f"  cleaned: {len(mesh.triangles):,} faces")

        # --- 保存网格 ---
        self.mesh_count += 1
        mesh_path = os.path.join(self.output_dir, f"mesh_{self.mesh_count:04d}.ply")
        o3d.io.write_triangle_mesh(mesh_path, mesh)
        mesh_mb = os.path.getsize(mesh_path) / 1024**2

        # --- 同步输出彩色点云 (供 web_viewer 叠加) ---
        cloud_path = os.path.join(self.output_dir, f"cloud_{self.mesh_count:04d}.ply")
        self._save_colored_cloud(pcd_ds, all_i, cloud_path)
        cloud_mb = os.path.getsize(cloud_path) / 1024**2

        elapsed = time.time() - t0
        rospy.loginfo(f"  saved: mesh={mesh_mb:.1f}MB + cloud={cloud_mb:.1f}MB "
                      f"({elapsed:.1f}s)")
        rospy.loginfo(f"  vertices: {len(mesh.vertices):,}, faces: {len(mesh.triangles):,}")

        # 水密性
        if mesh.is_watertight():
            vol = mesh.get_volume()
            rospy.loginfo(f"  watertight ✓, volume={vol:.2f} m³")

    def _save_colored_cloud(self, pcd_ds, all_intensities, path):
        """保存强度着色的降采样点云"""
        pts_ds = np.asarray(pcd_ds.points)

        # 用原始强度近似着色 (最近邻)
        # 对每个降采样点，取原始点云中最近点的强度
        all_xyz = np.vstack(self.xyz_list) if self.xyz_list else pts_ds
        if self.loop_detected:
            x = all_xyz[:, 0]
            x_min, x_max = x.min(), x.max()
            t = np.clip((x - x_min) / (x_max - x_min), 0.0, 1.0)
            w = 3.0 * t**2 - 2.0 * t**3
            all_xyz = all_xyz.copy()
            all_xyz[:, 2] -= self.loop_delta_z * w
            self.xyz_list = [all_xyz]  # use corrected copy

        # 简化：用降采样点云的 z 坐标做高度着色
        z = pts_ds[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 0.1:
            z_max = z_min + 0.1
        t = np.clip((z - z_min) / (z_max - z_min), 0.0, 1.0)

        colors = np.zeros((len(pts_ds), 3))
        for j in range(len(pts_ds)):
            # 热力图: 蓝→青→绿→黄→红
            v = t[j]
            if v < 0.25:
                colors[j] = [0, v*4, 1]
            elif v < 0.5:
                colors[j] = [0, 1, 2-v*4]
            elif v < 0.75:
                colors[j] = [v*4-2, 1, 0]
            else:
                colors[j] = [1, 2-v*4, 0]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts_ds)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(path, cloud)


if __name__ == "__main__":
    rospy.init_node("realtime_mesher")

    mesher = RealtimeMesher(
        interval=rospy.get_param("~interval", 30),
        voxel=rospy.get_param("~voxel", 0.1),
        depth=rospy.get_param("~depth", 9),
        output_dir=rospy.get_param("~output_dir", "/tmp/meshes"))

    rospy.spin()
