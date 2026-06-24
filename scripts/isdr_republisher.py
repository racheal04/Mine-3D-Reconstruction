#!/usr/bin/env python3
"""
ISDR LiDAR -> FAST-LIO2 point cloud converter.
Input fields: x, y, z, phase, angle, intensity (point_step=24)
Output fields: x, y, z, intensity, ring(uint16), time(float32) (point_step=32)

Ring is computed from the 'angle' field (vertical angle per point).
Time is set to 0 (FAST-LIO2 uses given_offset_time=false fallback).
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


class ISDRRepublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber('/point_cloud_in', PointCloud2,
                                     self.callback, queue_size=10)
        rospy.loginfo("ISDR republisher ready: /point_cloud_in -> /velodyne_points")

    def callback(self, msg):
        n = msg.width * msg.height
        old_step = msg.point_step  # 24 for ISDR
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, old_step)

        # --- Read ISDR fields ---
        x_f = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
        y_f = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
        z_f = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
        # phase at 12:16 (unused)
        angle_f = np.frombuffer(raw[:, 16:20].tobytes(), dtype=np.float32)  # vertical angle in degrees?
        i_f = np.frombuffer(raw[:, 20:24].tobytes(), dtype=np.float32)

        # --- Filter NaNs ---
        valid = np.isfinite(x_f) & np.isfinite(y_f) & np.isfinite(z_f)
        n_valid = valid.sum()
        if n_valid == 0:
            return

        x_f = x_f[valid]; y_f = y_f[valid]; z_f = z_f[valid]
        angle_f = angle_f[valid]; i_f = i_f[valid]

        # --- Compute ring from point position ---
        if msg.height > 1:
            # Structured cloud: ring = row index
            rows = np.arange(n) // msg.width
            ring = rows[valid].astype(np.uint16)
            num_rings = msg.height
        else:
            # Unstructured: compute vertical angle from point position
            dist_xy = np.sqrt(x_f**2 + y_f**2 + 1e-6)
            vert_angle = np.degrees(np.arctan2(z_f, dist_xy))

            # Cluster into rings by sorting vertical angles and finding gaps
            angles_sorted = np.sort(vert_angle)
            diffs = np.diff(angles_sorted)
            # Find significant gaps (> 2x median diff) as ring boundaries
            threshold = np.median(np.abs(diffs)) * 3
            boundaries = np.where(diffs > threshold)[0]
            ring_bounds = np.concatenate([[0], boundaries + 1, [len(angles_sorted)]])
            num_rings = len(ring_bounds) - 1

            # Assign ring: 0 = topmost
            ring = np.zeros(n_valid, dtype=np.uint16)
            for i in range(num_rings):
                lo_val = angles_sorted[ring_bounds[i]]
                hi_val = angles_sorted[ring_bounds[i+1] - 1]
                if i == num_rings - 1:
                    mask = vert_angle >= lo_val
                else:
                    mask = (vert_angle >= lo_val) & (vert_angle <= hi_val)
                ring[mask] = num_rings - 1 - i  # invert: 0=top

        rospy.loginfo_throttle(10, f"Detected {num_rings} rings (h={msg.height}, w={msg.width}, step={old_step})")
        if not hasattr(self, '_first_done'):
            rospy.loginfo(f"[FIRST MSG] height={msg.height} width={msg.width} "
                          f"point_step={old_step} n={n} n_valid={n_valid} "
                          f"vert_angle_range=[{np.degrees(np.arctan2(z_f, np.sqrt(x_f**2+y_f**2+1e-6))).min():.1f}, "
                          f"{np.degrees(np.arctan2(z_f, np.sqrt(x_f**2+y_f**2+1e-6))).max():.1f}] deg "
                          f"num_rings={num_rings}")
            self._first_done = True
        ring_bytes = ring.view(np.uint8).reshape(n_valid, 2)

        # --- Time = 0 (FAST-LIO2 given_offset_time=false fallback) ---
        time_val = np.zeros(n_valid, dtype=np.float32)
        time_bytes = time_val.view(np.uint8).reshape(n_valid, 4)

        # --- Build output (point_step=32) ---
        new_step = 32
        new_buf = np.zeros((n_valid, new_step), dtype=np.uint8)
        new_buf[:,  0: 4] = x_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:,  4: 8] = y_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:,  8:12] = z_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:, 16:20] = i_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:, 20:22] = ring_bytes
        new_buf[:, 22:26] = time_bytes

        fields = [
            PointField('x',         0, PointField.FLOAT32, 1),
            PointField('y',         4, PointField.FLOAT32, 1),
            PointField('z',         8, PointField.FLOAT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1),
            PointField('ring',      20, PointField.UINT16,  1),
            PointField('time',      22, PointField.FLOAT32, 1),
        ]

        out = PointCloud2()
        out.header        = msg.header
        out.header.frame_id = 'velodyne'
        out.height        = 1
        out.width         = n_valid
        out.fields        = fields
        out.is_bigendian  = False
        out.point_step    = new_step
        out.row_step      = new_step * n_valid
        out.is_dense      = True
        out.data          = new_buf.tobytes()

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('isdr_republisher')
    node = ISDRRepublisher()
    rospy.spin()
