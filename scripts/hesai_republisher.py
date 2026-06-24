#!/usr/bin/env python3
"""Hesai Pandar -> FAST-LIO2: convert timestamp(float64) -> time(float32), publish to /velodyne_points"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


class HesaiRepublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber('/point_cloud_in', PointCloud2,
                                     self.callback, queue_size=10)
        rospy.loginfo("Hesai republisher ready: /point_cloud_in -> /velodyne_points")

    def callback(self, msg):
        n = msg.width * msg.height
        old_step = msg.point_step  # 48 for Hesai
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, old_step)

        # Extract fields from Hesai format (offsets from bag inspection):
        # x:0-3(float32), y:4-7(float32), z:8-11(float32), pad:12-15,
        # intensity:16-19(float32), pad:20-23,
        # timestamp:24-31(float64), pad:?, ring:32-33(uint16)
        x_f    = raw[:, 0:4].copy().view(np.float32).flatten()
        y_f    = raw[:, 4:8].copy().view(np.float32).flatten()
        z_f    = raw[:, 8:12].copy().view(np.float32).flatten()
        i_f    = raw[:, 16:20].copy().view(np.float32).flatten()
        ts_f64 = raw[:, 24:32].copy().view(np.float64).flatten()  # float64 seconds
        ring_u16 = raw[:, 32:34].copy().view(np.uint16).flatten()

        # Diagnostic (first message only)
        if not hasattr(self, '_diag_done'):
            rospy.loginfo(f"[Hesai] x:[{x_f.min():.2f},{x_f.max():.2f}] "
                          f"y:[{y_f.min():.2f},{y_f.max():.2f}] "
                          f"z:[{z_f.min():.2f},{z_f.max():.2f}] "
                          f"ts:[{ts_f64.min():.6f},{ts_f64.max():.6f}] "
                          f"ring:[{ring_u16.min()},{ring_u16.max()}] "
                          f"n={n} step={old_step}")
            self._diag_done = True

        # Filter NaN
        valid = np.isfinite(x_f) & np.isfinite(y_f) & np.isfinite(z_f)
        n_valid = valid.sum()
        if n_valid == 0:
            return

        x_f = x_f[valid]; y_f = y_f[valid]; z_f = z_f[valid]
        i_f = i_f[valid]; ts_f64 = ts_f64[valid]; ring_u16 = ring_u16[valid]

        # Convert absolute timestamp to scan-relative offset (0.0 ~ 0.1s)
        ts_f32 = (ts_f64 - ts_f64.min()).astype(np.float32)

        # Build 32-byte Velodyne-format output
        new_step = 32
        new_buf = np.zeros((n_valid, new_step), dtype=np.uint8)
        new_buf[:,  0: 4] = x_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:,  4: 8] = y_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:,  8:12] = z_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:, 16:20] = i_f.view(np.uint8).reshape(n_valid, 4)
        new_buf[:, 20:22] = ring_u16.view(np.uint8).reshape(n_valid, 2)
        new_buf[:, 22:26] = ts_f32.view(np.uint8).reshape(n_valid, 4)

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
    rospy.init_node('hesai_republisher')
    node = HesaiRepublisher()
    rospy.spin()
