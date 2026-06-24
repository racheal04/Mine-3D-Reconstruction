#!/usr/bin/env python3
"""Old Ouster driver -> FAST-LIO2: rebuild point cloud with exact ouster_ros::Point layout"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


class OusterRepublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/os_cloud_node/points', PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber('/chinook/ouster/points', PointCloud2,
                                     self.callback, queue_size=10)
        rospy.loginfo("Ouster republisher ready: /chinook/ouster/points -> /os_cloud_node/points")

    def callback(self, msg):
        n = msg.width * msg.height
        old_step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, old_step)

        # Read original Ouster fields at KNOWN offsets
        x_i = raw[:, 0:4].copy().view(np.float32).flatten()
        y_i = raw[:, 4:8].copy().view(np.float32).flatten()
        z_i = raw[:, 8:12].copy().view(np.float32).flatten()
        intensity_i = raw[:, 16:20].copy().view(np.float32).flatten()
        t_i = raw[:, 20:24].copy().view(np.uint32).flatten()
        reflectivity_i = raw[:, 24:26].copy().view(np.uint16).flatten()
        ring_i = raw[:, 26:27].copy().view(np.uint8).flatten()
        # noise at 28:30 — discard
        range_i = raw[:, 32:36].copy().view(np.uint32).flatten()

        # Build FAST-LIO2 ouster_ros::Point layout:
        # x(0), y(4), z(8), pad(12), intensity(16), t(20), reflectivity(24),
        # ring(26), ambient(28), range(30)
        new_step = 48
        new_buf = np.zeros((n, new_step), dtype=np.uint8)
        new_buf[:, 0:4] = x_i.view(np.uint8).reshape(n, 4)
        new_buf[:, 4:8] = y_i.view(np.uint8).reshape(n, 4)
        new_buf[:, 8:12] = z_i.view(np.uint8).reshape(n, 4)
        new_buf[:, 16:20] = intensity_i.view(np.uint8).reshape(n, 4)
        new_buf[:, 20:24] = t_i.view(np.uint8).reshape(n, 4)
        new_buf[:, 24:26] = reflectivity_i.view(np.uint8).reshape(n, 2)
        new_buf[:, 26:27] = ring_i.view(np.uint8).reshape(n, 1)
        # ambient at 28:30 = 0 (not present in old driver)
        new_buf[:, 30:34] = range_i.view(np.uint8).reshape(n, 4)

        fields = [
            PointField('x',           0, PointField.FLOAT32, 1),
            PointField('y',           4, PointField.FLOAT32, 1),
            PointField('z',           8, PointField.FLOAT32, 1),
            PointField('intensity',  16, PointField.FLOAT32, 1),
            PointField('t',          20, PointField.UINT32,  1),
            PointField('reflectivity', 24, PointField.UINT16, 1),
            PointField('ring',       26, PointField.UINT8,   1),
            PointField('ambient',    28, PointField.UINT16,  1),
            PointField('range',      30, PointField.UINT32,  1),
        ]

        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = 'os_sensor'
        out.height = msg.height
        out.width = msg.width
        out.fields = fields
        out.is_bigendian = False
        out.point_step = new_step
        out.row_step = new_step * n
        out.is_dense = msg.is_dense
        out.data = new_buf.tobytes()
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('ouster_republisher')
    node = OusterRepublisher()
    rospy.spin()
