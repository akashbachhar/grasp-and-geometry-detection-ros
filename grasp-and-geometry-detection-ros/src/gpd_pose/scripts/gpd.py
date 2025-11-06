#!/usr/bin/env python3
import rospy, sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d, numpy as np, subprocess, shlex, sys
from scipy.spatial.transform import Rotation as R
import tf2_ros
import os

class GpdRunnerWithFilter:
    def __init__(self):
        self.pc_topic = rospy.get_param("~pc_topic", "/wrist/realsense/depth/color/points")
        home_dir = os.path.expanduser("~")
        self.out_pcd = rospy.get_param("~out_pcd", os.path.join("/tmp", "gpd_input_xyz.pcd"))
        self.gpd_bin = rospy.get_param("~gpd_bin", os.path.join(home_dir, "gpd", "build", "detect_grasps"))
        self.gpd_cfg = rospy.get_param("~gpd_cfg", os.path.join(home_dir, "gpd", "cfg", "eigen_param.cfg"))
        self.timeout = rospy.get_param("~timeout", 120.0)
        self.axis = int(rospy.get_param("~axis", 1))
        self.thresh = float(rospy.get_param("~thresh", 0.201))
        self.mode = str(rospy.get_param("~mode", "gt"))
        self.filtered_pcd = rospy.get_param("~filtered_pcd", self.out_pcd)
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.5))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("GPD runner (TF-based filter) starting.")
        rospy.Subscriber(self.pc_topic, PointCloud2, self.cb, queue_size=1)

    def get_tf(self, camera_frame, stamp):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(camera_frame, self.world_frame, rospy.Time(0), rospy.Duration(self.tf_timeout))

        except Exception as e:
            rospy.logerr("TF lookup failed for %s->%s: %s", self.world_frame, camera_frame, str(e))
            rospy.signal_shutdown("missing_tf")
            return None, None
        tr = tf_stamped.transform.translation
        rot = tf_stamped.transform.rotation
        t_vec = np.array([tr.x, tr.y, tr.z], dtype=float)
        quat = np.array([rot.x, rot.y, rot.z, rot.w], dtype=float)
        quat = quat / np.linalg.norm(quat)
        R_c2w = R.from_quat(quat).as_matrix()
        return t_vec, R_c2w

    def cb(self, msg):
        pts = np.array([p[:3] for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)], dtype=float)
        rospy.loginfo("Before filter: %d points", pts.shape[0])
        if pts.shape[0] == 0:
            rospy.signal_shutdown("empty_cloud")
            return

        t_vec, R_c2w = self.get_tf(msg.header.frame_id, msg.header.stamp)
        if t_vec is None:
            return
        R_w2c = R_c2w.T

        pts_w = (R_c2w @ pts.T).T + t_vec
        if self.mode == 'gt':
            mask = pts_w[:, self.axis] > self.thresh
        else:
            mask = pts_w[:, self.axis] < self.thresh
        pts_w_filtered = pts_w[mask]
        rospy.loginfo("After world-filter: %d points", pts_w_filtered.shape[0])
        if pts_w_filtered.shape[0] == 0:
            rospy.signal_shutdown("no_points_after_filter")
            return

        pts_c_back = (R_w2c @ (pts_w_filtered - t_vec).T).T
        pc_o3 = o3d.geometry.PointCloud()
        pc_o3.points = o3d.utility.Vector3dVector(pts_c_back)
        ok = o3d.io.write_point_cloud(self.filtered_pcd, pc_o3, write_ascii=False)
        if not ok:
            rospy.signal_shutdown("pcd_write_failed")
            return
        rospy.loginfo("Saved filtered PCD to %s", self.filtered_pcd)

        cmd = f"{self.gpd_bin} {self.gpd_cfg} {self.filtered_pcd}"
        rospy.loginfo("Running: %s", cmd)
        try:
            proc = subprocess.run(shlex.split(cmd), stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=self.timeout)
            out = proc.stdout.decode(errors='ignore')
            err = proc.stderr.decode(errors='ignore')
            rospy.loginfo("GPD stdout:\n%s", out)
            if err:
                rospy.logwarn("GPD stderr:\n%s", err)
        except subprocess.TimeoutExpired:
            rospy.logerr("GPD binary timed out after %.1f s", self.timeout)
        except Exception as e:
            rospy.logerr("Failed to run GPD binary: %s", str(e))

        rospy.signal_shutdown("finished")

def main():
    rospy.init_node("open3d_gpd_runner_with_filter", anonymous=False)
    node = GpdRunnerWithFilter()
    rospy.spin()

if __name__ == "__main__":
    main()
