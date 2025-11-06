#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Float64MultiArray
import tf2_ros
from scipy.spatial.transform import Rotation as R
import os

CSV_PATH = os.path.join(os.path.expanduser("~"), "Documents", "grasp_poses.csv")
OUT_TOPIC = "/grasp_array"
WORLD_FRAME = "world"
CAMERA_FRAME = "wrist_realsense_gazebo"
TF_TIMEOUT = 0.5
Z_THRESHOLD = 0.40
FILTER_POSITIVE_SCORES = True

def get_camera_to_world():
    buf = tf2_ros.Buffer()
    lis = tf2_ros.TransformListener(buf)
    rospy.sleep(0.1)
    try:
        tfs = buf.lookup_transform(WORLD_FRAME, CAMERA_FRAME, rospy.Time(0), rospy.Duration(TF_TIMEOUT))
    except Exception as e:
        rospy.logerr("TF lookup failed: %s", str(e))
        return None, None
    tr = tfs.transform.translation
    rot = tfs.transform.rotation
    t = np.array([tr.x, tr.y, tr.z], dtype=float)
    quat = np.array([rot.x, rot.y, rot.z, rot.w], dtype=float)
    quat = quat / np.linalg.norm(quat)
    R_c2w = R.from_quat(quat).as_matrix()
    return t, R_c2w

def main():
    rospy.init_node("csv_to_array_pub", anonymous=False)
    pub = rospy.Publisher(OUT_TOPIC, Float64MultiArray, queue_size=1, latch=True)
    t, R_c2w = get_camera_to_world()
    if t is None:
        rospy.signal_shutdown("missing_tf")
        return
    df = pd.read_csv(CSV_PATH)
    df = df[df['pos_z'] > Z_THRESHOLD]
    if FILTER_POSITIVE_SCORES:
        df = df[df['score'] > 0]
    rows_out = []
    for _, row in df.iterrows():
        center_cam = np.array([row['pos_x'], row['pos_y'], row['pos_z']], dtype=float)
        axis_cam   = np.array([row['axis_x'], row['axis_y'], row['axis_z']], dtype=float)
        binorm_cam = np.array([row['binormal_x'], row['binormal_y'], row['binormal_z']], dtype=float)
        appr_cam   = np.array([row['approach_x'], row['approach_y'], row['approach_z']], dtype=float)
        center_w = (R_c2w @ center_cam) + t
        axis_w   = (R_c2w @ axis_cam)
        binorm_w = (R_c2w @ binorm_cam)
        appr_w   = (R_c2w @ appr_cam)
        rows_out.append([
            center_w[0], center_w[1], center_w[2],
            axis_w[0], axis_w[1], axis_w[2],
            appr_w[0], appr_w[1], appr_w[2],
            binorm_w[0], binorm_w[1], binorm_w[2],
            row['score']
        ])
    if len(rows_out) == 0:
        rospy.logwarn("No grasps after filtering")
        rospy.signal_shutdown("no_grasps")
        return
    arr = np.array(rows_out, dtype=float)
    msg = Float64MultiArray()
    msg.data = arr.flatten().tolist()
    pub.publish(msg)
    rospy.loginfo("Published transformed grasp array shape %s", repr(arr.shape))
    rospy.spin()

if __name__ == "__main__":
    main()
