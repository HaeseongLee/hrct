import os
import numpy as np
from tqdm import tqdm
import glob
import re
import matplotlib.pyplot as plt


def main():

    joint_q_dir = "/home/hs_dyros/ros2_ws/src/data_storage/data/ep_06/joint_q.txt"
    hand_eye_dir = "/home/hs_dyros/ros2_ws/src/data_storage/data/ep_06/hand_eye"
    top_down_cam_dir = "/home/hs_dyros/ros2_ws/src/data_storage/data/ep_06/top_down_cam"

    with open(joint_q_dir, 'r') as file:
        n_samples = sum(1 for _ in file)

    joint_q = np.zeros((n_samples, 7))
    t_robot = np.zeros((n_samples, ))
    
    idx = 0
    with open(joint_q_dir, 'r') as file:
        for line in tqdm(file, total=n_samples, desc="Reading lines", unit="line"):
            data = [item for item in line.strip().split() if item]
            data = [float(i) for i in data]

            t_robot[idx] = data[0]
            joint_q[idx] = data[1:]

            idx += 1

    hand_eye_files = glob.glob(os.path.join(hand_eye_dir, "*.png"))
    hand_eye_files.sort()
    top_down_cam_files = glob.glob(os.path.join(top_down_cam_dir, "*.png"))
    top_down_cam_files.sort()

    t_hc = [float(re.search(r'hand_eye_(\d+)\.png', path).group(1))/1000 for path in hand_eye_files]
    t_hc.sort()
    t_hc = np.array(t_hc)

    t_tc = [float(re.search(r'top_down_cam_(\d+)\.png', path).group(1))/1000 for path in top_down_cam_files]
    t_tc.sort()
    t_tc = np.array(t_tc)

    start_time = max(t_robot[0], t_hc[0], t_tc[0])
    end_time = min(t_robot[-1], t_hc[-1], t_tc[-1])

    robot_start = np.where(t_robot == start_time)[0][0]
    robot_end = np.where(t_robot == end_time)[0][0]

    hc_start = np.where(t_hc == start_time)[0][0]
    hc_end = np.where(t_hc == end_time)[0][0]

    tc_start = np.where(t_tc == start_time)[0][0]
    tc_end = np.where(t_tc == end_time)[0][0]

    sliced_joint_q = joint_q[robot_start:robot_end]
    sliced_hand_eye_files = hand_eye_files[hc_start:hc_end]
    sliced_top_down_cam_files = top_down_cam_files[tc_start:tc_end]

    print(np.shape(sliced_joint_q))
    print(np.shape(sliced_hand_eye_files))
    print(np.shape(sliced_top_down_cam_files))


    # filtered_files = [file for file in t_hc if any(num in file for num in sliced_t_hc*1000)]
    # filtered_files = [file for file in t_hc if str(int(sliced_t_hc * 1000)) in file]

    # plt.figure(1, figsize=(8, 5))

    # plt.plot(diff_hc, marker='o', linestyle='-', color='b')
    # plt.hlines(0.033, 0, len(diff_hc))
    # plt.xlabel("Index")
    # plt.ylabel("Time Difference (s)")
    # plt.title("Time Difference Between Consecutive Timestamps")
    # plt.grid(True)
    # # plt.show()

    # plt.figure(2, figsize=(8, 5))
    # plt.plot(diff_tc, marker='o', linestyle='-', color='b')
    # plt.hlines(0.033, 0, len(diff_tc))
    # plt.xlabel("Index")
    # plt.ylabel("Time Difference (s)")
    # plt.title("Time Difference Between Consecutive Timestamps")
    # plt.grid(True)
    # plt.show()


if __name__ == "__main__":
    main()