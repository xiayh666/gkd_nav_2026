import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import os
import subprocess
import itertools

DEBUG = False

root = tk.Tk()
root.title("导航")
root.geometry("400x400")

options = sorted(list({f.split(".")[0] for f in os.listdir("install/pb2025_nav_bringup/share/pb2025_nav_bringup/map/reality/")}))

selected_option = tk.StringVar(value=options[0])

run_button = tk.Button(root, text="运行", width=8)
mapping_button = tk.Button(root, text="建图", width=8)
save_button = tk.Button(root, text="保存", width=8)
filename_input = tk.Entry(root, width=10)

navigate_button = tk.Button(root, text="导航", width=8)
navigate_pos_input = tk.Entry(root, width=10)


world = ttk.Combobox(root, textvariable=selected_option, values=options, state="readonly", width=10)

run_button.grid(row=0, column=0, padx=5, pady=10)
world.grid(row=0, column=1, padx=5, pady=10)
mapping_button.grid(row=1, column=0, padx=5, pady=10)
save_button.grid(row=2, column=0, padx=5, pady=10)
filename_input.grid(row=2, column=1, padx=5, pady=10)

navigate_button.grid(row=3, column=0, padx=5, pady=10)
navigate_pos_input.grid(row=3, column=1, padx=5, pady=10)

def on_navigate():
    def split(str):
        return list(itertools.chain(*[i.split(",") for i in str.split(" ")]))

    x, y = split(navigate_pos_input.get().strip())
    os.system('ros2 action send_goal /red_standard_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: '+x+', y: '+y+'}}}}" --feedback')

def on_run():
    selected_world = f"{selected_option.get()}"
    # os.system("ros2 launch rmu_gazebo_simulator bringup_sim.launch.py &")
    # os.system(f"ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:={selected_world} slam:=False use_sim_time:=False use_robot_state_pub:=True &")
    os.system(f"ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:={selected_world} \
slam:=False \
use_robot_state_pub:=True")

def on_mapping():
    # os.system("ros2 launch rmu_gazebo_simulator bringup_sim.launch.py &")
    # os.system("ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True &")
    os.system("ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True &")

def on_save():
    filename = filename_input.get().strip()
    if not filename:
        messagebox.showerror("错误", "请输入文件名")
        return
    if filename in options:
        messagebox.showerror("错误", "文件已存在，请更换文件名")
        return
    os.system(f"ros2 run nav2_map_server map_saver_cli -f {filename} --ros-args -r __ns:=/red_standard_robot1")
    os.system(f"mv {filename}.pgm src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/{filename}.pgm")
    os.system(f"mv {filename}.yaml src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/{filename}.yaml")
    os.system(f"cp src/pb2025_sentry_nav/point_lio/PCD/scans.pcd src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/{filename}.pcd")
    # os.system(f"cp src/pb2025_sentry_nav/point_lio/PCD/scans.pcd {filename}.pcd")
    messagebox.showinfo("成功", "地图保存成功")


run_button.config(command=on_run)
mapping_button.config(command=on_mapping)
save_button.config(command=on_save)

navigate_button.config(command=on_navigate)

root.mainloop()
