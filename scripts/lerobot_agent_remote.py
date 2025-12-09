# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Modified for Remote Leader Bridge support.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to run Isaac Lab SO-101 Teleop with Remote Leader arm.

Used when Leader arm is connected to a different machine (Mac) and Isaac Sim runs on Ubuntu.

Usage on Ubuntu (Isaac Sim machine):
    python scripts/lerobot_agent_remote.py \
        --task Lerobot-So101-Teleop-Rock-A-Stack-Simple \
        --server_host <MAC_IP_ADDRESS> \
        --server_port 5555

Notes:
    - Must run remote_leader_server.py on Mac first
    - MAC_IP_ADDRESS is the IP of Mac in VPN or local network
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import time

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Isaac Lab SO-101 Teleop with Remote Leader.")

# Remote server arguments
parser.add_argument(
    "--server_host",
    type=str,
    required=True,
    help="IP address of machine running Remote Leader Server (Mac)",
)
parser.add_argument(
    "--server_port",
    type=int,
    default=5555,
    help="Port of Remote Leader Server",
)

# Standard arguments
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument(
    "--num_envs", type=int, default=None, help="Number of environments to simulate."
)
parser.add_argument("--task", type=str, default=None, help="Name of the task.")

parser.add_argument(
    "--repo_id", type=str, default=None, help="Repository ID to store the dataset."
)
parser.add_argument(
    "--repo_root", type=str, default=None, help="Repository root to store the dataset."
)
parser.add_argument("--task_name", type=str, default=None, help="Name of the task.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# always enable cameras to record video
args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

import lerobot_so101_teleop.tasks  # noqa: F401
from lerobot_so101_teleop.keyboard import KeyboardControl
from lerobot_so101_teleop.remote_leader_client import RemoteLeaderInterface
from lerobot_so101_teleop.lerobot_recorder import LeRobotRecorder


def main():

    keyboard_control = KeyboardControl()

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    print(f"[INFO]: Click 'R' to reset the world")
    print(f"[INFO]: Click 'S' to start/stop recording; 'R' will also stop recording")
    # reset environment
    env.reset()

    # cameras
    cameras = {}
    for obj in env.unwrapped.scene.keys():
        if obj.startswith("camera_"):
            camera_cfg = getattr(env.unwrapped.scene.cfg, obj)
            cameras[obj.replace("camera_", "")] = {
                "height": camera_cfg.height,
                "width": camera_cfg.width,
            }
            print(f"[INFO]: Found Camera: {obj.replace('camera_', '')}")
    if len(cameras) == 0:
        print(f"[Info]: No cameras found - videos will not be recorded")

    # Use Remote Leader Interface instead of local
    print(f"[INFO]: Connecting to Remote Leader at {args_cli.server_host}:{args_cli.server_port}...")
    
    robot_iface = RemoteLeaderInterface(
        device=env.unwrapped.device,
        server_host=args_cli.server_host,
        server_port=args_cli.server_port,
        cameras=cameras,
        fps=30,
    )
    robot_iface.init_device()
    robot_iface.connect()
    
    print(f"[INFO]: Remote Leader connected successfully!")

    # Allocate action tensor
    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)

    # simulate environment

    # Recording dataset
    if all([args_cli.repo_id, args_cli.repo_root, args_cli.task_name]):
        recording_mode = True
    else:
        recording_mode = False

    if recording_mode:
        recorder = LeRobotRecorder(
            task_name=args_cli.task_name,
            repo_id=args_cli.repo_id,
            dataset_root=args_cli.repo_root,
            fps=30,
            device=env.unwrapped.device,
            cameras=cameras,
        )
        try:
            recorder.init_dataset()
        except ValueError:
            print(f"[ERROR]: Failed to initialize dataset. folder already exists")
            env.close()
            simulation_app.close()

    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            real_action = robot_iface.robot.get_action()
            real_action, mapped_action = robot_iface.real_to_sim_obs_processor(
                real_action
            )
            actions[:] = mapped_action

            obs, _, _, _, _ = env.step(actions)

            if keyboard_control.reset_world:
                keyboard_control.reset_world = False
                env.reset()
                continue

            if recording_mode and keyboard_control.recording:
                real_obs, visual_buffers = robot_iface.sim_to_real_dataset_processor(
                    obs["policy"][0], 
                    obs["visual"]
                )
                recorder.push_frame_to_buffer(real_action, real_obs, visual_buffers)

    env.close()


if __name__ == "__main__":

    main()

    while True:
        simulation_app.update()

    simulation_app.close()
