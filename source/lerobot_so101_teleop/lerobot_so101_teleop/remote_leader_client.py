"""
Remote Leader Client - Receive Leader arm positions from remote server

This client connects to Remote Leader Server to receive joint positions
of the Leader arm over network, used to control follower in Isaac Sim

Usage:
    Replace LeRobotSO101Interface with RemoteLeaderClient in lerobot_agent.py
"""

import json
import time
import torch
import zmq
from typing import Optional, Tuple


class RemoteLeaderClient:
    """Client to receive Leader arm positions from remote server via ZMQ"""
    
    SO101_USD_MAPPING = {
        "shoulder_pan": {"joint_min": -110, "joint_max": 110},
        "shoulder_lift": {"joint_min": -100, "joint_max": 100},
        "elbow_flex": {"joint_min": -100, "joint_max": 90},
        "wrist_flex": {"joint_min": -95, "joint_max": 95},
        "wrist_roll": {"joint_min": -160, "joint_max": 160},
        "gripper": {"joint_min": -10, "joint_max": 100},
    }
    
    SO101_JOINT_ORDER = [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos",
    ]
    
    def __init__(
        self,
        server_host: str,
        server_port: int = 5555,
        device: str = "cuda:0",
        cameras: dict = None,
        fps: int = 30,
        timeout_ms: int = 1000,
    ):
        """
        Args:
            server_host: IP address of server (Mac with Leader arm)
            server_port: ZMQ port of server
            device: Torch device
            cameras: Camera config dict (for recorder)
            fps: Target FPS
            timeout_ms: Timeout for ZMQ receive
        """
        self.server_host = server_host
        self.server_port = server_port
        self.device = device
        self.cameras = cameras or {}
        self.fps = fps
        self.timeout_ms = timeout_ms
        
        self.context = None
        self.subscriber = None
        self.req_socket = None
        self.connected = False
        
        # Joint mapping tensors
        self.joint_names = [joint.split(".")[0] for joint in self.SO101_JOINT_ORDER]
        self.joint_mins = torch.tensor(
            [self.SO101_USD_MAPPING[name]["joint_min"] for name in self.joint_names],
            dtype=torch.float32,
            device=self.device,
        )
        self.joint_maxs = torch.tensor(
            [self.SO101_USD_MAPPING[name]["joint_max"] for name in self.joint_names],
            dtype=torch.float32,
            device=self.device,
        )
        
        # Last received state (for error handling)
        self.last_state = None
        self.last_raw_action = torch.zeros(6, dtype=torch.float32, device=self.device)
        
    def init_device(self):
        """Initialize ZMQ context - compatible with old interface"""
        print(f"[INFO] Initializing Remote Leader Client...")
        self.context = zmq.Context()
        
    def connect(self):
        """Connect to remote server"""
        print(f"[INFO] Connecting to server at {self.server_host}:{self.server_port}...")
        
        # SUB socket to receive data stream
        self.subscriber = self.context.socket(zmq.SUB)
        
        # *** IMPORTANT: CONFLATE = keep only latest message, discard old ones ***
        # This fixes delay issue caused by buffering old messages
        self.subscriber.setsockopt(zmq.CONFLATE, 1)
        
        # Reduce receive buffer to minimize latency
        self.subscriber.setsockopt(zmq.RCVHWM, 1)  # High water mark = 1 message
        
        self.subscriber.connect(f"tcp://{self.server_host}:{self.server_port}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all
        self.subscriber.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        
        # REQ socket for ping and get info
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{self.server_host}:{self.server_port + 1}")
        self.req_socket.setsockopt(zmq.RCVTIMEO, 3000)  # 3s timeout for requests
        
        # Test connection
        try:
            self.req_socket.send_string("PING")
            response = self.req_socket.recv_string()
            if response == "PONG":
                print(f"[INFO] Connected to Remote Leader Server!")
                
                # Get server info
                self.req_socket.send_string("GET_INFO")
                info = json.loads(self.req_socket.recv_string())
                print(f"[INFO] Server info: {info}")
                
                # Flush old messages from buffer
                print(f"[INFO] Flushing old messages from buffer...")
                flushed = 0
                while True:
                    try:
                        self.subscriber.recv_string(zmq.NOBLOCK)
                        flushed += 1
                    except zmq.Again:
                        break
                if flushed > 0:
                    print(f"[INFO] Flushed {flushed} old messages")
                
                self.connected = True
            else:
                raise ConnectionError(f"Unexpected response: {response}")
        except zmq.Again:
            raise ConnectionError(f"Connection timeout - server not responding at {self.server_host}:{self.server_port + 1}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect: {e}")
            
    def get_raw_action(self) -> dict:
        """Receive action from remote server - returns format matching LeRobot"""
        try:
            msg = self.subscriber.recv_string()
            state = json.loads(msg)
            self.last_state = state
            return state["joints"]
        except zmq.Again:
            print(f"[WARN] Timeout receiving data, using last known state")
            if self.last_state:
                return self.last_state["joints"]
            else:
                # Return zero position if no data yet
                return {joint: 0.0 for joint in self.SO101_JOINT_ORDER}
        except Exception as e:
            print(f"[ERROR] Error receiving data: {e}")
            if self.last_state:
                return self.last_state["joints"]
            return {joint: 0.0 for joint in self.SO101_JOINT_ORDER}
    
    def get_raw_actions_tensor(self, real_action: dict) -> torch.Tensor:
        """Convert dict action to tensor"""
        return torch.tensor(
            [real_action[joint] for joint in self.SO101_JOINT_ORDER],
            dtype=torch.float32,
            device=self.device,
        )
    
    def get_mapped_actions_vectorized(self, raw_values: torch.Tensor) -> torch.Tensor:
        """Map raw values (-100 to 100) to joint angles (radians)"""
        normalized = torch.zeros_like(raw_values)
        normalized[:-1] = (raw_values[:-1] + 100) / 200.0  # first 5 joints: -100-100 -> 0-1
        normalized[-1] = raw_values[-1] / 100.0  # gripper: 0-100 -> 0-1

        # Map to joint ranges (degrees)
        mapped_deg = self.joint_mins + normalized * (self.joint_maxs - self.joint_mins)

        # Convert to radians
        return mapped_deg * torch.pi / 180
    
    def get_raw_actions_from_radians(self, raw_values: torch.Tensor) -> torch.Tensor:
        """Convert from radians back to raw values (-100 to 100)"""
        # Convert from radians to degrees
        mapped_deg = raw_values * 180 / torch.pi

        # Reverse the joint range mapping
        normalized = (mapped_deg - self.joint_mins) / (self.joint_maxs - self.joint_mins)

        # Reverse the normalization
        raw_degrees = torch.zeros_like(normalized)
        raw_degrees[:-1] = normalized[:-1] * 200.0 - 100  # first 5 joints
        raw_degrees[-1] = normalized[-1] * 100.0  # gripper

        return raw_degrees
    
    def real_to_sim_obs_processor(
        self, real_action: dict
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Convert real action (dict from Leader) to simulation action
        
        Returns:
            Tuple of (raw_action_tensor, mapped_action_for_sim)
        """
        real_action_tensor = self.get_raw_actions_tensor(real_action)
        mapped_action = self.get_mapped_actions_vectorized(real_action_tensor)
        self.last_raw_action = real_action_tensor
        return real_action_tensor, mapped_action
    
    def sim_to_real_dataset_processor(
        self, policy_obs: torch.Tensor, visual_obs: dict
    ) -> Tuple[torch.Tensor, dict]:
        """
        Convert simulation observation back to real/dataset format
        
        Args:
            policy_obs: Observation from Isaac Sim (radians)
            visual_obs: Visual observations dict
            
        Returns:
            Tuple of (real_obs_tensor, visual_buffers_dict)
        """
        real_obs = self.get_raw_actions_from_radians(policy_obs)
        visual_buffers = {}
        for camera in self.cameras.keys():
            visual_buffers[camera] = visual_obs[f"camera_{camera}"][0]
        return real_obs, visual_buffers
    
    def disconnect(self):
        """Close connection"""
        print("[INFO] Disconnecting from server...")
        self.connected = False
        
        if self.subscriber:
            self.subscriber.close()
        if self.req_socket:
            self.req_socket.close()
        if self.context:
            self.context.term()
            
        print("[INFO] Disconnected.")


class RemoteLeaderRobot:
    """
    Wrapper class for compatibility with current lerobot_agent.py interface
    This is a "fake robot" object to replace self.robot in LeRobotSO101Interface
    """
    
    def __init__(self, client: RemoteLeaderClient):
        self.client = client
        
    def get_action(self) -> dict:
        """Get action from remote Leader - compatible with LeRobot robot interface"""
        return self.client.get_raw_action()


class RemoteLeaderInterface:
    """
    Drop-in replacement for LeRobotSO101Interface when using Remote Bridge
    
    Maintains the same interface to minimize code changes in lerobot_agent.py
    """
    
    SO101_USD_MAPPING = {
        "shoulder_pan": {"joint_min": -110, "joint_max": 110},
        "shoulder_lift": {"joint_min": -100, "joint_max": 100},
        "elbow_flex": {"joint_min": -100, "joint_max": 90},
        "wrist_flex": {"joint_min": -95, "joint_max": 95},
        "wrist_roll": {"joint_min": -160, "joint_max": 160},
        "gripper": {"joint_min": -10, "joint_max": 100},
    }
    
    SO101_JOINT_ORDER = [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos",
    ]
    
    def __init__(
        self,
        device: str,
        server_host: str,
        server_port: int = 5555,
        cameras: dict = None,
        fps: int = 30,
    ):
        """
        Args:
            device: Torch device (cuda:0, cpu, etc.)
            server_host: IP of Mac running Remote Leader Server
            server_port: Port of Remote Server
            cameras: Camera config
            fps: Target FPS
        """
        self.device = device
        self.server_host = server_host
        self.server_port = server_port
        self.cameras = cameras or {}
        self.fps = fps
        
        self._client = None
        self.robot = None
        
        # Joint mapping
        self.joint_names = [joint.split(".")[0] for joint in self.SO101_JOINT_ORDER]
        self.joint_mins = torch.tensor(
            [self.SO101_USD_MAPPING[name]["joint_min"] for name in self.joint_names],
            dtype=torch.float32,
            device=self.device,
        )
        self.joint_maxs = torch.tensor(
            [self.SO101_USD_MAPPING[name]["joint_max"] for name in self.joint_names],
            dtype=torch.float32,
            device=self.device,
        )
        
    def init_device(self):
        """Initialize connection to remote server"""
        self._client = RemoteLeaderClient(
            server_host=self.server_host,
            server_port=self.server_port,
            device=self.device,
            cameras=self.cameras,
            fps=self.fps,
        )
        self._client.init_device()
        
        # Create wrapper robot object
        self.robot = RemoteLeaderRobot(self._client)
        
        print(f"[INFO] RemoteLeaderInterface initialized - Server: {self.server_host}:{self.server_port}")
        
    def connect(self):
        """Connect to remote server"""
        self._client.connect()
        
    def get_raw_actions_tensor(self, real_action: dict) -> torch.Tensor:
        """Convert dict to tensor"""
        return torch.tensor(
            [real_action[joint] for joint in self.SO101_JOINT_ORDER],
            dtype=torch.float32,
            device=self.device,
        )
    
    def get_mapped_actions_vectorized(self, raw_values: torch.Tensor) -> torch.Tensor:
        """Map raw to sim values"""
        normalized = torch.zeros_like(raw_values)
        normalized[:-1] = (raw_values[:-1] + 100) / 200.0
        normalized[-1] = raw_values[-1] / 100.0
        mapped_deg = self.joint_mins + normalized * (self.joint_maxs - self.joint_mins)
        return mapped_deg * torch.pi / 180
    
    def get_raw_actions_from_radians(self, raw_values: torch.Tensor) -> torch.Tensor:
        """Convert radians back to raw"""
        mapped_deg = raw_values * 180 / torch.pi
        normalized = (mapped_deg - self.joint_mins) / (self.joint_maxs - self.joint_mins)
        raw_degrees = torch.zeros_like(normalized)
        raw_degrees[:-1] = normalized[:-1] * 200.0 - 100
        raw_degrees[-1] = normalized[-1] * 100.0
        return raw_degrees
    
    def real_to_sim_obs_processor(
        self, real_action: dict
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """Process real action to sim format"""
        real_action_tensor = self.get_raw_actions_tensor(real_action)
        mapped_action = self.get_mapped_actions_vectorized(real_action_tensor)
        return real_action_tensor, mapped_action
    
    def sim_to_real_dataset_processor(
        self, policy_obs: torch.Tensor, visual_obs: dict
    ) -> Tuple[torch.Tensor, dict]:
        """Process sim observation to real/dataset format"""
        real_obs = self.get_raw_actions_from_radians(policy_obs)
        visual_buffers = {}
        for camera in self.cameras.keys():
            visual_buffers[camera] = visual_obs[f"camera_{camera}"][0]
        return real_obs, visual_buffers


if __name__ == "__main__":
    # Test client
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, required=True, help="Server IP")
    parser.add_argument("--port", type=int, default=5555, help="Server port")
    args = parser.parse_args()
    
    client = RemoteLeaderClient(
        server_host=args.host,
        server_port=args.port,
        device="cpu"
    )
    
    client.init_device()
    client.connect()
    
    print("\nReceiving data from server...")
    try:
        while True:
            action = client.get_raw_action()
            print(f"Gripper: {action['gripper.pos']:.2f}", end="\r")
            time.sleep(0.033)  # ~30fps
    except KeyboardInterrupt:
        print("\nStopping...")
        client.disconnect()
