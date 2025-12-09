#!/usr/bin/env python3
"""
Remote Leader Server - Run on Mac (or machine connected to Leader arm)

This server reads joint positions from the Leader arm and streams them
via ZMQ socket to clients (Isaac Sim on Ubuntu machine)

Usage on Mac:
    python remote_leader_server.py --port /dev/tty.usbmodem* --host 0.0.0.0 --zmq_port 5555

Notes: 
- Replace /dev/tty.usbmodem* with actual port of Leader arm on Mac
- Open firewall for port 5555 if needed
"""

import argparse
import time
import json
import zmq
import signal
import sys

# LeRobot imports
from lerobot.teleoperators.so101_leader import SO101LeaderConfig
from lerobot.robots import make_robot_from_config


class RemoteLeaderServer:
    """Server to stream Leader arm positions over network"""
    
    SO101_JOINT_ORDER = [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos",
    ]
    
    def __init__(self, port: str, leader_id: str, host: str = "0.0.0.0", zmq_port: int = 5555):
        self.port = port
        self.leader_id = leader_id
        self.host = host
        self.zmq_port = zmq_port
        self.robot = None
        self.context = None
        self.socket = None
        self.running = False
        
    def init_leader(self):
        """Initialize connection to Leader arm"""
        print(f"[INFO] Initializing Leader arm at {self.port}...")
        
        cfg = SO101LeaderConfig(port=self.port, id=self.leader_id)
        self.robot = make_robot_from_config(cfg)
        self.robot.connect()
        
        print(f"[INFO] Leader arm connected successfully!")
        
    def init_zmq_server(self):
        """Initialize ZMQ server"""
        print(f"[INFO] Starting ZMQ server on {self.host}:{self.zmq_port}...")
        
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)  # Publisher pattern
        
        # Reduce send buffer to avoid message accumulation when client is slow
        self.socket.setsockopt(zmq.SNDHWM, 1)  # Send High Water Mark = 1
        self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
        
        self.socket.bind(f"tcp://{self.host}:{self.zmq_port}")
        
        # Add REP socket for handling requests (ping/pong and commands)
        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.bind(f"tcp://{self.host}:{self.zmq_port + 1}")
        self.rep_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        
        print(f"[INFO] ZMQ PUB server started on tcp://{self.host}:{self.zmq_port}")
        print(f"[INFO] ZMQ REP server started on tcp://{self.host}:{self.zmq_port + 1}")
        print(f"[INFO] Low-latency mode enabled (SNDHWM=1)")
        
    def get_leader_state(self) -> dict:
        """Read current state from Leader arm"""
        action = self.robot.get_action()
        
        # Return dict with joint positions
        state = {
            "timestamp": time.time(),
            "joints": {joint: action[joint] for joint in self.SO101_JOINT_ORDER},
            "raw_values": [action[joint] for joint in self.SO101_JOINT_ORDER]
        }
        return state
    
    def handle_requests(self):
        """Handle requests from clients"""
        try:
            msg = self.rep_socket.recv_string(zmq.NOBLOCK)
            if msg == "PING":
                self.rep_socket.send_string("PONG")
            elif msg == "GET_INFO":
                info = {
                    "leader_id": self.leader_id,
                    "port": self.port,
                    "joint_order": self.SO101_JOINT_ORDER
                }
                self.rep_socket.send_string(json.dumps(info))
            else:
                self.rep_socket.send_string("UNKNOWN_COMMAND")
        except zmq.Again:
            pass  # No message available
        except Exception as e:
            print(f"[WARN] Error handling request: {e}")
            try:
                self.rep_socket.send_string(f"ERROR: {e}")
            except:
                pass
    
    def run(self, fps: int = 30):
        """Main loop to stream data"""
        self.running = True
        dt = 1.0 / fps
        
        print(f"[INFO] Starting streaming at {fps} FPS...")
        print(f"[INFO] Press Ctrl+C to stop")
        
        frame_count = 0
        start_time = time.time()
        
        while self.running:
            loop_start = time.time()
            
            # Read and send state
            state = self.get_leader_state()
            self.socket.send_string(json.dumps(state))
            
            # Handle requests
            self.handle_requests()
            
            frame_count += 1
            
            # Print info every 2 seconds
            if frame_count % (fps * 2) == 0:
                elapsed = time.time() - start_time
                actual_fps = frame_count / elapsed
                print(f"[INFO] Frame {frame_count}, Actual FPS: {actual_fps:.1f}, "
                      f"Gripper: {state['joints']['gripper.pos']:.1f}")
            
            # Maintain target FPS
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def stop(self):
        """Stop server"""
        print("\n[INFO] Stopping server...")
        self.running = False
        
        if self.robot:
            try:
                self.robot.disconnect()
            except:
                pass
                
        if self.socket:
            self.socket.close()
        if hasattr(self, 'rep_socket') and self.rep_socket:
            self.rep_socket.close()
        if self.context:
            self.context.term()
            
        print("[INFO] Server stopped.")


def main():
    parser = argparse.ArgumentParser(description="Remote Leader Server for SO-101")
    parser.add_argument("--port", type=str, required=True, 
                        help="Serial port of Leader arm (e.g., /dev/tty.usbmodem58760431551)")
    parser.add_argument("--id", type=str, default="leader_arm_1",
                        help="ID of Leader arm (from calibration)")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                        help="Host to bind (0.0.0.0 to accept all)")
    parser.add_argument("--zmq_port", type=int, default=5555,
                        help="ZMQ port to publish data")
    parser.add_argument("--fps", type=int, default=30,
                        help="Streaming frequency (FPS)")
    
    args = parser.parse_args()
    
    server = RemoteLeaderServer(
        port=args.port,
        leader_id=args.id,
        host=args.host,
        zmq_port=args.zmq_port
    )
    
    # Handle Ctrl+C
    def signal_handler(sig, frame):
        server.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        server.init_leader()
        server.init_zmq_server()
        server.run(fps=args.fps)
    except Exception as e:
        print(f"[ERROR] {e}")
        server.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()
