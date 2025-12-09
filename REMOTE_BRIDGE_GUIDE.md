# LeRobot Remote Bridge Guide

Guide to control SO-101 Follower on Isaac Sim when Leader arm is connected to a different machine (Mac) via VPN/Network.

## Architecture

```
┌─────────────────────────────┐        VPN/Network        ┌─────────────────────────────────┐
│       MACBOOK (Leader)      │◀─────────────────────────▶│    UBUNTU (Isaac Sim)           │
│                             │                           │                                 │
│  ┌───────────────────────┐  │                           │  ┌───────────────────────────┐  │
│  │   SO-101 Leader Arm   │  │                           │  │   Isaac Sim + Follower    │  │
│  │   (USB Serial)        │  │                           │  │   (Simulated Robot)       │  │
│  └───────────┬───────────┘  │                           │  └───────────────────────────┘  │
│              │              │                           │               ▲                 │
│  ┌───────────▼───────────┐  │      ZMQ Socket           │  ┌────────────┴────────────┐   │
│  │  remote_leader_server │  │  (Port 5555 + 5556)       │  │ lerobot_agent_remote.py │   │
│  │  (Python script)      │──┼──────────────────────────▶│  │ (RemoteLeaderInterface) │   │
│  └───────────────────────┘  │                           │  └─────────────────────────┘   │
└─────────────────────────────┘                           └─────────────────────────────────┘
```

## Requirements

### On Mac (Leader side)
- Python 3.10+
- LeRobot installed (`pip install lerobot`)
- pyzmq (`pip install pyzmq`)
- Leader arm calibrated

### On Ubuntu (Isaac Sim side)
- Isaac Sim 5.1.0 + Isaac Lab 2.3.0
- `lerobot_so101_teleop` repo installed
- pyzmq (`pip install pyzmq` or `uv pip install pyzmq`)

### Network
- VPN or local network allowing connection between two machines
- Ports 5555 and 5556 must be open (firewall)

---

## Step 1: Calibrate Leader Arm (on Mac)

If not calibrated yet, follow the guide: https://huggingface.co/docs/lerobot/en/so101#calibrate

```bash
# On Mac
conda activate lerobot  # or your environment

# Find Leader arm port
ls /dev/tty.usbmodem*

# Calibrate
lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \  # Replace with actual port
    --teleop.id=my_leader_arm
```

Save the port and id information after calibration.

---

## Step 2: Run Remote Leader Server (on Mac)

```bash
# On Mac
cd lerobot_so101_teleop

# Install pyzmq if not installed
pip install pyzmq

# Check Mac's IP in VPN
ifconfig  # or ip addr

# Run server
python scripts/remote_leader_server.py \
    --port /dev/tty.usbmodem58760431551 \
    --id my_leader_arm \
    --host 0.0.0.0 \
    --zmq_port 5555 \
    --fps 30
```

Server will display:
```
[INFO] Leader arm connected successfully!
[INFO] ZMQ PUB server started on tcp://0.0.0.0:5555
[INFO] ZMQ REP server started on tcp://0.0.0.0:5556
[INFO] Low-latency mode enabled (SNDHWM=1)
[INFO] Starting streaming at 30 FPS...
```

Keep this terminal open.

---

## Step 3: Test Connection (on Ubuntu)

```bash
# On Ubuntu
cd lerobot_so101_teleop
conda activate lerobot-isaac-lab

# Install pyzmq
uv pip install pyzmq

# Test connection
python source/lerobot_so101_teleop/lerobot_so101_teleop/remote_leader_client.py \
    --host <MAC_VPN_IP> \
    --port 5555
```

If successful, you'll see gripper value update when moving the Leader arm.

---

## Step 4: Run Isaac Sim with Remote Leader (on Ubuntu)

```bash
# On Ubuntu
cd lerobot_so101_teleop
conda activate lerobot-isaac-lab

# Run teleop
python scripts/lerobot_agent_remote.py \
    --task Lerobot-So101-Teleop-Rock-A-Stack-Simple \
    --server_host <MAC_VPN_IP> \
    --server_port 5555
```

Replace `<MAC_VPN_IP>` with Mac's IP in VPN (e.g., `10.8.0.5`).

### Controls in Isaac Sim:
- **R**: Reset world
- **S**: Start/Stop recording
- **C**: Cancel recording
- **Ctrl+C**: Exit

---

## Step 5: Record Dataset (Optional)

```bash
python scripts/lerobot_agent_remote.py \
    --task Lerobot-So101-Teleop-Rock-A-Stack-Hard \
    --server_host <MAC_VPN_IP> \
    --server_port 5555 \
    --repo_id $HF_USER/so101_teleop_remote \
    --repo_root $(pwd)/datasets/so101_teleop \
    --task_name "Pick up the yellow ring and put it on the pole"
```

---

## Troubleshooting

### Connection Refused
```
ConnectionError: Failed to connect - Connection refused
```
**Fix:**
1. Check server is running on Mac
2. Verify IP address is correct
3. Check firewall: `sudo ufw allow 5555` and `sudo ufw allow 5556`
4. Test ping: `ping <MAC_VPN_IP>`

### Timeout
```
[WARN] Timeout receiving data, using last known state
```
**Fix:**
1. Check network latency (VPN can be slow)
2. Reduce FPS: `--fps 20` on both server and client
3. Increase timeout in code if needed

### Leader Arm Not Found
```
Error: Could not open port /dev/tty.usbmodem*
```
**Fix:**
1. Check USB cable
2. Verify correct port: `ls /dev/tty.usbmodem*`
3. Try unplug and replug USB

### Isaac Sim Lag
If Isaac Sim runs slowly with teleop:
1. Reduce number of environments: `--num_envs 1`
2. Disable some visual effects
3. Check GPU utilization

---

## Advanced: Custom Port Mapping

If you need different ports (VPN restrictions):

**Server (Mac):**
```bash
python scripts/remote_leader_server.py \
    --port /dev/tty.usbmodem* \
    --id my_leader_arm \
    --zmq_port 8888  # Custom port
```

**Client (Ubuntu):**
```bash
python scripts/lerobot_agent_remote.py \
    --task Lerobot-So101-Teleop-Rock-A-Stack-Simple \
    --server_host <MAC_IP> \
    --server_port 8888
```

---

## Network Performance Tips

1. **VPN**: Use WireGuard instead of OpenVPN (lower latency)
2. **Latency**: Ideal < 50ms, acceptable < 100ms
3. **Bandwidth**: ~10KB/s for joint positions (very small)

Test latency:
```bash
# On Ubuntu
ping <MAC_VPN_IP>

# Expected:
# rtt min/avg/max = 5/10/20 ms  # Good
# rtt min/avg/max = 50/80/150 ms  # Okay but may lag
```

---

## Alternative: SSH Port Forwarding

If VPN doesn't work, you can use SSH tunnel:

```bash
# On Ubuntu, create tunnel to Mac
ssh -L 5555:localhost:5555 -L 5556:localhost:5556 user@<MAC_IP>

# Then run client with localhost
python scripts/lerobot_agent_remote.py \
    --server_host localhost \
    --server_port 5555
```

---

## File Structure

```
lerobot_so101_teleop/
├── scripts/
│   ├── lerobot_agent.py           # Original (local Leader)
│   ├── lerobot_agent_remote.py    # New (remote Leader via ZMQ)
│   └── remote_leader_server.py    # Server runs on Mac
│
├── source/lerobot_so101_teleop/lerobot_so101_teleop/
│   ├── lerobot_interface.py       # Original interface
│   └── remote_leader_client.py    # Remote client interface
│
└── REMOTE_BRIDGE_GUIDE.md         # This guide
```

---

## Next Steps

After successfully setting up Remote Bridge, you can:

1. **Collect demonstrations** in Isaac Sim
2. **Train policies** with collected data
3. **Evaluate policies** in simulation
4. **Deploy to real robot** when ready

Refer to the main README.md for more information on training and evaluation.
