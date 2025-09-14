# Ros2-Moveit

# Universal Robots ROS2 Setup and Usage Guide

This guide provides setup instructions and usage examples for working with Universal Robots (UR) in ROS2 environment using both simulation and fake hardware modes.

## Prerequisites

- ROS2 Humble installed
- Ubuntu 22.04 LTS
- Git installed
- Colcon build tools

## Package Installation

### 1. Install Required System Packages

```bash
# Install CycloneDDS middleware
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Install ROS2 Control packages
sudo apt install ros-${ROS_DISTRO}-ros2controlcli
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Install additional dependencies
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-tf2-tools
```

### 2. Install MoveIt2

```bash
# Install MoveIt2 packages
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-resources
sudo apt install ros-humble-moveit-visual-tools
sudo apt install ros-humble-moveit-servo
sudo apt install ros-humble-geometric-shapes
sudo apt install ros-humble-moveit-planners-ompl
sudo apt install ros-humble-pilz-industrial-motion-planner
```

### 3. Install PyMoveIt2

```bash
# Install PyMoveIt2 via pip
pip3 install pymoveit2

# Or build from source in your workspace
cd ~/ros2_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ..
colcon build --packages-select pymoveit2
```

### 4. Install Universal Robots Driver

```bash
# Create workspace if it doesn't exist
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone Universal Robots ROS2 driver
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

# Clone Universal Robots description
git clone -b ros2 https://github.com/ros-industrial/universal_robot.git

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --ignore-src --from-paths src -y

# Build the workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5. Install Gazebo Integration (Optional)

```bash
# Install Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control

# Clone UR Gazebo simulation
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git

# Build with Gazebo support
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Initial Setup

### Environment Setup

Add these lines to your `~/.bashrc` or source them manually:

```bash
source /home/ws/install/setup.bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add workspace to path (adjust path as needed)
source ~/ros2_ws/install/setup.bash
```

After editing ~/.bashrc, reload it:
```bash
source ~/.bashrc
```

## Usage Scenarios

### 1. Gazebo Simulation

Launch UR robot in Gazebo simulation:

**Terminal 1:** Launch Gazebo simulation
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur10e
```

**Terminal 2:** Run example movement
```bash
ros2 run ur_robot_driver example_move.py --ros-args -p controller_name:=joint_trajectory_controller
```

**Alternative:** Launch with MoveIt integration
```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur10e
```

### 2. Fake Hardware Mode

#### Basic Fake Hardware Setup

**Terminal 1:** Launch with fake hardware
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_fake_hardware:=true robot_ip:=yyy.yyy.yyy.yyy
```

**Terminal 2:** Test scaled joint trajectory controller
```bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

#### Alternative Fake Hardware Configuration

**Terminal 1:** Launch with joint trajectory controller
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e \
  robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller
```

**Terminal 2:** Test joint trajectory controller
```bash
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

### 3. MoveIt Integration

#### Complete MoveIt Setup Sequence

**Step 1:** Launch fake hardware with sensor commands
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  use_fake_hardware:=true \
  fake_sensor_commands:=true \
  robot_ip:=yyy.yyy.yyy.yyy
```

**Step 2:** Verify controllers and actions
```bash
ros2 control list_controllers
ros2 action list
```

**Step 3:** Launch MoveIt with RViz
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true \
  use_sim_time:=false
```

## Controller Management

Check available controllers:
```bash
ros2 control list_controllers
```

## Launch Arguments Reference

### ur_control.launch.py Arguments

| Argument | Description | Default | Valid Options |
|----------|-------------|---------|---------------|
| `ur_type` | Type/series of UR robot | - | ur3, ur3e, ur5, ur5e, ur7e, ur10, ur10e, ur12e, ur16e, ur15, ur20, ur30 |
| `robot_ip` | Robot IP address | - | IP address format |
| `use_fake_hardware` | Use fake hardware | false | true/false |
| `fake_sensor_commands` | Enable fake sensor commands | false | true/false |
| `initial_joint_controller` | Initial robot controller | scaled_joint_trajectory_controller | See controller options below |
| `safety_limits` | Enable safety limits | true | true/false |
| `launch_rviz` | Launch RViz | true | true/false |
| `launch_dashboard_client` | Launch Dashboard Client | true | true/false |

### Available Controllers

- `scaled_joint_trajectory_controller` (default)
- `joint_trajectory_controller`
- `forward_velocity_controller`
- `forward_position_controller`
- `freedrive_mode_controller`
- `passthrough_trajectory_controller`

### Communication Ports

| Port | Purpose | Default |
|------|---------|---------|
| `script_command_port` | URScript commands | 50004 |
| `reverse_port` | Cyclic instructions | 50001 |
| `script_sender_port` | External control URScript | 50002 |
| `trajectory_port` | Trajectory control | 50003 |
| `tool_tcp_port` | Tool serial bridging | 54321 |

## Tool Communication (E-Series Only)

For e-series robots, tool communication can be enabled with these parameters:

```bash
use_tool_communication:=true
tool_baud_rate:=115200
tool_parity:=0
tool_stop_bits:=1
tool_voltage:=0
```

## PyMoveIt2 Usage Example

# Safe UR5e Controller Script - Detailed Explanation

## Overview

This Python script demonstrates safe robotic arm control using PyMoveIt2 with the UR5e robot. It implements workspace boundaries, safety checks, and controlled movement execution to prevent collisions and unsafe operations.

## Script Structure

### 1. Import Statements

```python
#!/usr/bin/env python3
import rclpy
from pymoveit2 import MoveIt2
import time
```

- **`rclpy`**: ROS2 Python client library for node creation and communication
- **`pymoveit2`**: Python wrapper for MoveIt2 motion planning framework
- **`time`**: For adding delays between operations

### 2. Safety Check Function

```python
def is_position_safe(position):
    """
    Safety check using the parameters from your workspace analysis
    """
    x, y, z = position
    
    # Your UR5e safe parameters from the analysis
    X_MIN, X_MAX = 0.15, 0.68
    Y_MIN, Y_MAX = -0.41, 0.41
    Z_MIN, Z_MAX = 0.10, 0.60
    
    if not (X_MIN <= x <= X_MAX):
        print(f"UNSAFE: X={x:.3f} outside range [{X_MIN}, {X_MAX}]")
        return False
    
    if not (Y_MIN <= y <= Y_MAX):
        print(f"UNSAFE: Y={y:.3f} outside range [{Y_MIN}, {Y_MAX}]")
        return False
        
    if not (Z_MIN <= z <= Z_MAX):
        print(f"UNSAFE: Z={z:.3f} outside range [{Z_MIN}, {Z_MAX}]")
        return False
    
    print(f"SAFE: Position {position} within bounds")
    return True
```

**Purpose**: Validates if a target position is within safe operating boundaries.

**Safety Parameters**:
- **X-axis**: 0.15m to 0.68m (forward/backward from base)
- **Y-axis**: -0.41m to +0.41m (left/right from base)
- **Z-axis**: 0.10m to 0.60m (height from base)

**Logic**:
1. Extracts X, Y, Z coordinates from input position
2. Checks each coordinate against predefined safe ranges
3. Returns `False` immediately if any coordinate is unsafe
4. Provides detailed feedback about which coordinate failed
5. Returns `True` only if all coordinates are within bounds

### 3. Safe Movement Function

```python
def safe_move(moveit2, position, orientation, node):
    """Move only if position is safe"""
    if not is_position_safe(position):
        return False
    
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=orientation,
        cartesian=False
    )
    
    # Wait for completion
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        state = moveit2.query_state()
        if state.name == 'IDLE':
            break
    
    return moveit2.motion_suceeded
```

**Purpose**: Executes movement only after safety validation.

**Process**:
1. **Pre-movement Safety Check**: Calls `is_position_safe()` before any movement
2. **Movement Execution**: Uses `move_to_pose()` with:
   - `position`: Target XYZ coordinates
   - `quat_xyzw`: Orientation as quaternion [x, y, z, w]
   - `cartesian=False`: Uses joint-space planning (more reliable)
3. **Completion Monitoring**: 
   - Spins the ROS2 node to process callbacks
   - Queries motion state continuously
   - Waits until robot reaches 'IDLE' state
4. **Success Reporting**: Returns whether the movement succeeded

### 4. Main Function - Robot Initialization

```python
def main():
    rclpy.init()
    
    node = rclpy.create_node("safe_ur5e_controller")
    
    moveit2 = MoveIt2(
        node=node,
        joint_names=[
            "shoulder_pan_joint",
            "shoulder_lift_joint", 
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur_manipulator",
        use_move_group_action=True,
    )
```

**Robot Configuration**:
- **Joint Names**: All 6 UR5e joints in correct order
- **Base Link**: Robot's fixed base frame
- **End Effector**: Tool flange (tool0)
- **Group Name**: MoveIt planning group for the arm
- **Move Group Action**: Uses action server for robust communication

### 5. Workspace and Speed Configuration

```python
    # Set workspace boundaries in MoveIt
    moveit2.set_workspace_parameters(
        min_corner=(0.15, -0.41, 0.10),
        max_corner=(0.68, 0.41, 0.60),
        frame_id="base_link"
    )
    
    # Fast but safe speed
    moveit2.max_velocity = 0.4
    moveit2.max_acceleration = 0.4
```

**Workspace Setup**:
- Defines a 3D bounding box for safe operation
- Coordinates relative to `base_link` frame
- Prevents planning motions outside safe zone

**Speed Limits**:
- **Max Velocity**: 40% of maximum (0.4)
- **Max Acceleration**: 40% of maximum (0.4)
- Balances speed with safety and control

### 6. Test Execution

```python
    test_positions = [
        # Safe positions
        ([0.3, 0.0, 0.3], "Center safe zone"),
        ([0.5, -0.2, 0.4], "Left reach"),  
        ([0.4, 0.3, 0.5], "Right reach"),
    ]
    
    orientation = [0.0, 0.707, 0.0, 0.707]  # Tool pointing down
    
    for i, (pos, description) in enumerate(test_positions):
        print(f"\n--- Test {i+1}: {description} ---")
        print(f"Target: {pos}")
        
        success = safe_move(moveit2, pos, orientation, node)
        
        if success:
            print("✓ Movement completed successfully")
        else:
            print("✗ Movement failed or rejected")
        
        time.sleep(1.0)
```

**Test Sequence**:
1. **Position 1**: Center of workspace (0.3, 0.0, 0.3)
2. **Position 2**: Left side reach (0.5, -0.2, 0.4)
3. **Position 3**: Right side reach (0.4, 0.3, 0.5)

**Orientation**: Tool pointing straight down (common for pick-and-place)

## Key Safety Features

### 1. **Pre-Movement Validation**
- Every movement request is validated before execution
- Prevents sending unsafe commands to the robot
- Immediate feedback on why a position is rejected

### 2. **Workspace Boundaries**
- Hard limits prevent robot from reaching dangerous areas
- Protects robot from self-collision
- Prevents damage to surrounding equipment

### 3. **Speed Limiting**
- Reduced velocity and acceleration for safer operation
- Allows better reaction time for emergency stops
- Reduces impact forces in case of unexpected contact

### 4. **State Monitoring**
- Continuous monitoring of robot state during movement
- Ensures movements complete before starting next command
- Provides feedback on movement success/failure

## Usage Instructions

### Prerequisites
1. UR5e robot with fake hardware or simulation running
2. MoveIt2 properly configured for UR5e
3. PyMoveIt2 installed and working

### Running the Script

1. **Terminal 1**: Start robot driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  use_fake_hardware:=true \
  fake_sensor_commands:=true \
  robot_ip:=192.168.1.100
```

2. **Terminal 2**: Start MoveIt
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

3. **Terminal 3**: Run the safety script
```bash
python3 safe_ur5e_controller.py
```

## Expected Output

```
UR5e initialized with safe boundaries
SAFE: Position [0.3, 0.0, 0.3] within bounds

--- Test 1: Center safe zone ---
Target: [0.3, 0.0, 0.3]
✓ Movement completed successfully

SAFE: Position [0.5, -0.2, 0.4] within bounds

--- Test 2: Left reach ---
Target: [0.5, -0.2, 0.4]
✓ Movement completed successfully

SAFE: Position [0.4, 0.3, 0.5] within bounds

--- Test 3: Right reach ---
Target: [0.4, 0.3, 0.5]
✓ Movement completed successfully

Safe workspace testing completed!
```

## Customization Options

### Modify Safety Boundaries
```python
# Adjust these values based on your setup
X_MIN, X_MAX = 0.20, 0.60  # More conservative X range
Y_MIN, Y_MAX = -0.30, 0.30  # Narrower Y range
Z_MIN, Z_MAX = 0.15, 0.50   # Lower maximum height
```

### Add More Test Positions
```python
test_positions = [
    ([0.3, 0.0, 0.3], "Center safe zone"),
    ([0.6, 0.0, 0.2], "Far reach low"),
    ([0.2, 0.4, 0.5], "Close right high"),
    # Add your custom positions here
]
```

### Change Speed Settings
```python
# Slower for precision tasks
moveit2.max_velocity = 0.1
moveit2.max_acceleration = 0.1

# Faster for productivity (use with caution)
moveit2.max_velocity = 0.8
moveit2.max_acceleration = 0.8
```

This script provides a robust foundation for safe robot control that can be extended for various applications while maintaining safety as the top priority.

## Verification Commands

Verify your installation:

```bash
# Check if packages are installed
ros2 pkg list | grep ur
ros2 pkg list | grep moveit

# Check available launch files
ros2 pkg list | xargs -I {} find /opt/ros/humble/share/{} -name "*.launch.py" 2>/dev/null | grep ur

# Test basic functionality
ros2 launch ur_robot_driver ur_control.launch.py --show-args
```

## Troubleshooting

1. **Controller Issues:** Check if controllers are loaded and active:
   ```bash
   ros2 control list_controllers
   ```

2. **Network Issues:** Ensure the robot IP is reachable and ports are not blocked.

3. **RMW Implementation:** Make sure CycloneDDS is properly set as the RMW implementation.

4. **Build Issues:** If you encounter build errors:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ur_robot_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

5. **Missing Dependencies:** Update rosdep and install missing dependencies:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Quick Start Commands

For a quick working setup with UR5e:

```bash
# Terminal 1
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  use_fake_hardware:=true \
  fake_sensor_commands:=true \
  robot_ip:=192.168.1.100

# Terminal 2 (after Terminal 1 is fully loaded)
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

This setup provides a complete working environment with fake hardware and MoveIt integration for testing and development.

## Additional Resources

- [Universal Robots ROS2 Driver Documentation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [PyMoveIt2 Repository](https://github.com/AndrejOrsula/pymoveit2)
- [ROS2 Control Documentation](https://control.ros.org/)