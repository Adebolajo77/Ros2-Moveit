#!/usr/bin/env python3

import rclpy
from pymoveit2 import MoveIt2
import time

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
    
    # Set workspace boundaries in MoveIt
    moveit2.set_workspace_parameters(
        min_corner=(0.15, -0.41, 0.10),
        max_corner=(0.68, 0.41, 0.60),
        frame_id="base_link"
    )
    
    # Fast but safe speed
    moveit2.max_velocity = 0.4
    moveit2.max_acceleration = 0.4
    
    print("UR5e initialized with safe boundaries")
    time.sleep(2.0)
    
    # Test positions - mix of safe and unsafe
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
    
    print("\nSafe workspace testing completed!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()