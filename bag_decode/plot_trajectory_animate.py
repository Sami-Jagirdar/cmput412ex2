#!/usr/bin/env python3
import rosbag
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arrow
import matplotlib.colors as mcolors

WHEEL_RADIUS = 0.0318
WHEEL_BASE = 0.09
SCALING_FACTOR = 7.5
vehicle_name = 'csc22926'
wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

def calculate_trajectory(bag_file):
    bag = rosbag.Bag(bag_file)
    previous_time = None
    xi, yi = 0, 0
    theta = math.pi/2
    trajectory = [(xi, yi)]
    thetas = [theta]
    
    for topic, msg, t in bag.read_messages(topics=[wheels_topic]):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        if previous_time is not None:
            dt = current_time - previous_time
        else:
            dt = 0.05
        previous_time = current_time
        
        v_left = msg.vel_left * WHEEL_RADIUS
        v_right = msg.vel_right * WHEEL_RADIUS
        
        change_in_xr = ((v_right + v_left) / 2) * SCALING_FACTOR
        change_in_theta_r = ((v_right - v_left) / WHEEL_BASE) * SCALING_FACTOR
        
        xi += change_in_xr * math.cos(theta) * dt
        yi += change_in_xr * math.sin(theta) * dt
        theta += change_in_theta_r * dt
        
        trajectory.append((xi, yi))
        thetas.append(theta)
    
    bag.close()
    return np.array(trajectory), np.array(thetas)

def create_static_plot(trajectory, thetas, output_file='trajectory_static.png'):
    plt.figure(figsize=(10, 10))
    
    # Plot the main trajectory
    plt.plot(trajectory[:,0], trajectory[:,1], 'b-', linewidth=1.5, label='Trajectory')
    
    # Add small arrows every n points
    n = len(trajectory) // 15  # Show 15 arrows along the path
    for i in range(0, len(trajectory), n):
        if i+1 < len(trajectory):
            dx = math.cos(thetas[i]) * 0.05  # Reduced arrow size
            dy = math.sin(thetas[i]) * 0.05  # Reduced arrow size
            plt.arrow(trajectory[i,0], trajectory[i,1], dx, dy,
                     head_width=0.02, head_length=0.03, fc='red', ec='red', alpha=0.5)
    
    # Add start and end points
    plt.plot(trajectory[0,0], trajectory[0,1], 'go', label='Start')
    plt.plot(trajectory[-1,0], trajectory[-1,1], 'ro', label='End')
    
    plt.title('Duckiebot Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True, linestyle='--', alpha=0.3)
    plt.axis('equal')
    plt.legend()
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def create_animation(trajectory, thetas, output_file='trajectory_animation.gif'):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    def init():
        ax.set_xlim(np.min(trajectory[:,0]) - 0.5, np.max(trajectory[:,0]) + 0.5)
        ax.set_ylim(np.min(trajectory[:,1]) - 0.5, np.max(trajectory[:,1]) + 0.5)
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Duckiebot Trajectory Animation')
        return []

    def animate(frame):
        ax.clear()
        init()
        
        # Plot full path in light gray
        ax.plot(trajectory[:,0], trajectory[:,1], 'lightgray', alpha=0.5, linewidth=1)
        
        # Plot traveled path until current frame
        if frame > 0:
            points = trajectory[:frame]
            colors = np.linspace(0, 1, len(points))
            ax.scatter(points[:,0], points[:,1], c=colors, cmap='viridis', 
                      s=30, alpha=0.6)
            
        # Plot current position with arrow
        dx = math.cos(thetas[frame]) * 0.1
        dy = math.sin(thetas[frame]) * 0.1
        ax.arrow(trajectory[frame,0], trajectory[frame,1], dx, dy,
                head_width=0.05, head_length=0.1, fc='red', ec='red')
        
        # Add start and current position markers
        ax.plot(trajectory[0,0], trajectory[0,1], 'go', markersize=15, label='Start')
        ax.plot(trajectory[frame,0], trajectory[frame,1], 'bo', markersize=10, label='Current')
        ax.legend()
        
        return []

    # Create animation with 100 frames
    n_frames = 100
    frame_indices = np.linspace(0, len(trajectory)-1, n_frames, dtype=int)
    anim = FuncAnimation(fig, animate, frames=frame_indices,
                        init_func=init, blit=True, interval=50)
    
    anim.save(output_file, writer='pillow', fps=30)
    plt.close()

# Main execution
if __name__ == "__main__":
    trajectory, thetas = calculate_trajectory('D2.bag')
    create_static_plot(trajectory, thetas)
    create_animation(trajectory, thetas)