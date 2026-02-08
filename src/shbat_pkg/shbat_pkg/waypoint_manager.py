#!/usr/bin/env python3
"""
Waypoint Manager - All-in-one GUI for waypoint collection and patrol

Features:
- Collect waypoints by clicking in RViz
- Reorder waypoints (move up/down)
- Delete individual waypoints
- Start/Stop patrol
- Go to next/previous waypoint
- Loop or single-run mode
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import threading
import yaml
import os
import math
from enum import Enum


class PatrolState(Enum):
    IDLE = "Idle"
    RUNNING = "Running"
    PAUSED = "Paused"
    NAVIGATING = "Navigating"


class WaypointManager(Node):
    def __init__(self, gui_callback):
        super().__init__('waypoint_manager')
        
        self.gui_callback = gui_callback
        self.waypoints = []
        self.current_index = 0
        self.state = PatrolState.IDLE
        self.loop_mode = True
        self.goal_handle = None
        
        # Parameters
        self.waypoint_file = os.path.expanduser(
            '~/sahabat_ws/src/shbat_pkg/config/patrol_waypoints.yaml')
        
        # Mode: True = add waypoints, False = navigate
        self.add_mode = True
        
        # TF buffer for getting current robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to dedicated waypoint topic (for RViz tool)
        # Use "2D Goal Pose" tool in RViz, then remap in RViz or use /add_waypoint_pose
        self.waypoint_pose_sub = self.create_subscription(
            PoseStamped, '/add_waypoint_pose', self.waypoint_pose_callback, 10)
        
        # Subscribe to goal pose from RViz "2D Goal Pose" button (only in add mode)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Also subscribe to clicked_point from RViz "Publish Point" button
        self.point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.point_callback, 10)
        
        # Publisher for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer to publish markers
        self.marker_timer = self.create_timer(0.5, self.publish_markers)
        
        # Load existing waypoints if available
        self.load_waypoints()
        
        self.get_logger().info('Waypoint Manager started')
        self.get_logger().info('In Add Mode: Use "2D Goal Pose" or "Publish Point" in RViz to add waypoints')
        self.get_logger().info('Or click "Add Current Pose" button to add robot current location')

    def waypoint_pose_callback(self, msg: PoseStamped):
        """Called when publishing to /add_waypoint_pose - ALWAYS adds waypoint with rotation"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.add_waypoint_at(x, y, yaw)

    def point_callback(self, msg: PointStamped):
        """Called when user clicks 'Publish Point' in RViz - ALWAYS adds waypoint"""
        self.add_waypoint_at(msg.point.x, msg.point.y, 0.0)

    def goal_callback(self, msg: PoseStamped):
        """Called when user clicks 2D Goal Pose in RViz"""
        if not self.add_mode:
            return  # Let Nav2 handle it
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        self.add_waypoint_at(x, y, yaw)

    def add_current_pose_as_waypoint(self):
        """Add the robot's current pose as a waypoint"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            self.add_waypoint_at(x, y, yaw)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to get robot pose: {e}')
            return False

    def add_waypoint_at(self, x, y, yaw):
        """Add a waypoint at the given position"""
        waypoint = {
            'name': f'waypoint_{len(self.waypoints) + 1}',
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw': round(yaw, 3)
        }
        
        self.waypoints.append(waypoint)
        self.get_logger().info(f'Added waypoint: {waypoint["name"]} at ({x:.2f}, {y:.2f})')
        self.gui_callback('update_list')
        self.publish_markers()

    def publish_markers(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, wp in enumerate(self.waypoints):
            # Color: green for current, blue for others
            is_current = (i == self.current_index and self.state != PatrolState.IDLE)
            
            # Cylinder marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.1
            marker.scale.x = 0.4 if is_current else 0.3
            marker.scale.y = 0.4 if is_current else 0.3
            marker.scale.z = 0.2
            marker.color.r = 1.0 if is_current else 0.0
            marker.color.g = 1.0 if not is_current else 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{i+1}"
            marker_array.markers.append(text_marker)
            
            # Arrow for direction
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'arrows'
            arrow.id = i + 2000
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = wp['x']
            arrow.pose.position.y = wp['y']
            arrow.pose.position.z = 0.1
            arrow.pose.orientation.z = math.sin(wp['yaw'] / 2)
            arrow.pose.orientation.w = math.cos(wp['yaw'] / 2)
            arrow.scale.x = 0.5
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            arrow.color.r = 0.0
            arrow.color.g = 0.5
            arrow.color.b = 1.0
            arrow.color.a = 0.8
            marker_array.markers.append(arrow)
        
        self.marker_pub.publish(marker_array)

    def navigate_to_waypoint(self, index):
        """Navigate to a specific waypoint"""
        if not self.waypoints or index >= len(self.waypoints):
            return False
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Navigation server not available')
            self.gui_callback('nav_failed')
            return False
        
        wp = self.waypoints[index]
        self.current_index = index
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.orientation.z = math.sin(wp['yaw'] / 2)
        goal_msg.pose.pose.orientation.w = math.cos(wp['yaw'] / 2)
        
        # Only change state if IDLE (patrol sets RUNNING before calling this)
        if self.state == PatrolState.IDLE:
            self.state = PatrolState.NAVIGATING
        self.gui_callback('update_status')
        
        self.get_logger().info(f'Navigating to waypoint {index + 1}: {wp["name"]}')
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """Called when goal is accepted/rejected"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.state = PatrolState.IDLE
            self.gui_callback('update_status')
            return
        
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Called when navigation completes"""
        result = future.result()
        
        if self.state == PatrolState.PAUSED:
            return
        
        # Only continue to next waypoint if in RUNNING (patrol) mode
        if self.state == PatrolState.RUNNING:
            self.get_logger().info(f'Reached waypoint {self.current_index + 1}')
            
            # Move to next waypoint
            next_index = self.current_index + 1
            
            if next_index >= len(self.waypoints):
                if self.loop_mode:
                    next_index = 0
                    self.get_logger().info('Looping back to start')
                else:
                    self.get_logger().info('Patrol complete')
                    self.state = PatrolState.IDLE
                    self.gui_callback('update_status')
                    return
            
            self.navigate_to_waypoint(next_index)
        else:
            # Single waypoint navigation (Go To Selected) - just stop
            self.get_logger().info(f'Reached waypoint {self.current_index + 1}')
            self.state = PatrolState.IDLE
        
        self.gui_callback('update_status')

    def start_patrol(self):
        """Start patrolling waypoints"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to patrol')
            return False
        
        self.state = PatrolState.RUNNING
        self.current_index = 0
        self.navigate_to_waypoint(0)
        return True

    def stop_patrol(self):
        """Stop patrol"""
        self.state = PatrolState.IDLE
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.gui_callback('update_status')

    def pause_patrol(self):
        """Pause patrol"""
        if self.state == PatrolState.NAVIGATING:
            self.state = PatrolState.PAUSED
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.gui_callback('update_status')

    def resume_patrol(self):
        """Resume patrol"""
        if self.state == PatrolState.PAUSED:
            self.state = PatrolState.RUNNING
            self.navigate_to_waypoint(self.current_index)

    def go_next(self):
        """Go to next waypoint (single, not patrol)"""
        if not self.waypoints:
            return
        next_idx = (self.current_index + 1) % len(self.waypoints)
        self.state = PatrolState.NAVIGATING  # Single nav, not patrol
        self.navigate_to_waypoint(next_idx)

    def go_previous(self):
        """Go to previous waypoint (single, not patrol)"""
        if not self.waypoints:
            return
        prev_idx = (self.current_index - 1) % len(self.waypoints)
        self.state = PatrolState.NAVIGATING  # Single nav, not patrol
        self.navigate_to_waypoint(prev_idx)

    def go_to_index(self, index):
        """Go to specific waypoint by index (single, not patrol)"""
        if 0 <= index < len(self.waypoints):
            self.state = PatrolState.NAVIGATING  # Single nav, not patrol
            self.navigate_to_waypoint(index)

    def move_waypoint_up(self, index):
        """Move waypoint up in list"""
        if index > 0:
            self.waypoints[index], self.waypoints[index-1] = \
                self.waypoints[index-1], self.waypoints[index]
            self.gui_callback('update_list')

    def move_waypoint_down(self, index):
        """Move waypoint down in list"""
        if index < len(self.waypoints) - 1:
            self.waypoints[index], self.waypoints[index+1] = \
                self.waypoints[index+1], self.waypoints[index]
            self.gui_callback('update_list')

    def delete_waypoint(self, index):
        """Delete waypoint at index"""
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]
            self.gui_callback('update_list')

    def rename_waypoint(self, index, new_name):
        """Rename waypoint"""
        if 0 <= index < len(self.waypoints):
            self.waypoints[index]['name'] = new_name
            self.gui_callback('update_list')

    def clear_all(self):
        """Clear all waypoints"""
        self.waypoints = []
        self.current_index = 0
        self.gui_callback('update_list')

    def save_waypoints(self):
        """Save waypoints to file"""
        if not self.waypoints:
            return False, "No waypoints to save"
        
        try:
            os.makedirs(os.path.dirname(self.waypoint_file), exist_ok=True)
            with open(self.waypoint_file, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)
            return True, f"Saved {len(self.waypoints)} waypoints"
        except Exception as e:
            return False, str(e)

    def load_waypoints(self):
        """Load waypoints from file"""
        try:
            if os.path.exists(self.waypoint_file):
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        self.waypoints = []
                        for i, wp in enumerate(data['waypoints']):
                            # Ensure all waypoints have a name
                            if 'name' not in wp:
                                wp['name'] = f'waypoint_{i+1}'
                            # Ensure x, y, yaw exist
                            wp.setdefault('x', 0.0)
                            wp.setdefault('y', 0.0)
                            wp.setdefault('yaw', 0.0)
                            self.waypoints.append(wp)
                        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
                        return True
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
        return False


class WaypointManagerGUI:
    def __init__(self):
        # Initialize ROS first (before GUI)
        rclpy.init()
        self.node = WaypointManager(self.ros_callback)
        
        # Create GUI
        self.root = tk.Tk()
        self.root.title("Waypoint Manager")
        self.root.geometry("450x650")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.running = True
        
        self.create_gui()
        self.update_list()
        self.update_status()
        
        # Use after() for ROS spinning instead of threading
        self.ros_spin()
    
    def ros_spin(self):
        """Spin ROS using tkinter's after() - avoids threading issues"""
        if self.running:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.root.after(10, self.ros_spin)
    
    def on_close(self):
        """Handle window close"""
        self.running = False
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()
    
    def ros_callback(self, action):
        """Called from ROS node to update GUI"""
        self.root.after(0, lambda: self.handle_ros_callback(action))
    
    def handle_ros_callback(self, action):
        if action == 'update_list':
            self.update_list()
        elif action == 'update_status':
            self.update_status()
        elif action == 'nav_failed':
            self.log("ERR: Navigation server not available")
    
    def create_gui(self):
        # Main frame
        main = ttk.Frame(self.root, padding="10")
        main.pack(fill='both', expand=True)
        
        # ===== STATUS BAR =====
        status_frame = ttk.Frame(main)
        status_frame.pack(fill='x', pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Status: Idle", font=('Arial', 12, 'bold'))
        self.status_label.pack(side='left')
        
        self.loop_var = tk.BooleanVar(value=True)
        loop_check = ttk.Checkbutton(status_frame, text="Loop", variable=self.loop_var,
                                      command=self.toggle_loop)
        loop_check.pack(side='right')
        
        # ===== MODE TOGGLE =====
        mode_frame = ttk.LabelFrame(main, text="Click Mode", padding="5")
        mode_frame.pack(fill='x', pady=5)
        
        self.add_mode_var = tk.BooleanVar(value=True)
        ttk.Radiobutton(mode_frame, text="Add Waypoints (click in RViz adds to list)", 
                        variable=self.add_mode_var, value=True,
                        command=self.toggle_add_mode).pack(anchor='w')
        ttk.Radiobutton(mode_frame, text="Navigate (click in RViz sends robot there)", 
                        variable=self.add_mode_var, value=False,
                        command=self.toggle_add_mode).pack(anchor='w')
        
        # ===== WAYPOINT LIST =====
        list_frame = ttk.LabelFrame(main, text="Waypoints (click '2D Goal Pose' in RViz to add)", padding="5")
        list_frame.pack(fill='both', expand=True, pady=5)
        
        # Listbox with scrollbar
        list_container = ttk.Frame(list_frame)
        list_container.pack(fill='both', expand=True)
        
        self.waypoint_listbox = tk.Listbox(list_container, height=10, font=('Courier', 10))
        self.waypoint_listbox.pack(side='left', fill='both', expand=True)
        
        scrollbar = ttk.Scrollbar(list_container, orient='vertical', 
                                   command=self.waypoint_listbox.yview)
        scrollbar.pack(side='right', fill='y')
        self.waypoint_listbox['yscrollcommand'] = scrollbar.set
        
        # List control buttons
        list_btn_frame = ttk.Frame(list_frame)
        list_btn_frame.pack(fill='x', pady=5)
        
        ttk.Button(list_btn_frame, text="Add Current Pose", width=15, 
                   command=self.add_current_pose).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Up", width=6, 
                   command=self.move_up).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Down", width=6,
                   command=self.move_down).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Rename", width=7,
                   command=self.rename_selected).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Delete", width=6,
                   command=self.delete_selected).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Clear", width=6,
                   command=self.clear_all).pack(side='left', padx=2)
        
        # ===== NAVIGATION CONTROLS =====
        nav_frame = ttk.LabelFrame(main, text="Navigation", padding="10")
        nav_frame.pack(fill='x', pady=5)
        
        # Row 1: Go to selected, Previous, Next
        nav_row1 = ttk.Frame(nav_frame)
        nav_row1.pack(fill='x', pady=2)
        
        ttk.Button(nav_row1, text="<< Prev", width=12,
                   command=self.go_previous).pack(side='left', padx=2)
        ttk.Button(nav_row1, text="Go To Selected", width=14,
                   command=self.go_to_selected).pack(side='left', padx=2)
        ttk.Button(nav_row1, text="Next >>", width=12,
                   command=self.go_next).pack(side='left', padx=2)
        
        # Row 2: Patrol controls
        nav_row2 = ttk.Frame(nav_frame)
        nav_row2.pack(fill='x', pady=2)
        
        ttk.Button(nav_row2, text="Start Patrol", width=14,
                   command=self.start_patrol).pack(side='left', padx=2)
        ttk.Button(nav_row2, text="Pause", width=10,
                   command=self.pause_patrol).pack(side='left', padx=2)
        ttk.Button(nav_row2, text="Resume", width=10,
                   command=self.resume_patrol).pack(side='left', padx=2)
        ttk.Button(nav_row2, text="Stop", width=10,
                   command=self.stop_patrol).pack(side='left', padx=2)
        
        # ===== FILE OPERATIONS =====
        file_frame = ttk.LabelFrame(main, text="File", padding="5")
        file_frame.pack(fill='x', pady=5)
        
        ttk.Button(file_frame, text="Save Waypoints", 
                   command=self.save_waypoints).pack(side='left', padx=5)
        ttk.Button(file_frame, text="Reload from File",
                   command=self.reload_waypoints).pack(side='left', padx=5)
        
        # ===== LOG =====
        log_frame = ttk.LabelFrame(main, text="Log", padding="5")
        log_frame.pack(fill='both', expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, height=5, state='disabled', wrap='word')
        self.log_text.pack(fill='both', expand=True)
        
        self.log("Waypoint Manager Ready")
        self.log("Click '2D Goal Pose' in RViz to add waypoints")
    
    def log(self, message):
        self.log_text.config(state='normal')
        self.log_text.insert('end', message + '\n')
        self.log_text.see('end')
        self.log_text.config(state='disabled')
    
    def update_list(self):
        """Update the waypoint listbox"""
        self.waypoint_listbox.delete(0, tk.END)
        for i, wp in enumerate(self.node.waypoints):
            current = "→ " if i == self.node.current_index and self.node.state != PatrolState.IDLE else "  "
            self.waypoint_listbox.insert(tk.END, 
                f"{current}{i+1}. {wp['name']} ({wp['x']:.2f}, {wp['y']:.2f})")
    
    def update_status(self):
        """Update status label"""
        state = self.node.state.value
        if self.node.state != PatrolState.IDLE:
            state += f" - Waypoint {self.node.current_index + 1}/{len(self.node.waypoints)}"
        self.status_label.config(text=f"Status: {state}")
    
    def get_selected_index(self):
        selection = self.waypoint_listbox.curselection()
        return selection[0] if selection else None
    
    def move_up(self):
        idx = self.get_selected_index()
        if idx is not None and idx > 0:
            self.node.move_waypoint_up(idx)
            self.waypoint_listbox.selection_set(idx - 1)
    
    def move_down(self):
        idx = self.get_selected_index()
        if idx is not None:
            self.node.move_waypoint_down(idx)
            self.waypoint_listbox.selection_set(idx + 1)
    
    def add_current_pose(self):
        """Add robot's current position as a waypoint"""
        if self.node.add_current_pose_as_waypoint():
            self.log("Added current robot pose as waypoint")
        else:
            self.log("ERR: Could not get robot pose (TF not available)")
            messagebox.showerror("Error", "Could not get robot pose.\nMake sure robot is localized.")
    
    def delete_selected(self):
        idx = self.get_selected_index()
        if idx is not None:
            self.node.delete_waypoint(idx)
            self.log(f"Deleted waypoint {idx + 1}")
    
    def rename_selected(self):
        idx = self.get_selected_index()
        if idx is not None:
            current_name = self.node.waypoints[idx]['name']
            new_name = simpledialog.askstring("Rename", "New name:", initialvalue=current_name)
            if new_name:
                self.node.rename_waypoint(idx, new_name)
                self.log(f"Renamed to: {new_name}")
    
    def clear_all(self):
        if messagebox.askyesno("Clear All", "Delete all waypoints?"):
            self.node.clear_all()
            self.log("Cleared all waypoints")
    
    def go_to_selected(self):
        idx = self.get_selected_index()
        if idx is not None:
            self.node.go_to_index(idx)
            self.log(f"Going to waypoint {idx + 1}")
    
    def go_next(self):
        self.node.go_next()
        self.log("Going to next waypoint")
    
    def go_previous(self):
        self.node.go_previous()
        self.log("Going to previous waypoint")
    
    def start_patrol(self):
        if self.node.start_patrol():
            self.log("Started patrol")
        else:
            self.log("ERR: No waypoints to patrol")
    
    def pause_patrol(self):
        self.node.pause_patrol()
        self.log("Paused patrol")
    
    def resume_patrol(self):
        self.node.resume_patrol()
        self.log("Resumed patrol")
    
    def stop_patrol(self):
        self.node.stop_patrol()
        self.log("Stopped patrol")
    
    def toggle_loop(self):
        self.node.loop_mode = self.loop_var.get()
        mode = "Loop" if self.node.loop_mode else "Single run"
        self.log(f"Mode: {mode}")
    
    def toggle_add_mode(self):
        self.node.add_mode = self.add_mode_var.get()
        if self.node.add_mode:
            self.log("Mode: ADD WAYPOINTS - clicks add to list")
        else:
            self.log("Mode: NAVIGATE - clicks send robot there")
    
    def save_waypoints(self):
        success, msg = self.node.save_waypoints()
        if success:
            self.log(f"OK: {msg}")
        else:
            self.log(f"ERR: {msg}")
    
    def reload_waypoints(self):
        if self.node.load_waypoints():
            self.update_list()
            self.log(f"Loaded {len(self.node.waypoints)} waypoints")
        else:
            self.log("Could not load waypoints")
    
    def run(self):
        self.root.mainloop()


def main():
    try:
        gui = WaypointManagerGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()
