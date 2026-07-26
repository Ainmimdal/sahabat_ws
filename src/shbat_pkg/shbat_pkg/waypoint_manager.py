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
import uuid
from enum import Enum

from .waypoint_store import WaypointStore


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
        self.cancel_requested = False
        self.patrol_indices = []
        
        # Parameters. Keep the historical path as the default so existing
        # commands behave exactly as before, while launches can now select an
        # operational state file outside the source tree.
        self.declare_parameter(
            'waypoint_file',
            '~/sahabat_ws/src/shbat_pkg/config/patrol_waypoints.yaml',
        )
        self.waypoint_file = os.path.expanduser(
            self.get_parameter('waypoint_file').value
        )
        self.store = WaypointStore(os.path.dirname(self.waypoint_file))
        self.waypoint_sets = []
        self.active_set_id = ''
        self.revision = 0
        self.dock = None
        
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
            'id': uuid.uuid4().hex,
            'name': f'waypoint_{len(self.waypoints) + 1}',
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw': round(yaw, 3),
            'dwell_seconds': 0.0,
            'enabled': True,
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
        if not self.waypoints[index].get('enabled', True):
            self.get_logger().warning('Selected waypoint is disabled')
            return False
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Navigation server not available')
            self.gui_callback('nav_failed')
            return False
        
        wp = self.waypoints[index]
        self.current_index = index
        self.cancel_requested = False
        
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
        if self.cancel_requested or self.state == PatrolState.IDLE:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
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
            try:
                patrol_position = self.patrol_indices.index(self.current_index)
            except ValueError:
                patrol_position = -1
            next_position = patrol_position + 1

            if next_position >= len(self.patrol_indices):
                if self.loop_mode:
                    next_position = 0
                    self.get_logger().info('Looping back to start')
                else:
                    self.get_logger().info('Patrol complete')
                    self.state = PatrolState.IDLE
                    self.gui_callback('update_status')
                    return

            self.navigate_to_waypoint(self.patrol_indices[next_position])
        else:
            # Single waypoint navigation (Go To Selected) - just stop
            self.get_logger().info(f'Reached waypoint {self.current_index + 1}')
            self.state = PatrolState.IDLE
        
        self.gui_callback('update_status')

    def start_patrol(self):
        """Start patrolling waypoints"""
        self.patrol_indices = [
            index for index, waypoint in enumerate(self.waypoints)
            if waypoint.get('enabled', True)
        ]
        if not self.patrol_indices:
            self.get_logger().warn('No waypoints to patrol')
            return False
        
        self.state = PatrolState.RUNNING
        self.current_index = self.patrol_indices[0]
        self.navigate_to_waypoint(self.current_index)
        return True

    def stop_patrol(self):
        """Stop patrol"""
        self.cancel_requested = True
        self.state = PatrolState.IDLE
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
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
        enabled = [
            index for index, waypoint in enumerate(self.waypoints)
            if waypoint.get('enabled', True)
        ]
        if not enabled:
            return
        later = [index for index in enabled if index > self.current_index]
        next_idx = later[0] if later else enabled[0]
        self.state = PatrolState.NAVIGATING  # Single nav, not patrol
        self.navigate_to_waypoint(next_idx)

    def go_previous(self):
        """Go to previous waypoint (single, not patrol)"""
        enabled = [
            index for index, waypoint in enumerate(self.waypoints)
            if waypoint.get('enabled', True)
        ]
        if not enabled:
            return
        earlier = [index for index in enabled if index < self.current_index]
        prev_idx = earlier[-1] if earlier else enabled[-1]
        self.state = PatrolState.NAVIGATING  # Single nav, not patrol
        self.navigate_to_waypoint(prev_idx)

    def go_to_index(self, index):
        """Go to specific waypoint by index (single, not patrol)"""
        if (
            0 <= index < len(self.waypoints)
            and self.waypoints[index].get('enabled', True)
        ):
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
        """Save the selected map-scoped waypoint set."""
        try:
            self.revision = self.store.save_set(
                self.active_set_id, self.revision, self.waypoints
            )
            self.refresh_waypoint_sets()
            return True, (
                f'Saved {len(self.waypoints)} waypoints to '
                f'{self.active_set_name}'
            )
        except RuntimeError as error:
            if str(error).startswith('revision:'):
                return False, 'Set changed on disk; reload before saving'
            return False, str(error)
        except Exception as e:
            return False, str(e)

    @property
    def active_set_name(self):
        match = next(
            (item for item in self.waypoint_sets
             if item['id'] == self.active_set_id),
            None,
        )
        return match['name'] if match else self.active_set_id

    def refresh_waypoint_sets(self):
        self.waypoint_sets = self.store.list_sets()
        self.active_set_id = self.store.active_set_id()
        self.dock = self.store.load_dock()
        return self.waypoint_sets

    def select_waypoint_set(self, set_id):
        self.store.select_set(set_id)
        self.active_set_id = set_id
        return self.load_waypoints()

    def create_waypoint_set(self, name):
        self.active_set_id = self.store.create_set(name)
        self.refresh_waypoint_sets()
        return self.load_waypoints()

    def rename_waypoint_set(self, name):
        self.store.rename_set(self.active_set_id, name)
        self.refresh_waypoint_sets()

    def delete_waypoint_set(self):
        self.active_set_id = self.store.delete_set(self.active_set_id)
        self.refresh_waypoint_sets()
        return self.load_waypoints()

    def save_current_pose_as_dock(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            q = transform.transform.rotation
            self.dock = {
                'id': 'dock',
                'name': 'dock',
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'yaw': math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                ),
            }
            self.store.save_dock(self.dock)
            return True
        except Exception as error:
            self.get_logger().error(f'Failed to save dock pose: {error}')
            return False

    def navigate_to_dock(self):
        """Navigate to the map-level dock pose."""
        if not self.dock:
            self.get_logger().warning('No dock pose is saved for this map')
            return False
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Navigation server not available')
            self.gui_callback('nav_failed')
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.dock['x']
        goal_msg.pose.pose.position.y = self.dock['y']
        goal_msg.pose.pose.orientation.z = math.sin(self.dock['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.dock['yaw'] / 2.0)
        self.state = PatrolState.NAVIGATING
        self.gui_callback('update_status')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Navigating to map dock')
        return True

    def save_as_exhibits(self, exhibit_file):
        """Export waypoints as exhibits for exhibit navigator"""
        if not self.waypoints:
            return False, "No waypoints to export"
        
        try:
            existing = {'routes': {}, 'default_tour': [], 'settings': {}}
            if os.path.exists(exhibit_file):
                with open(exhibit_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        existing = data
            
            exhibits = {}
            for i, wp in enumerate(self.waypoints):
                raw_name = wp.get('name', f'Waypoint {i+1}')
                safe_name = raw_name.lower().replace(' ', '_').replace('-', '_').replace('.', '_')
                exhibits[safe_name] = {
                    'x': round(wp['x'], 3),
                    'y': round(wp['y'], 3),
                    'yaw': round(wp['yaw'], 3),
                    'description': raw_name
                }
            
            output = {
                'exhibits': exhibits,
                'routes': existing.get('routes', {}),
                'default_tour': existing.get('default_tour', []),
                'settings': existing.get('settings', {
                    'wait_at_exhibit': 5.0,
                    'announce_arrival': True,
                    'allow_direct_navigation': True,
                    'battery_low_threshold': 30.0,
                    'battery_critical_threshold': 15.0,
                    'obstruction_timeout': 10.0
                })
            }
            
            os.makedirs(os.path.dirname(exhibit_file), exist_ok=True)
            with open(exhibit_file, 'w') as f:
                f.write("# Exhibit Routes Configuration\n")
                f.write("# Generated from Waypoint Manager\n\n")
                yaml.dump(output, f, default_flow_style=False, sort_keys=False)
            
            return True, f"Saved {len(exhibits)} exhibits to exhibit_routes.yaml"
        except Exception as e:
            return False, str(e)

    def load_exhibits(self, exhibit_file):
        """Load exhibits from exhibit_routes.yaml as waypoints"""
        try:
            if not os.path.exists(exhibit_file):
                return False, "exhibit_routes.yaml not found"
            with open(exhibit_file, 'r') as f:
                data = yaml.safe_load(f)
            if not data or 'exhibits' not in data:
                return False, "No exhibits in file"

            self.waypoints = []
            for name, info in data['exhibits'].items():
                self.waypoints.append({
                    'name': name,
                    'x': float(info['x']),
                    'y': float(info['y']),
                    'yaw': float(info.get('yaw', 0.0))
                })
            return True, f"Loaded {len(self.waypoints)} exhibits"
        except Exception as e:
            return False, str(e)

    def load_waypoints(self):
        """Load the active map-scoped waypoint set."""
        try:
            self.refresh_waypoint_sets()
            self.revision, self.waypoints = self.store.read_set(
                self.active_set_id
            )
            self.current_index = 0
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from '
                f'{self.active_set_name}'
            )
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
        self.root.title("Sahabat Waypoints")
        self.root.geometry("620x720")
        self.root.minsize(560, 620)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.running = True
        self.waypoints_dirty = False
        
        self.create_gui()
        self.refresh_set_controls()
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
            self.set_waypoint_dirty(True)
        elif action == 'update_status':
            self.update_status()
        elif action == 'nav_failed':
            self.log("ERR: Navigation server not available")
    
    def create_gui(self):
        main = ttk.Frame(self.root, padding="12")
        main.pack(fill='both', expand=True)
        
        # ===== STATUS BAR =====
        status_frame = ttk.Frame(main)
        status_frame.pack(fill='x', pady=(0, 8))
        
        self.status_label = ttk.Label(status_frame, text="Status: Idle", font=('Arial', 12, 'bold'))
        self.status_label.pack(side='left')
        
        self.loop_var = tk.BooleanVar(value=True)
        loop_check = ttk.Checkbutton(status_frame, text="Loop", variable=self.loop_var,
                                      command=self.toggle_loop)
        loop_check.pack(side='right')

        # ===== MAP-SCOPED SET =====
        set_frame = ttk.LabelFrame(main, text="Waypoint set", padding="8")
        set_frame.pack(fill='x', pady=(0, 8))
        set_frame.columnconfigure(0, weight=1)

        self.set_var = tk.StringVar()
        self.set_combo = ttk.Combobox(
            set_frame, textvariable=self.set_var, state='readonly'
        )
        self.set_combo.grid(row=0, column=0, sticky='ew', padx=(0, 6))
        self.set_combo.bind('<<ComboboxSelected>>', self.on_set_selected)
        ttk.Button(
            set_frame, text="New", command=self.create_set
        ).grid(row=0, column=1, padx=2)
        ttk.Button(
            set_frame, text="Rename", command=self.rename_set
        ).grid(row=0, column=2, padx=2)
        ttk.Button(
            set_frame, text="Delete", command=self.delete_set
        ).grid(row=0, column=3, padx=(2, 0))

        self.dock_label = ttk.Label(set_frame, text="Map dock: not captured")
        self.dock_label.grid(row=1, column=0, columnspan=2, sticky='w', pady=(8, 0))
        dock_actions = ttk.Frame(set_frame)
        dock_actions.grid(row=1, column=2, columnspan=2, sticky='e', pady=(8, 0))
        ttk.Button(
            dock_actions, text="Go to dock", command=self.go_to_dock
        ).pack(side='left', padx=2)
        ttk.Button(
            dock_actions, text="Capture here", command=self.capture_dock
        ).pack(side='left', padx=2)
        
        # ===== MODE TOGGLE =====
        mode_frame = ttk.LabelFrame(main, text="Click Mode", padding="5")
        mode_frame.pack(fill='x', pady=(0, 8))
        
        self.add_mode_var = tk.BooleanVar(value=True)
        ttk.Radiobutton(mode_frame, text="Add Waypoints (click in RViz adds to list)", 
                        variable=self.add_mode_var, value=True,
                        command=self.toggle_add_mode).pack(anchor='w')
        ttk.Radiobutton(mode_frame, text="Navigate (click in RViz sends robot there)", 
                        variable=self.add_mode_var, value=False,
                        command=self.toggle_add_mode).pack(anchor='w')
        
        # ===== WAYPOINT LIST =====
        list_frame = ttk.LabelFrame(
            main,
            text="Waypoints — use RViz 2D Goal Pose to add",
            padding="8",
        )
        list_frame.pack(fill='both', expand=True, pady=(0, 8))
        
        # Listbox with scrollbar
        list_container = ttk.Frame(list_frame)
        list_container.pack(fill='both', expand=True)
        
        self.waypoint_listbox = tk.Listbox(
            list_container, height=10, font=('TkFixedFont', 10),
            selectmode=tk.BROWSE,
        )
        self.waypoint_listbox.pack(side='left', fill='both', expand=True)
        
        scrollbar = ttk.Scrollbar(list_container, orient='vertical', 
                                   command=self.waypoint_listbox.yview)
        scrollbar.pack(side='right', fill='y')
        self.waypoint_listbox['yscrollcommand'] = scrollbar.set
        
        # List control buttons
        list_btn_frame = ttk.Frame(list_frame)
        list_btn_frame.pack(fill='x', pady=(8, 0))
        
        ttk.Button(list_btn_frame, text="Add Current Pose", width=15, 
                   command=self.add_current_pose).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Up", width=6, 
                   command=self.move_up).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Down", width=6,
                   command=self.move_down).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Edit", width=7,
                   command=self.edit_selected).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Delete", width=6,
                   command=self.delete_selected).pack(side='left', padx=2)
        ttk.Button(list_btn_frame, text="Clear set", width=9,
                   command=self.clear_all).pack(side='left', padx=2)
        
        # ===== NAVIGATION CONTROLS =====
        nav_frame = ttk.LabelFrame(main, text="Navigation", padding="10")
        nav_frame.pack(fill='x', pady=(0, 8))
        
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
        
        # ===== SAVE AND SECONDARY TOOLS =====
        file_frame = ttk.Frame(main)
        file_frame.pack(fill='x', pady=(0, 8))
        
        self.save_button = ttk.Button(
            file_frame, text="Saved", command=self.save_waypoints,
            state='disabled',
        )
        self.save_button.pack(side='left', padx=5)
        ttk.Button(file_frame, text="Discard changes",
                   command=self.reload_waypoints).pack(side='left', padx=5)
        ttk.Button(file_frame, text="Save as Exhibits",
                   command=self.save_waypoints_as_exhibits).pack(side='right', padx=5)
        ttk.Button(file_frame, text="Load Exhibits",
                   command=self.load_waypoints_from_exhibits).pack(side='right', padx=5)
        
        # ===== LOG =====
        log_frame = ttk.LabelFrame(main, text="Log", padding="5")
        log_frame.pack(fill='x')
        
        self.log_text = tk.Text(log_frame, height=4, state='disabled', wrap='word')
        self.log_text.pack(fill='both', expand=True)
        
        self.log("Waypoint Manager Ready")
        self.log("Click '2D Goal Pose' in RViz to add waypoints")

    def refresh_set_controls(self):
        """Refresh the set selector and map-level dock summary."""
        self.node.refresh_waypoint_sets()
        values = [
            f"{item['name']}  ·  {item['waypoint_count']} waypoint(s)"
            for item in self.node.waypoint_sets
        ]
        self.set_combo['values'] = values
        active_index = next(
            (index for index, item in enumerate(self.node.waypoint_sets)
             if item['id'] == self.node.active_set_id),
            0,
        )
        if values:
            self.set_combo.current(active_index)
        dock = self.node.dock
        self.dock_label.config(
            text=(
                f"Map dock: {dock['x']:.2f}, {dock['y']:.2f}, "
                f"{dock['yaw']:.2f} rad"
                if dock else 'Map dock: not captured'
            )
        )

    def set_waypoint_dirty(self, dirty):
        """Show whether the selected set differs from disk."""
        self.waypoints_dirty = dirty
        self.save_button.config(
            text='Save changes' if dirty else 'Saved',
            state='normal' if dirty else 'disabled',
        )

    def on_set_selected(self, _event=None):
        index = self.set_combo.current()
        if index < 0 or index >= len(self.node.waypoint_sets):
            return
        selected = self.node.waypoint_sets[index]
        if selected['id'] == self.node.active_set_id:
            return
        if self.waypoints_dirty and not messagebox.askyesno(
            'Discard changes?', 'Switch sets and discard unsaved changes?'
        ):
            self.refresh_set_controls()
            return
        if self.node.state != PatrolState.IDLE:
            messagebox.showwarning(
                'Navigation active', 'Stop navigation before switching sets.'
            )
            self.refresh_set_controls()
            return
        self.node.select_waypoint_set(selected['id'])
        self.refresh_set_controls()
        self.update_list()
        self.set_waypoint_dirty(False)
        self.log(f"Selected set: {selected['name']}")

    def create_set(self):
        if self.waypoints_dirty and not messagebox.askyesno(
            'Discard changes?', 'Create a set and discard unsaved changes?'
        ):
            return
        name = simpledialog.askstring('New waypoint set', 'Set name:')
        if not name or not name.strip():
            return
        self.node.create_waypoint_set(name.strip())
        self.refresh_set_controls()
        self.update_list()
        self.set_waypoint_dirty(False)
        self.log(f'Created set: {name.strip()}')

    def rename_set(self):
        name = simpledialog.askstring(
            'Rename waypoint set', 'Set name:',
            initialvalue=self.node.active_set_name,
        )
        if not name or not name.strip():
            return
        self.node.rename_waypoint_set(name.strip())
        self.refresh_set_controls()
        self.log(f'Renamed set to: {name.strip()}')

    def delete_set(self):
        if self.node.state != PatrolState.IDLE:
            messagebox.showwarning(
                'Navigation active', 'Stop navigation before deleting a set.'
            )
            return
        if len(self.node.waypoint_sets) <= 1:
            messagebox.showinfo('Keep one set', 'A map must keep at least one set.')
            return
        if not messagebox.askyesno(
            'Archive waypoint set',
            f'Archive “{self.node.active_set_name}”?',
        ):
            return
        old_name = self.node.active_set_name
        self.node.delete_waypoint_set()
        self.refresh_set_controls()
        self.update_list()
        self.set_waypoint_dirty(False)
        self.log(f'Archived set: {old_name}')

    def capture_dock(self):
        if self.node.save_current_pose_as_dock():
            self.refresh_set_controls()
            self.log('Updated map dock from current pose')
        else:
            messagebox.showerror(
                'Dock not saved',
                'A valid map-frame robot pose is required.',
            )

    def go_to_dock(self):
        if self.node.navigate_to_dock():
            self.log('Going to map dock')
        else:
            messagebox.showerror(
                'Cannot go to dock',
                'Save a dock pose and ensure Nav2 is available.',
            )
    
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
            enabled = " " if wp.get('enabled', True) else "×"
            self.waypoint_listbox.insert(tk.END, 
                f"{current}{enabled} {i+1:02d}  {wp['name']:<24} "
                f"{wp['x']:>7.2f}  {wp['y']:>7.2f}")
    
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

    def edit_selected(self):
        """Edit waypoint metadata without crowding the main list."""
        idx = self.get_selected_index()
        if idx is None:
            return
        waypoint = self.node.waypoints[idx]
        dialog = tk.Toplevel(self.root)
        dialog.title('Edit waypoint')
        dialog.transient(self.root)
        dialog.grab_set()
        body = ttk.Frame(dialog, padding='12')
        body.pack(fill='both', expand=True)
        body.columnconfigure(1, weight=1)

        name_var = tk.StringVar(value=waypoint['name'])
        dwell_var = tk.DoubleVar(value=waypoint.get('dwell_seconds', 0.0))
        enabled_var = tk.BooleanVar(value=waypoint.get('enabled', True))
        ttk.Label(body, text='Name').grid(row=0, column=0, sticky='w', pady=4)
        name_entry = ttk.Entry(body, textvariable=name_var, width=32)
        name_entry.grid(row=0, column=1, sticky='ew', pady=4)
        ttk.Label(body, text='Dwell seconds').grid(
            row=1, column=0, sticky='w', pady=4
        )
        ttk.Spinbox(
            body, from_=0.0, to=3600.0, increment=0.5,
            textvariable=dwell_var,
        ).grid(row=1, column=1, sticky='ew', pady=4)
        ttk.Checkbutton(
            body, text='Include in patrol', variable=enabled_var
        ).grid(row=2, column=1, sticky='w', pady=4)
        actions = ttk.Frame(body)
        actions.grid(row=3, column=0, columnspan=2, sticky='e', pady=(10, 0))

        def save():
            name = name_var.get().strip()
            if not name:
                return
            waypoint['name'] = name
            waypoint['dwell_seconds'] = max(0.0, float(dwell_var.get()))
            waypoint['enabled'] = enabled_var.get()
            self.update_list()
            self.set_waypoint_dirty(True)
            self.waypoint_listbox.selection_set(idx)
            dialog.destroy()

        ttk.Button(actions, text='Cancel', command=dialog.destroy).pack(
            side='left', padx=4
        )
        ttk.Button(actions, text='Apply', command=save).pack(side='left')
        name_entry.focus_set()
    
    def clear_all(self):
        if messagebox.askyesno(
            "Clear set", "Remove all waypoints from this set?"
        ):
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
            self.refresh_set_controls()
            self.set_waypoint_dirty(False)
            messagebox.showinfo('Waypoint set saved', msg)
        else:
            self.log(f"ERR: {msg}")

    def reload_waypoints(self):
        if self.waypoints_dirty and not messagebox.askyesno(
            'Discard changes?', 'Reload and discard unsaved changes?'
        ):
            return
        if self.node.load_waypoints():
            self.refresh_set_controls()
            self.update_list()
            self.set_waypoint_dirty(False)
            self.log(
                f"Reloaded {self.node.active_set_name}: "
                f"{len(self.node.waypoints)} waypoints"
            )
        else:
            self.log("Could not load waypoints")
    
    def save_waypoints_as_exhibits(self):
        """Save current waypoints as exhibits for exhibit navigator"""
        exhibit_file = os.path.expanduser(
            '~/sahabat_ws/src/shbat_pkg/config/exhibit_routes.yaml')
        success, msg = self.node.save_as_exhibits(exhibit_file)
        if success:
            messagebox.showinfo("Export Successful", msg)
            self.log(f"OK: {msg}")
        else:
            messagebox.showerror("Export Failed", msg)
            self.log(f"ERR: {msg}")

    def load_waypoints_from_exhibits(self):
        """Load exhibits from exhibit_routes.yaml as waypoints"""
        exhibit_file = os.path.expanduser(
            '~/sahabat_ws/src/shbat_pkg/config/exhibit_routes.yaml')
        success, msg = self.node.load_exhibits(exhibit_file)
        if success:
            self.update_list()
            self.set_waypoint_dirty(True)
            self.log(f"OK: {msg}")
        else:
            self.log(f"ERR: {msg}")

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
