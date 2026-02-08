#!/usr/bin/env python3
"""
Waypoint Control Panel - GUI with buttons for waypoint operations

Run this alongside your navigation launch to get a control panel with buttons.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import tkinter as tk
from tkinter import ttk, messagebox
import threading


class WaypointControlPanel:
    def __init__(self):
        # Initialize ROS
        rclpy.init()
        self.node = rclpy.create_node('waypoint_control_panel')
        
        # Service clients
        self.clients = {
            'save': self.node.create_client(Trigger, '/waypoint_collector/save'),
            'clear': self.node.create_client(Trigger, '/waypoint_collector/clear'),
            'undo': self.node.create_client(Trigger, '/waypoint_collector/undo'),
            'list': self.node.create_client(Trigger, '/waypoint_collector/list'),
            'patrol_pause': self.node.create_client(Trigger, '/waypoint_patrol/pause'),
            'patrol_resume': self.node.create_client(Trigger, '/waypoint_patrol/resume'),
            'patrol_skip': self.node.create_client(Trigger, '/waypoint_patrol/skip'),
            'patrol_stop': self.node.create_client(Trigger, '/waypoint_patrol/stop'),
            'record': self.node.create_client(Trigger, '/waypoint_patrol/record_waypoint'),
        }
        
        # Start ROS spinning in background
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Create GUI
        self.create_gui()
    
    def spin_ros(self):
        """Spin ROS in background thread"""
        rclpy.spin(self.node)
    
    def call_service(self, service_name):
        """Call a service and show result"""
        if service_name not in self.clients:
            self.log(f"Unknown service: {service_name}")
            return
        
        client = self.clients[service_name]
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.log(f"Service {service_name} not available")
            messagebox.showwarning("Service Unavailable", 
                f"Service '{service_name}' is not available.\n\n"
                "Make sure the appropriate node is running:\n"
                "- waypoint_collector for collect commands\n"
                "- waypoint_patrol for patrol commands")
            return
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Wait for result (with timeout)
        self.log(f"Calling {service_name}...")
        
        def check_result():
            if future.done():
                try:
                    result = future.result()
                    if result.success:
                        self.log(f"✓ {result.message}")
                    else:
                        self.log(f"✗ {result.message}")
                except Exception as e:
                    self.log(f"Error: {e}")
            else:
                self.root.after(100, check_result)
        
        self.root.after(100, check_result)
    
    def log(self, message):
        """Add message to log"""
        self.log_text.config(state='normal')
        self.log_text.insert('end', message + '\n')
        self.log_text.see('end')
        self.log_text.config(state='disabled')
    
    def clear_log(self):
        """Clear log"""
        self.log_text.config(state='normal')
        self.log_text.delete('1.0', 'end')
        self.log_text.config(state='disabled')
    
    def create_gui(self):
        """Create the GUI"""
        self.root = tk.Tk()
        self.root.title("Waypoint Control Panel")
        self.root.geometry("400x500")
        self.root.resizable(True, True)
        
        # Style
        style = ttk.Style()
        style.configure('Green.TButton', foreground='green')
        style.configure('Red.TButton', foreground='red')
        style.configure('Blue.TButton', foreground='blue')
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill='both', expand=True)
        
        # ===== WAYPOINT COLLECTOR SECTION =====
        collector_frame = ttk.LabelFrame(main_frame, text="📍 Waypoint Collector", padding="10")
        collector_frame.pack(fill='x', pady=5)
        
        ttk.Label(collector_frame, text="Click '2D Goal Pose' in RViz to add waypoints").pack()
        
        btn_frame1 = ttk.Frame(collector_frame)
        btn_frame1.pack(fill='x', pady=5)
        
        ttk.Button(btn_frame1, text="💾 Save Waypoints", width=18,
                   command=lambda: self.call_service('save')).pack(side='left', padx=2)
        ttk.Button(btn_frame1, text="↩️ Undo Last", width=18,
                   command=lambda: self.call_service('undo')).pack(side='left', padx=2)
        
        btn_frame2 = ttk.Frame(collector_frame)
        btn_frame2.pack(fill='x', pady=5)
        
        ttk.Button(btn_frame2, text="📋 List Waypoints", width=18,
                   command=lambda: self.call_service('list')).pack(side='left', padx=2)
        ttk.Button(btn_frame2, text="🗑️ Clear All", width=18,
                   command=lambda: self.call_service('clear')).pack(side='left', padx=2)
        
        # ===== WAYPOINT PATROL SECTION =====
        patrol_frame = ttk.LabelFrame(main_frame, text="🚗 Waypoint Patrol", padding="10")
        patrol_frame.pack(fill='x', pady=5)
        
        btn_frame3 = ttk.Frame(patrol_frame)
        btn_frame3.pack(fill='x', pady=5)
        
        ttk.Button(btn_frame3, text="⏸️ Pause", width=12,
                   command=lambda: self.call_service('patrol_pause')).pack(side='left', padx=2)
        ttk.Button(btn_frame3, text="▶️ Resume", width=12,
                   command=lambda: self.call_service('patrol_resume')).pack(side='left', padx=2)
        ttk.Button(btn_frame3, text="⏭️ Skip", width=12,
                   command=lambda: self.call_service('patrol_skip')).pack(side='left', padx=2)
        
        btn_frame4 = ttk.Frame(patrol_frame)
        btn_frame4.pack(fill='x', pady=5)
        
        ttk.Button(btn_frame4, text="⏹️ Stop Patrol", width=20,
                   command=lambda: self.call_service('patrol_stop')).pack(side='left', padx=2)
        ttk.Button(btn_frame4, text="📍 Record Current Pos", width=20,
                   command=lambda: self.call_service('record')).pack(side='left', padx=2)
        
        # ===== INSTRUCTIONS =====
        info_frame = ttk.LabelFrame(main_frame, text="ℹ️ Instructions", padding="10")
        info_frame.pack(fill='x', pady=5)
        
        instructions = """1. Start waypoint_collector node
2. Click "2D Goal Pose" in RViz to add waypoints
3. Click "Save Waypoints" when done
4. Start waypoint_patrol node
5. Use Pause/Resume/Skip to control patrol"""
        
        ttk.Label(info_frame, text=instructions, justify='left').pack()
        
        # ===== LOG SECTION =====
        log_frame = ttk.LabelFrame(main_frame, text="📝 Log", padding="5")
        log_frame.pack(fill='both', expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, height=8, state='disabled', wrap='word')
        self.log_text.pack(fill='both', expand=True, side='left')
        
        scrollbar = ttk.Scrollbar(log_frame, orient='vertical', command=self.log_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.log_text['yscrollcommand'] = scrollbar.set
        
        ttk.Button(main_frame, text="Clear Log", command=self.clear_log).pack(pady=5)
        
        # Initial log message
        self.log("Waypoint Control Panel Ready")
        self.log("Make sure to run:")
        self.log("  ros2 run shbat_pkg waypoint_collector")
        self.log("  OR")
        self.log("  ros2 run shbat_pkg waypoint_patrol")
    
    def run(self):
        """Run the GUI"""
        try:
            self.root.mainloop()
        finally:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    panel = WaypointControlPanel()
    panel.run()


if __name__ == '__main__':
    main()
