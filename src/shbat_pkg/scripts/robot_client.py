#!/usr/bin/env python3
"""
Robot API Client - For Raspberry Pi / LLM Integration

This client connects to the Sahabat robot's API bridge to:
- Send navigation/waypoint commands
- Monitor robot status (battery, position, stuck detection)
- Control patrol behavior

Usage:
    from robot_client import RobotClient
    
    robot = RobotClient("192.168.1.100")  # Robot's IP
    
    # Send robot to a location
    robot.navigate(x=1.5, y=2.0, yaw=0.0)
    
    # Check status
    status = robot.get_status()
    print(f"Battery: {status['battery_percentage']}%")
    print(f"Is stuck: {status['is_stuck']}")
    
    # Start patrol with waypoints
    robot.start_patrol([
        {"name": "Lobby", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"name": "Room A", "x": 3.0, "y": 1.0, "yaw": 1.57},
        {"name": "Room B", "x": 3.0, "y": -2.0, "yaw": -1.57},
    ])

Author: Sahabat Robot Team
"""

import requests
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass


@dataclass
class RobotStatus:
    """Robot status data structure"""
    battery_percentage: float
    battery_voltage: float
    battery_charging: bool
    position_x: float
    position_y: float
    orientation_yaw: float
    linear_velocity: float
    angular_velocity: float
    nav_state: str
    current_goal_x: Optional[float]
    current_goal_y: Optional[float]
    current_waypoint_index: int
    total_waypoints: int
    is_stuck: bool
    stuck_duration: float
    error_message: str
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'RobotStatus':
        return cls(**data)
    
    def is_navigating(self) -> bool:
        return self.nav_state in ['navigating', 'patrolling']
    
    def is_idle(self) -> bool:
        return self.nav_state == 'idle'
    
    def reached_goal(self) -> bool:
        return self.nav_state == 'reached_goal'
    
    def has_failed(self) -> bool:
        return self.nav_state == 'failed'


class RobotClient:
    """Client for communicating with Sahabat robot API"""
    
    def __init__(self, robot_ip: str, port: int = 5000, timeout: float = 5.0):
        """
        Initialize robot client
        
        Args:
            robot_ip: IP address of the robot (e.g., "192.168.1.100")
            port: API port (default 5000)
            timeout: Request timeout in seconds
        """
        self.base_url = f"http://{robot_ip}:{port}"
        self.timeout = timeout
    
    def _request(self, method: str, endpoint: str, data: Optional[Dict] = None) -> Dict:
        """Make HTTP request to robot API"""
        url = f"{self.base_url}{endpoint}"
        try:
            if method == 'GET':
                response = requests.get(url, timeout=self.timeout)
            elif method == 'POST':
                response = requests.post(url, json=data, timeout=self.timeout)
            else:
                raise ValueError(f"Unsupported method: {method}")
            
            response.raise_for_status()
            return response.json()
        except requests.exceptions.ConnectionError:
            raise ConnectionError(f"Cannot connect to robot at {self.base_url}")
        except requests.exceptions.Timeout:
            raise TimeoutError(f"Request to {url} timed out")
        except requests.exceptions.RequestException as e:
            raise RuntimeError(f"Request failed: {e}")
    
    def health_check(self) -> bool:
        """Check if robot API is reachable"""
        try:
            result = self._request('GET', '/health')
            return result.get('status') == 'ok'
        except:
            return False
    
    def get_status(self) -> RobotStatus:
        """Get current robot status"""
        data = self._request('GET', '/status')
        return RobotStatus.from_dict(data)
    
    def get_status_dict(self) -> Dict:
        """Get current robot status as dictionary"""
        return self._request('GET', '/status')
    
    def navigate(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        Navigate to a specific pose
        
        Args:
            x: X coordinate in map frame (meters)
            y: Y coordinate in map frame (meters)
            yaw: Orientation in radians (0 = facing +X)
            
        Returns:
            True if navigation started successfully
        """
        result = self._request('POST', '/navigate', {'x': x, 'y': y, 'yaw': yaw})
        return result.get('success', False)
    
    def navigate_and_wait(self, x: float, y: float, yaw: float = 0.0, 
                          timeout: float = 120.0, poll_interval: float = 1.0) -> bool:
        """
        Navigate to pose and wait for completion
        
        Args:
            x, y, yaw: Target pose
            timeout: Maximum time to wait (seconds)
            poll_interval: How often to check status
            
        Returns:
            True if goal reached, False if failed/cancelled/timeout
        """
        if not self.navigate(x, y, yaw):
            return False
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            
            if status.reached_goal():
                return True
            elif status.has_failed():
                return False
            elif status.nav_state == 'cancelled':
                return False
            elif not status.is_navigating():
                return False
            
            time.sleep(poll_interval)
        
        # Timeout - cancel navigation
        self.cancel()
        return False
    
    def get_waypoints(self) -> List[Dict]:
        """Get current waypoint list"""
        result = self._request('GET', '/waypoints')
        return result.get('waypoints', [])
    
    def set_waypoints(self, waypoints: List[Dict]) -> bool:
        """
        Set waypoint list
        
        Args:
            waypoints: List of waypoint dicts with keys: name, x, y, yaw
            
        Example:
            robot.set_waypoints([
                {"name": "Lobby", "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"name": "Room A", "x": 3.0, "y": 1.0, "yaw": 1.57},
            ])
        """
        result = self._request('POST', '/waypoints', {'waypoints': waypoints})
        return result.get('success', False)
    
    def start_patrol(self, waypoints: Optional[List[Dict]] = None, loop: bool = True) -> bool:
        """
        Start waypoint patrol
        
        Args:
            waypoints: Optional list of waypoints (uses saved if not provided)
            loop: Whether to loop continuously
        """
        data = {'loop': loop}
        if waypoints:
            data['waypoints'] = waypoints
        
        result = self._request('POST', '/patrol/start', data)
        return result.get('success', False)
    
    def stop_patrol(self) -> bool:
        """Stop current patrol"""
        result = self._request('POST', '/patrol/stop')
        return result.get('success', False)
    
    def cancel(self) -> bool:
        """Cancel current navigation/patrol"""
        result = self._request('POST', '/cancel')
        return result.get('success', False)
    
    def emergency_stop(self) -> bool:
        """Emergency stop - immediately halt all motion"""
        result = self._request('POST', '/emergency_stop')
        return result.get('success', False)
    
    def wait_until_idle(self, timeout: float = 60.0, poll_interval: float = 1.0) -> RobotStatus:
        """Wait until robot is idle (not navigating)"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if not status.is_navigating():
                return status
            time.sleep(poll_interval)
        return self.get_status()


# ============================================================================
# LLM Integration Helper Functions
# ============================================================================

def create_llm_tools(robot: RobotClient) -> List[Dict]:
    """
    Create tool definitions for LLM function calling
    
    Use these with OpenAI/Claude/etc function calling APIs
    """
    return [
        {
            "type": "function",
            "function": {
                "name": "navigate_robot",
                "description": "Navigate the robot to a specific location. Use this when the user asks the robot to go somewhere.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number", "description": "X coordinate in meters"},
                        "y": {"type": "number", "description": "Y coordinate in meters"},
                        "yaw": {"type": "number", "description": "Orientation in radians (optional)", "default": 0.0}
                    },
                    "required": ["x", "y"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "get_robot_status",
                "description": "Get the current status of the robot including battery level, position, and navigation state.",
                "parameters": {"type": "object", "properties": {}}
            }
        },
        {
            "type": "function",
            "function": {
                "name": "start_patrol",
                "description": "Start the robot patrolling through predefined waypoints.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "loop": {"type": "boolean", "description": "Whether to loop continuously", "default": True}
                    }
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "stop_robot",
                "description": "Stop the robot's current navigation or patrol.",
                "parameters": {"type": "object", "properties": {}}
            }
        },
        {
            "type": "function",
            "function": {
                "name": "emergency_stop",
                "description": "Emergency stop - immediately halt all robot motion. Use only in urgent situations.",
                "parameters": {"type": "object", "properties": {}}
            }
        }
    ]


def execute_llm_tool(robot: RobotClient, tool_name: str, args: Dict) -> str:
    """
    Execute a tool call from an LLM
    
    Args:
        robot: RobotClient instance
        tool_name: Name of the tool to execute
        args: Arguments from the LLM
        
    Returns:
        String response to feed back to the LLM
    """
    try:
        if tool_name == "navigate_robot":
            success = robot.navigate(args['x'], args['y'], args.get('yaw', 0.0))
            if success:
                return f"Navigation started to ({args['x']}, {args['y']})"
            else:
                return "Failed to start navigation"
        
        elif tool_name == "get_robot_status":
            status = robot.get_status()
            return (
                f"Battery: {status.battery_percentage:.1f}%\n"
                f"Position: ({status.position_x:.2f}, {status.position_y:.2f})\n"
                f"State: {status.nav_state}\n"
                f"Is stuck: {status.is_stuck}"
            )
        
        elif tool_name == "start_patrol":
            success = robot.start_patrol(loop=args.get('loop', True))
            if success:
                return "Patrol started"
            else:
                return "Failed to start patrol"
        
        elif tool_name == "stop_robot":
            robot.cancel()
            return "Robot stopped"
        
        elif tool_name == "emergency_stop":
            robot.emergency_stop()
            return "Emergency stop executed"
        
        else:
            return f"Unknown tool: {tool_name}"
    
    except Exception as e:
        return f"Error executing {tool_name}: {e}"


# ============================================================================
# Status Monitor (for continuous monitoring)
# ============================================================================

class RobotMonitor:
    """Monitor robot status and trigger callbacks on events"""
    
    def __init__(self, robot: RobotClient, poll_interval: float = 2.0):
        self.robot = robot
        self.poll_interval = poll_interval
        self.running = False
        
        # Callbacks
        self.on_stuck = None
        self.on_low_battery = None
        self.on_goal_reached = None
        self.on_navigation_failed = None
        
        self.low_battery_threshold = 20.0  # percent
        self._last_state = None
    
    def start(self):
        """Start monitoring in background thread"""
        import threading
        self.running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Stop monitoring"""
        self.running = False
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            try:
                status = self.robot.get_status()
                
                # Check for stuck condition
                if status.is_stuck and self.on_stuck:
                    self.on_stuck(status)
                
                # Check for low battery
                if status.battery_percentage < self.low_battery_threshold and self.on_low_battery:
                    self.on_low_battery(status)
                
                # Check for state transitions
                if self._last_state != status.nav_state:
                    if status.nav_state == 'reached_goal' and self.on_goal_reached:
                        self.on_goal_reached(status)
                    elif status.nav_state == 'failed' and self.on_navigation_failed:
                        self.on_navigation_failed(status)
                    self._last_state = status.nav_state
                
            except Exception as e:
                print(f"Monitor error: {e}")
            
            time.sleep(self.poll_interval)


# ============================================================================
# Example Usage
# ============================================================================

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Robot API Client')
    parser.add_argument('--ip', default='localhost', help='Robot IP address')
    parser.add_argument('--port', type=int, default=5000, help='API port')
    args = parser.parse_args()
    
    print(f"Connecting to robot at {args.ip}:{args.port}...")
    robot = RobotClient(args.ip, args.port)
    
    # Health check
    if not robot.health_check():
        print("ERROR: Cannot connect to robot API")
        exit(1)
    
    print("Connected!")
    
    # Get and print status
    status = robot.get_status()
    print(f"\nRobot Status:")
    print(f"  Battery: {status.battery_percentage:.1f}%")
    print(f"  Position: ({status.position_x:.2f}, {status.position_y:.2f})")
    print(f"  Orientation: {status.orientation_yaw:.2f} rad")
    print(f"  State: {status.nav_state}")
    print(f"  Is stuck: {status.is_stuck}")
    
    # Example: Navigate to origin
    print("\nSending robot to (0, 0)...")
    if robot.navigate(0.0, 0.0, 0.0):
        print("Navigation started!")
        
        # Wait for completion
        print("Waiting for goal...")
        final_status = robot.wait_until_idle(timeout=60)
        print(f"Final state: {final_status.nav_state}")
    else:
        print("Failed to start navigation")
