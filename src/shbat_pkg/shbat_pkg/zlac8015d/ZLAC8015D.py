"""
ZLAC8015D AC Servo Driver Python API

Based on rasheeddo/ZLAC8015D_python with modifications for:
- 6.5 inch wheel support (configurable)
- Enhanced odometry
- Better error handling
- ROS2 integration optimizations

Original: https://github.com/rasheeddo/ZLAC8015D_python
"""

from pymodbus.client import ModbusSerialClient as ModbusClient
import numpy as np
import time


class Controller:
    """
    ZLAC8015D Dual-Channel AC Servo Driver Controller
    
    Controls two motors via RS485 Modbus RTU protocol.
    Supports velocity control, position control, and odometry feedback.
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1.0):
        """
        Initialize the ZLAC8015D controller.
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0')
            baudrate: Communication baudrate (default 115200)
            timeout: Read timeout in seconds
        """
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        
        self.client = ModbusClient(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout,
            parity='N',
            stopbits=1,
            bytesize=8
        )
        
        if not self.client.connect():
            raise ConnectionError(f"Failed to connect to ZLAC8015D on {port}")
        
        self.ID = 1  # Modbus slave ID

        ######################
        ## Register Address ##
        ######################
        # Common
        self.CONTROL_REG = 0x200E
        self.OPR_MODE = 0x200D
        self.L_ACL_TIME = 0x2080
        self.R_ACL_TIME = 0x2081
        self.L_DCL_TIME = 0x2082
        self.R_DCL_TIME = 0x2083

        # Velocity control
        self.L_CMD_RPM = 0x2088
        self.R_CMD_RPM = 0x2089
        self.L_FB_RPM = 0x20AB
        self.R_FB_RPM = 0x20AC

        # Position control
        self.POS_CONTROL_TYPE = 0x200F
        self.L_MAX_RPM_POS = 0x208E
        self.R_MAX_RPM_POS = 0x208F
        self.L_CMD_REL_POS_HI = 0x208A
        self.L_CMD_REL_POS_LO = 0x208B
        self.R_CMD_REL_POS_HI = 0x208C
        self.R_CMD_REL_POS_LO = 0x208D
        self.L_FB_POS_HI = 0x20A7
        self.L_FB_POS_LO = 0x20A8
        self.R_FB_POS_HI = 0x20A9
        self.R_FB_POS_LO = 0x20AA

        # Troubleshooting
        self.L_FAULT = 0x20A5
        self.R_FAULT = 0x20A6

        ########################
        ## Control CMDs (REG) ##
        ########################
        self.EMER_STOP = 0x05
        self.ALRM_CLR = 0x06
        self.DOWN_TIME = 0x07
        self.ENABLE = 0x08
        self.POS_SYNC = 0x10
        self.POS_L_START = 0x11
        self.POS_R_START = 0x12

        ####################
        ## Operation Mode ##
        ####################
        self.POS_REL_CONTROL = 1
        self.POS_ABS_CONTROL = 2
        self.VEL_CONTROL = 3

        self.ASYNC = 0
        self.SYNC = 1

        #################
        ## Fault codes ##
        #################
        self.NO_FAULT = 0x0000
        self.OVER_VOLT = 0x0001
        self.UNDER_VOLT = 0x0002
        self.OVER_CURR = 0x0004
        self.OVER_LOAD = 0x0008
        self.CURR_OUT_TOL = 0x0010
        self.ENCOD_OUT_TOL = 0x0020
        self.MOTOR_BAD = 0x0040
        self.REF_VOLT_ERROR = 0x0080
        self.EEPROM_ERROR = 0x0100
        self.WALL_ERROR = 0x0200
        self.HIGH_TEMP = 0x0400
        self.FAULT_LIST = [
            self.OVER_VOLT, self.UNDER_VOLT, self.OVER_CURR,
            self.OVER_LOAD, self.CURR_OUT_TOL, self.ENCOD_OUT_TOL,
            self.MOTOR_BAD, self.REF_VOLT_ERROR, self.EEPROM_ERROR,
            self.WALL_ERROR, self.HIGH_TEMP
        ]
        
        self.FAULT_NAMES = {
            self.OVER_VOLT: "Over Voltage",
            self.UNDER_VOLT: "Under Voltage",
            self.OVER_CURR: "Over Current",
            self.OVER_LOAD: "Overload",
            self.CURR_OUT_TOL: "Current Out of Tolerance",
            self.ENCOD_OUT_TOL: "Encoder Out of Tolerance",
            self.MOTOR_BAD: "Motor Bad",
            self.REF_VOLT_ERROR: "Reference Voltage Error",
            self.EEPROM_ERROR: "EEPROM Error",
            self.WALL_ERROR: "Hall Error",
            self.HIGH_TEMP: "High Temperature"
        }

        ##############
        ## Odometry ##
        ##############
        # Wheel: 173mm diameter (from actual drawings, marketed as 6.5 inch)
        self.R_Wheel = 0.0865  # meters (173mm / 2 = 86.5mm)
        self.travel_in_one_rev = 2 * np.pi * self.R_Wheel  # ~0.5435 meters
        # CPR = PPR × 4 (quadrature decoding)
        # If motor spec says 4096 PPR, then CPR = 4096 × 4 = 16384
        # ZLAC8015D returns quadrature-decoded counts
        self.cpr = 16384  # Counts per revolution (4096 PPR × 4 quadrature)
        
        # Track last encoder values for delta calculations
        self._last_left_tick = None
        self._last_right_tick = None

    def __del__(self):
        """Cleanup on destruction."""
        try:
            self.disable_motor()
            self.client.close()
        except:
            pass

    def close(self):
        """Close the connection."""
        try:
            self.disable_motor()
            self.client.close()
        except:
            pass

    def modbus_fail_read_handler(self, ADDR, WORD, max_retries=3):
        """
        Robust register reading with retry on failure.
        
        Args:
            ADDR: Register address
            WORD: Number of registers to read
            max_retries: Maximum retry attempts
            
        Returns:
            List of register values
        """
        for attempt in range(max_retries):
            try:
                result = self.client.read_holding_registers(address=ADDR, count=WORD, device_id=self.ID)
                if not result.isError() and hasattr(result, 'registers') and len(result.registers) == WORD:
                    return result.registers
            except Exception as e:
                if attempt == max_retries - 1:
                    raise IOError(f"Failed to read register 0x{ADDR:04X} after {max_retries} attempts: {e}")
            time.sleep(0.01)  # Small delay between retries
        
        raise IOError(f"Failed to read register 0x{ADDR:04X}")

    def rpm_to_radPerSec(self, rpm):
        """Convert RPM to radians per second."""
        return rpm * 2 * np.pi / 60.0

    def rpm_to_linear(self, rpm):
        """Convert RPM to linear velocity (m/s)."""
        W_Wheel = self.rpm_to_radPerSec(rpm)
        V = W_Wheel * self.R_Wheel
        return V

    def linear_to_rpm(self, velocity):
        """Convert linear velocity (m/s) to RPM."""
        W_Wheel = velocity / self.R_Wheel
        rpm = W_Wheel * 60.0 / (2 * np.pi)
        return rpm

    # ==================== MODE CONTROL ====================
    
    def set_mode(self, MODE):
        """
        Set operation mode.
        
        Args:
            MODE: 1=Relative Position, 2=Absolute Position, 3=Velocity Control
        """
        if MODE not in [1, 2, 3]:
            raise ValueError("Mode must be 1 (rel pos), 2 (abs pos), or 3 (velocity)")
        
        result = self.client.write_register(address=self.OPR_MODE, value=MODE, device_id=self.ID)
        return result

    def get_mode(self):
        """Get current operation mode."""
        registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)
        return registers[0]

    def enable_motor(self):
        """Enable both motors."""
        return self.client.write_register(address=self.CONTROL_REG, value=self.ENABLE, device_id=self.ID)

    def disable_motor(self):
        """Disable both motors (coast to stop)."""
        return self.client.write_register(address=self.CONTROL_REG, value=self.DOWN_TIME, device_id=self.ID)

    def emergency_stop(self):
        """Emergency stop - immediate halt."""
        return self.client.write_register(address=self.CONTROL_REG, value=self.EMER_STOP, device_id=self.ID)

    # ==================== FAULT HANDLING ====================
    
    def get_fault_code(self):
        """
        Get fault codes for both motors.
        
        Returns:
            Tuple of ((L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code))
        """
        try:
            fault_codes = self.modbus_fail_read_handler(self.L_FAULT, 2)
            L_fault_code = fault_codes[0]
            R_fault_code = fault_codes[1]
            
            L_fault_flag = L_fault_code in self.FAULT_LIST
            R_fault_flag = R_fault_code in self.FAULT_LIST
            
            return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)
        except:
            return (False, 0), (False, 0)

    def get_fault_string(self, fault_code):
        """Get human-readable fault description."""
        if fault_code == 0:
            return "No Fault"
        return self.FAULT_NAMES.get(fault_code, f"Unknown Fault (0x{fault_code:04X})")

    def clear_alarm(self):
        """Clear alarm/fault status."""
        return self.client.write_register(address=self.CONTROL_REG, value=self.ALRM_CLR, device_id=self.ID)

    # ==================== TIMING PARAMETERS ====================
    
    def set_accel_time(self, L_ms, R_ms):
        """
        Set acceleration time for both motors.
        
        Args:
            L_ms: Left motor acceleration time in milliseconds (0-32767)
            R_ms: Right motor acceleration time in milliseconds (0-32767)
        """
        L_ms = max(0, min(32767, int(L_ms)))
        R_ms = max(0, min(32767, int(R_ms)))
        return self.client.write_registers(address=self.L_ACL_TIME, values=[L_ms, R_ms], device_id=self.ID)

    def set_decel_time(self, L_ms, R_ms):
        """
        Set deceleration time for both motors.
        
        Args:
            L_ms: Left motor deceleration time in milliseconds (0-32767)
            R_ms: Right motor deceleration time in milliseconds (0-32767)
        """
        L_ms = max(0, min(32767, int(L_ms)))
        R_ms = max(0, min(32767, int(R_ms)))
        return self.client.write_registers(address=self.L_DCL_TIME, values=[L_ms, R_ms], device_id=self.ID)

    # ==================== VELOCITY CONTROL ====================
    
    def int16Dec_to_int16Hex(self, int16):
        """Convert signed int16 to unsigned representation for Modbus."""
        if int16 < 0:
            int16 = int16 + 65536
        lo_byte = (int16 & 0x00FF)
        hi_byte = (int16 & 0xFF00) >> 8
        all_bytes = (hi_byte << 8) | lo_byte
        return all_bytes

    def set_rpm(self, L_rpm, R_rpm):
        """
        Set target RPM for both motors.
        
        Args:
            L_rpm: Left motor RPM (-3000 to 3000)
            R_rpm: Right motor RPM (-3000 to 3000)
        """
        L_rpm = max(-3000, min(3000, int(L_rpm)))
        R_rpm = max(-3000, min(3000, int(R_rpm)))
        
        left_bytes = self.int16Dec_to_int16Hex(L_rpm)
        right_bytes = self.int16Dec_to_int16Hex(R_rpm)
        
        return self.client.write_registers(address=self.L_CMD_RPM, values=[left_bytes, right_bytes], device_id=self.ID)

    def get_rpm(self):
        """
        Get actual RPM feedback from both motors.
        
        Returns:
            Tuple (left_rpm, right_rpm)
        """
        registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)
        # Handle signed 16-bit values properly (values > 32767 are negative)
        raw_L = registers[0]
        raw_R = registers[1]
        fb_L_rpm = (raw_L if raw_L < 32768 else raw_L - 65536) / 10.0
        fb_R_rpm = (raw_R if raw_R < 32768 else raw_R - 65536) / 10.0
        return fb_L_rpm, fb_R_rpm

    def get_linear_velocities(self):
        """
        Get linear velocities for both wheels in m/s.
        
        Returns:
            Tuple (left_velocity, right_velocity) in m/s
            
        Note: Right motor is inverted because motors face opposite directions.
              Both return positive for forward motion.
        """
        rpmL, rpmR = self.get_rpm()
        VL = self.rpm_to_linear(rpmL)
        VR = self.rpm_to_linear(-rpmR)  # Invert right - motors face opposite directions
        return VL, VR

    def set_velocity(self, left_vel, right_vel):
        """
        Set wheel velocities in m/s.
        
        Args:
            left_vel: Left wheel velocity in m/s (positive = forward)
            right_vel: Right wheel velocity in m/s (positive = forward)
        """
        L_rpm = int(self.linear_to_rpm(left_vel))
        R_rpm = int(-self.linear_to_rpm(right_vel))  # Invert right motor
        return self.set_rpm(L_rpm, R_rpm)

    # ==================== POSITION/ODOMETRY ====================
    
    def get_wheels_travelled(self):
        """
        Get distance travelled by each wheel in meters.
        
        Returns:
            Tuple (left_distance, right_distance) in meters
        """
        registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]
        
        l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
        
        l_travelled = (float(l_pulse) / self.cpr) * self.travel_in_one_rev
        r_travelled = (float(r_pulse) / self.cpr) * self.travel_in_one_rev
        
        return l_travelled, r_travelled

    def get_wheels_tick(self):
        """
        Get raw encoder tick counts.
        
        Returns:
            Tuple (left_ticks, right_ticks)
        """
        registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]
        
        l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
        
        return l_tick, r_tick

    def get_delta_ticks(self):
        """
        Get change in encoder ticks since last call.
        
        Returns:
            Tuple (delta_left, delta_right) in ticks
        """
        l_tick, r_tick = self.get_wheels_tick()
        
        if self._last_left_tick is None:
            self._last_left_tick = l_tick
            self._last_right_tick = r_tick
            return 0, 0
        
        delta_l = l_tick - self._last_left_tick
        delta_r = r_tick - self._last_right_tick
        
        self._last_left_tick = l_tick
        self._last_right_tick = r_tick
        
        return delta_l, delta_r

    def get_delta_distance(self):
        """
        Get change in wheel distance since last call.
        
        Returns:
            Tuple (delta_left, delta_right) in meters
        """
        delta_l, delta_r = self.get_delta_ticks()
        
        dist_l = (float(delta_l) / self.cpr) * self.travel_in_one_rev
        dist_r = (float(delta_r) / self.cpr) * self.travel_in_one_rev
        
        return dist_l, dist_r

    # ==================== POSITION CONTROL ====================
    
    def set_maxRPM_pos(self, max_L_rpm, max_R_rpm):
        """Set maximum RPM for position control mode."""
        max_L_rpm = max(1, min(1000, int(max_L_rpm)))
        max_R_rpm = max(1, min(1000, int(max_R_rpm)))
        return self.client.write_registers(address=self.L_MAX_RPM_POS, values=[max_L_rpm, max_R_rpm], device_id=self.ID)

    def set_position_async_control(self):
        """Set position control to asynchronous mode."""
        return self.client.write_register(address=self.POS_CONTROL_TYPE, value=self.ASYNC, device_id=self.ID)

    def move_left_wheel(self):
        """Trigger left wheel position movement."""
        return self.client.write_register(address=self.CONTROL_REG, value=self.POS_L_START, device_id=self.ID)

    def move_right_wheel(self):
        """Trigger right wheel position movement."""
        return self.client.write_register(address=self.CONTROL_REG, value=self.POS_R_START, device_id=self.ID)

    def _map(self, val, in_min, in_max, out_min, out_max):
        """Map value from one range to another."""
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def deg_to_32bitArray(self, deg):
        """Convert degrees to 32-bit register array."""
        dec = int(self._map(deg, -1440, 1440, -65536, 65536))
        HI_WORD = (dec & 0xFFFF0000) >> 16
        LO_WORD = dec & 0x0000FFFF
        return [HI_WORD, LO_WORD]

    def set_relative_angle(self, ang_L, ang_R):
        """
        Set relative angle movement for both wheels.
        
        Args:
            ang_L: Left wheel angle in degrees
            ang_R: Right wheel angle in degrees
        """
        L_array = self.deg_to_32bitArray(ang_L)
        R_array = self.deg_to_32bitArray(ang_R)
        all_cmds_array = L_array + R_array
        return self.client.write_registers(address=self.L_CMD_REL_POS_HI, values=all_cmds_array, device_id=self.ID)

    # ==================== UTILITY ====================
    
    def reset_encoder(self):
        """Reset encoder position tracking."""
        self._last_left_tick = None
        self._last_right_tick = None

    def is_connected(self):
        """Check if connection is active."""
        return self.client.is_socket_open()

    def reconnect(self):
        """Attempt to reconnect."""
        self.client.close()
        time.sleep(0.1)
        return self.client.connect()
