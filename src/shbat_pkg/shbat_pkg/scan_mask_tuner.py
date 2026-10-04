#!/usr/bin/env python3
"""
Interactive tuner for the rear blind-zone mask in scan_filter.yaml.

Subscribes to the unfiltered /scan_raw and draws it in the robot frame (front
up) with the masked sector shaded. Drag the edge sliders to see which returns
the mask would remove, then save the result to config/scan_filter.yaml.

This tool only subscribes; it never publishes and never moves the robot. Run
the motor-free LIDAR check first:

    ros2 launch shbat_pkg lidar_check.launch.py
    ros2 run shbat_pkg scan_mask_tuner

The RPLIDAR is mounted upside down (lidar_joint rpy="3.14 0 3.14"), so a raw
scan angle theta maps to the robot-frame bearing phi = 180 - theta degrees.
The rear mask is edited as two robot-frame bearings: the left edge (positive,
default 90) and the right edge (negative, default -90). Everything from the
left edge through the rear to the right edge is removed.
"""

import math
import os
import re
import signal
import threading
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk


LIDAR_X_OFFSET = 0.07   # lidar_link is 7 cm ahead of base_link
ROBOT_RADIUS = 0.25     # Nav2 robot_radius
BODY_HINT_RANGE = 0.40  # Returns this close to the lidar are probably the robot


def wrap_deg(angle):
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def lidar_to_robot_deg(theta_deg):
    """Convert a raw upside-down lidar angle to a robot-frame bearing."""
    return wrap_deg(180.0 - theta_deg)


def rear_edges_to_zone(left_edge_deg, right_edge_deg):
    """
    Return the scan_filter lidar-frame zone for robot-frame rear edges.

    left_edge_deg must be in [0, 180] and right_edge_deg in [-180, 0]. The
    masked robot sector runs from the left edge through 180 to the right edge,
    which is one contiguous lidar-frame zone around theta = 0.
    """
    if not 0.0 <= left_edge_deg <= 180.0:
        raise ValueError('left edge must be within [0, 180] degrees')
    if not -180.0 <= right_edge_deg <= 0.0:
        raise ValueError('right edge must be within [-180, 0] degrees')
    return [-(180.0 + right_edge_deg), 180.0 - left_edge_deg]


def zone_to_rear_edges(zone):
    """Inverse of rear_edges_to_zone for a lidar zone containing theta = 0."""
    low, high = min(zone), max(zone)
    return 180.0 - high, -(180.0 + low)


def split_zones(flat_zones):
    """Split scan_filter's flat [a0, b0, a1, b1, ...] list into pairs."""
    return [
        (float(flat_zones[i]), float(flat_zones[i + 1]))
        for i in range(0, len(flat_zones) - 1, 2)
    ]


def find_rear_zone(zones):
    """Return the index of the zone that covers theta = 0, or None."""
    for index, (a, b) in enumerate(zones):
        if min(a, b) <= 0.0 <= max(a, b):
            return index
    return None


def angle_masked(theta_rad, zones_rad):
    """Match scan_filter: true when theta lies inside any inclusive zone."""
    return any(low <= theta_rad <= high for low, high in zones_rad)


def format_zones(zones):
    """Format zone pairs as the YAML flow list scan_filter expects."""
    values = []
    for a, b in zones:
        values.extend([a, b])
    return '[' + ', '.join(f'{value:.1f}' for value in values) + ']'


def replace_filter_zones(yaml_text, zones, comment):
    """
    Replace the filter_zones line in scan_filter.yaml text.

    Only that one line changes, so the file's other values and comments are
    preserved. Raises ValueError when the file has no filter_zones line.
    """
    pattern = re.compile(r'^(\s*)filter_zones:.*$', re.MULTILINE)
    match = pattern.search(yaml_text)
    if match is None:
        raise ValueError('filter_zones line not found')
    line = f'{match.group(1)}filter_zones: {format_zones(zones)}'
    if comment:
        line += f'  # {comment}'
    return yaml_text[:match.start()] + line + yaml_text[match.end():]


def default_config_path():
    """Locate the source scan_filter.yaml (symlink installs point at src)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(
            get_package_share_directory('shbat_pkg'),
            'config',
            'scan_filter.yaml',
        )
        return os.path.realpath(installed)
    except Exception:
        return os.path.expanduser(
            '~/sahabat_ws/src/shbat_pkg/config/scan_filter.yaml'
        )


def load_zones(path):
    """Read the flat filter_zones list without needing PyYAML."""
    with open(path, encoding='utf-8') as stream:
        text = stream.read()
    match = re.search(r'^\s*filter_zones:\s*\[([^\]]*)\]', text, re.MULTILINE)
    if match is None:
        return []
    return [float(value) for value in match.group(1).split(',') if value.strip()]


class ScanMaskTuner:
    """Tk window that previews and saves the rear scan mask."""

    def __init__(self, node, config_path):
        self.node = node
        self.config_path = config_path
        self.lock = threading.Lock()
        self.stop_requested = threading.Event()
        self.latest = None
        self.history = deque(maxlen=20)  # Raw scans for the persistence view

        zones = split_zones(load_zones(config_path))
        rear_index = find_rear_zone(zones)
        if rear_index is None:
            left, right = 90.0, -90.0
            self.other_zones = zones
        else:
            left, right = zone_to_rear_edges(zones[rear_index])
            self.other_zones = zones[:rear_index] + zones[rear_index + 1:]
        self.saved_edges = (left, right)

        self.root = tk.Tk()
        self.root.title('Sahabat Scan Mask Tuner')
        # Size the window from Tk's DPI scaling so HiDPI screens are not clipped.
        ui = max(1.0, float(self.root.tk.call('tk', 'scaling')) / 1.33)
        self.wrap = int(380 * ui)
        self.root.geometry(f'{int(1150 * ui)}x{int(820 * ui)}')
        self.left_edge = tk.DoubleVar(value=left)
        self.right_edge = tk.DoubleVar(value=right)
        self.view_range = tk.DoubleVar(value=3.0)
        self.persist = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value='Waiting for /scan_raw ...')
        self.readout = tk.StringVar(value='')
        self.mouse = None

        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        self.root.after(200, self._redraw)

    def on_scan(self, msg):
        """Store the newest scan; called from the ROS spin thread."""
        with self.lock:
            self.latest = msg
            self.history.append(msg)

    # ---------- layout ----------

    def _build(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill='both', expand=True)

        self.canvas = tk.Canvas(main, background='#101418', highlightthickness=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind('<Motion>', self._on_motion)
        self.canvas.bind('<Leave>', lambda _e: setattr(self, 'mouse', None))

        side = ttk.Frame(main, padding=(10, 0, 0, 0))
        side.pack(side='right', fill='y')

        ttk.Label(side, text='Rear mask (robot frame, front = 0°)',
                  font=('TkDefaultFont', 10, 'bold')).pack(anchor='w')
        self._slider(side, 'Left edge (°)', self.left_edge, 45.0, 180.0)
        self._slider(side, 'Right edge (°)', self.right_edge, -180.0, -45.0)

        nudges = ttk.Frame(side)
        nudges.pack(fill='x', pady=(2, 8))
        for text, dl, dr in (('See 1° more', 1, -1), ('See 1° less', -1, 1)):
            ttk.Button(
                nudges, text=text,
                command=lambda dl=dl, dr=dr: self._nudge(dl, dr),
            ).pack(side='left', expand=True, fill='x', padx=2)

        self.fov_label = ttk.Label(side, text='')
        self.fov_label.pack(anchor='w')
        self.zone_label = ttk.Label(side, text='', wraplength=self.wrap)
        self.zone_label.pack(anchor='w', pady=(0, 8))

        ttk.Separator(side).pack(fill='x', pady=6)
        self._slider(side, 'View range (m)', self.view_range, 0.5, 8.0)
        ttk.Checkbutton(
            side, text='Overlay last 20 scans (shows static robot parts)',
            variable=self.persist,
        ).pack(anchor='w', pady=(4, 8))

        ttk.Separator(side).pack(fill='x', pady=6)
        ttk.Button(side, text='Save to scan_filter.yaml',
                   command=self._save).pack(fill='x')
        ttk.Button(side, text='Revert to saved',
                   command=self._revert).pack(fill='x', pady=(4, 0))
        ttk.Label(side, text=self.config_path, wraplength=self.wrap,
                  foreground='#666').pack(anchor='w', pady=(4, 8))

        ttk.Label(side, textvariable=self.readout, wraplength=self.wrap,
                  font=('TkFixedFont', 9)).pack(anchor='w', pady=(4, 8))

        legend = (
            'green  kept return\n'
            'red    removed by mask\n'
            'orange kept but < 0.40 m from lidar\n'
            '       (likely robot body — widen mask)\n'
            'grey   older scans (overlay)\n'
            'circle Nav2 robot_radius 0.25 m'
        )
        ttk.Label(side, text=legend, font=('TkFixedFont', 9),
                  justify='left').pack(anchor='w')
        ttk.Label(side, text='Saved changes apply after restarting the scan '
                  'filter (re-run the lidar check or the robot stack).',
                  wraplength=self.wrap, foreground='#666').pack(anchor='w', pady=8)
        ttk.Label(side, textvariable=self.status, wraplength=self.wrap).pack(
            side='bottom', anchor='w')

    def _slider(self, parent, label, variable, low, high):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=2)
        ttk.Label(row, text=label, width=14).pack(side='left')
        value = ttk.Label(row, width=6)
        value.pack(side='right')
        scale = ttk.Scale(row, from_=low, to=high, variable=variable,
                          orient='horizontal')
        scale.pack(side='left', fill='x', expand=True)

        def update(*_args):
            value.configure(text=f'{variable.get():.1f}')
        variable.trace_add('write', update)
        update()

    def _nudge(self, d_left, d_right):
        self.left_edge.set(min(180.0, max(45.0, round(self.left_edge.get()) + d_left)))
        self.right_edge.set(min(-45.0, max(-180.0, round(self.right_edge.get()) + d_right)))

    # ---------- geometry ----------

    def _edges(self):
        return round(self.left_edge.get(), 1), round(self.right_edge.get(), 1)

    def _zones(self):
        left, right = self._edges()
        return self.other_zones + [tuple(rear_edges_to_zone(left, right))]

    def _to_canvas(self, x, y, cx, cy, scale):
        # Robot frame: +x forward drawn up, +y left drawn left.
        return cx - y * scale, cy - x * scale

    # ---------- drawing ----------

    def _redraw(self):
        # Ctrl+C/SIGTERM only set a flag; Python runs signal handlers between
        # these periodic callbacks, so close the window from here.
        if self.stop_requested.is_set():
            self.root.destroy()
            return
        try:
            self._draw()
        finally:
            self.root.after(150, self._redraw)

    def _draw(self):
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        cx, cy = w / 2, h / 2
        view = max(0.5, self.view_range.get())
        scale = min(w, h) / 2 / view
        left, right = self._edges()
        zones = self._zones()
        zones_rad = [(math.radians(min(a, b)), math.radians(max(a, b)))
                     for a, b in zones]

        self.fov_label.configure(
            text=f'Visible FOV: {left - right:.1f}°   '
                 f'(saved: {self.saved_edges[0] - self.saved_edges[1]:.1f}°)')
        self.zone_label.configure(
            text=f'filter_zones (lidar frame): {format_zones(zones)}')

        lx, ly = self._to_canvas(LIDAR_X_OFFSET, 0.0, cx, cy, scale)
        radius = view * 1.5 * scale
        # Masked sector: Tk arcs start at +x screen (robot right) and run CCW;
        # robot bearing phi is drawn at screen angle phi + 90.
        c.create_arc(lx - radius, ly - radius, lx + radius, ly + radius,
                     start=left + 90.0, extent=(360.0 + right) - left,
                     fill='#3a1a1a', outline='')
        for edge in (left, right):
            ex = LIDAR_X_OFFSET + view * 1.5 * math.cos(math.radians(edge))
            ey = view * 1.5 * math.sin(math.radians(edge))
            c.create_line(lx, ly, *self._to_canvas(ex, ey, cx, cy, scale),
                          fill='#ff5555', dash=(4, 3))

        for ring in range(1, int(view) + 1):
            r = ring * scale
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline='#2a3038')
            c.create_text(cx + 4, cy - r - 6, text=f'{ring} m',
                          fill='#556', anchor='w')
        c.create_line(cx, 0, cx, h, fill='#20262c')
        c.create_line(0, cy, w, cy, fill='#20262c')
        r = ROBOT_RADIUS * scale
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline='#4aa3ff', width=2)
        c.create_line(cx, cy, cx, cy - r, fill='#4aa3ff', width=2)
        c.create_text(cx, cy - r - 10, text='FRONT', fill='#4aa3ff')

        with self.lock:
            latest = self.latest
            history = list(self.history) if self.persist.get() else []

        if latest is None:
            c.create_text(cx, 30, text='No /scan_raw yet — is the lidar check '
                          'running?', fill='#ddd')
            return

        size = max(1, int(scale * 0.012))
        for msg in history[:-1]:
            for _theta, x, y, _r in self._points(msg, view, step=3):
                px, py = self._to_canvas(x, y, cx, cy, scale)
                c.create_rectangle(px, py, px + 1, py + 1,
                                   outline='#4a5058', fill='#4a5058')

        nearest = {}
        for theta, x, y, rng in self._points(latest, view, step=1):
            masked = angle_masked(theta, zones_rad)
            if masked:
                color = '#ff4444'
            elif rng < BODY_HINT_RANGE:
                color = '#ffa500'
            else:
                color = '#44dd66'
            px, py = self._to_canvas(x, y, cx, cy, scale)
            c.create_rectangle(px - size, py - size, px + size, py + size,
                               outline=color, fill=color)
            bearing = round(lidar_to_robot_deg(math.degrees(theta)))
            nearest[bearing] = min(rng, nearest.get(bearing, rng))

        self._update_readout(nearest, cx, cy, scale)
        self.status.set(
            f'{len(latest.ranges)} beams, frame {latest.header.frame_id}, '
            f'{len(self.history)} scans buffered')

    def _points(self, msg, view, step):
        """Yield (theta, x, y, range) in base_link for finite returns."""
        limit = view * 1.5
        for i in range(0, len(msg.ranges), step):
            rng = msg.ranges[i]
            if not math.isfinite(rng) or rng <= 0.0 or rng > limit:
                continue
            theta = msg.angle_min + i * msg.angle_increment
            phi = math.radians(lidar_to_robot_deg(math.degrees(theta)))
            yield (theta,
                   LIDAR_X_OFFSET + rng * math.cos(phi),
                   rng * math.sin(phi),
                   rng)

    def _on_motion(self, event):
        self.mouse = (event.x, event.y)

    def _update_readout(self, nearest, cx, cy, scale):
        lines = []
        if self.mouse is not None:
            # Bearing from the lidar origin to the cursor.
            lx, ly = self._to_canvas(LIDAR_X_OFFSET, 0.0, cx, cy, scale)
            dx, dy = self.mouse[0] - lx, self.mouse[1] - ly
            bearing = round(math.degrees(math.atan2(-dx, -dy)))
            rng = nearest.get(bearing)
            lines.append(f'cursor bearing {bearing:+4d}°  nearest '
                         + (f'{rng:.2f} m' if rng is not None else 'none'))
        left, right = self._edges()
        for name, edge in (('left', left), ('right', right)):
            base = round(edge)
            cells = []
            for offset in (-3, -2, -1, 0, 1, 2, 3):
                rng = nearest.get(int(wrap_deg(base + offset)))
                cells.append(f'{rng:4.2f}' if rng is not None else ' -- ')
            lines.append(f'{name} edge {base:+d}° ±3: ' + ' '.join(cells))
        self.readout.set('\n'.join(lines))

    # ---------- persistence ----------

    def _save(self):
        left, right = self._edges()
        zones = self._zones()
        comment = (f'rear mask: robot-frame left edge {left:.1f}°, '
                   f'right edge {right:.1f}°, visible {left - right:.1f}°')
        if not messagebox.askyesno(
                'Save mask',
                f'Write filter_zones: {format_zones(zones)}\n\nto '
                f'{self.config_path}?\n\nRestart the scan filter to apply.'):
            return
        try:
            with open(self.config_path, encoding='utf-8') as stream:
                text = stream.read()
            text = replace_filter_zones(text, zones, comment)
            with open(self.config_path, 'w', encoding='utf-8') as stream:
                stream.write(text)
        except (OSError, ValueError) as error:
            messagebox.showerror('Save failed', str(error))
            return
        self.saved_edges = (left, right)
        self.status.set(f'Saved {format_zones(zones)}')

    def _revert(self):
        self.left_edge.set(self.saved_edges[0])
        self.right_edge.set(self.saved_edges[1])


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan

    # rclpy's own handlers raise inside Tk callbacks and stall the redraw
    # loop, so this tool installs plain flag-setting handlers instead.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('scan_mask_tuner')
    node.declare_parameter('scan_topic', '/scan_raw')
    node.declare_parameter('config_path', default_config_path())
    topic = node.get_parameter('scan_topic').value
    config_path = node.get_parameter('config_path').value

    tuner = ScanMaskTuner(node, config_path)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: tuner.stop_requested.set())
    node.create_subscription(LaserScan, topic, tuner.on_scan,
                             qos_profile_sensor_data)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    try:
        tuner.root.mainloop()
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
