"""Tk settings and live-monitor application for JUNCTEK KG-F devices."""

from __future__ import annotations

from collections import deque
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from .junctek_kgf import JunctekKGFClient, SETTING_SPECS


SETTING_ORDER = [
    'over_voltage_v',
    'under_voltage_v',
    'positive_over_current_a',
    'negative_over_current_a',
    'over_power_w',
    'over_temperature_c',
    'recovery_time_s',
    'delay_time_s',
    'capacity_ah',
    'voltage_calibration',
    'current_calibration',
    'temperature_calibration',
    'relay_normally_closed',
    'current_ratio',
    'voltage_curve_scale',
    'current_curve_scale',
]


class JunctekMonitorApp:
    """Direct serial dashboard and guarded settings editor."""

    def __init__(self, root):
        self.root = root
        self.root.title('Sahabat JUNCTEK KG-F Monitor')
        self.root.minsize(900, 650)
        self._events = queue.Queue()
        self._commands = queue.Queue()
        self._stop_event = threading.Event()
        self._worker = None
        self._connected = False
        self._last_settings = None
        self._voltage_history = deque(maxlen=120)
        self._current_history = deque(maxlen=120)

        self.port_var = tk.StringVar(value='/dev/junctek')
        self.address_var = tk.IntVar(value=1)
        self.connection_var = tk.StringVar(value='Disconnected')
        self.remaining_percent_var = tk.DoubleVar(value=100.0)
        self.live_vars = {
            name: tk.StringVar(value='—')
            for name in (
                'battery', 'voltage', 'current', 'power', 'temperature',
                'remaining_ah', 'capacity_ah', 'energy_wh', 'runtime',
                'remaining_time', 'resistance', 'output', 'serial',
            )
        }
        self.setting_vars = {
            name: tk.StringVar(value='') for name in SETTING_ORDER
        }
        self.setting_widgets = {}
        self._build_ui()
        self.root.after(100, self._drain_events)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_ui(self):
        toolbar = ttk.Frame(self.root, padding=8)
        toolbar.pack(fill='x')
        ttk.Label(toolbar, text='Port').pack(side='left')
        self.port_entry = ttk.Entry(
            toolbar, textvariable=self.port_var, width=38
        )
        self.port_entry.pack(side='left', padx=(5, 12))
        ttk.Label(toolbar, text='Address').pack(side='left')
        self.address_spin = ttk.Spinbox(
            toolbar, from_=1, to=99, textvariable=self.address_var, width=5
        )
        self.address_spin.pack(side='left', padx=(5, 12))
        self.connect_button = ttk.Button(
            toolbar, text='Connect', command=self._toggle_connection
        )
        self.connect_button.pack(side='left')
        ttk.Label(
            toolbar, textvariable=self.connection_var
        ).pack(side='left', padx=15)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        dashboard = ttk.Frame(notebook, padding=12)
        settings = ttk.Frame(notebook, padding=12)
        maintenance = ttk.Frame(notebook, padding=12)
        log_tab = ttk.Frame(notebook, padding=8)
        notebook.add(dashboard, text='Live dashboard')
        notebook.add(settings, text='Settings')
        notebook.add(maintenance, text='Maintenance')
        notebook.add(log_tab, text='Log')

        cards = [
            ('Battery', 'battery'), ('Voltage', 'voltage'),
            ('Current', 'current'), ('Power', 'power'),
            ('Temperature', 'temperature'), ('Output', 'output'),
            ('Remaining', 'remaining_ah'), ('Configured capacity', 'capacity_ah'),
            ('Accumulated energy', 'energy_wh'), ('Runtime', 'runtime'),
            ('Estimated time', 'remaining_time'), ('Internal resistance', 'resistance'),
        ]
        for index, (label, key) in enumerate(cards):
            frame = ttk.LabelFrame(dashboard, text=label, padding=10)
            frame.grid(
                row=index // 3, column=index % 3,
                sticky='nsew', padx=5, pady=5,
            )
            ttk.Label(
                frame, textvariable=self.live_vars[key],
                font=('Sans', 15, 'bold'),
            ).pack()
        for column in range(3):
            dashboard.columnconfigure(column, weight=1)
        for row in range(4):
            dashboard.rowconfigure(row, weight=1)
        identity = ttk.Frame(dashboard)
        identity.grid(row=4, column=0, columnspan=3, sticky='ew', pady=8)
        ttk.Label(identity, text='Device serial:').pack(side='left')
        ttk.Label(identity, textvariable=self.live_vars['serial']).pack(
            side='left', padx=5
        )

        self.graph = tk.Canvas(
            dashboard, height=150, background='#15191d', highlightthickness=0
        )
        self.graph.grid(row=5, column=0, columnspan=3, sticky='nsew', pady=5)
        dashboard.rowconfigure(5, weight=2)

        warning = (
            'Only changed values are written. Protection, capacity, relay, and '
            'calibration changes affect the KG-F itself; verify every value '
            'against the battery and shunt before applying.'
        )
        ttk.Label(
            settings, text=warning, wraplength=820, foreground='#a34b00'
        ).grid(row=0, column=0, columnspan=4, sticky='w', pady=(0, 12))
        for row, name in enumerate(SETTING_ORDER, start=1):
            spec = SETTING_SPECS[name]
            ttk.Label(settings, text=spec.label).grid(
                row=row, column=0, sticky='w', padx=(0, 8), pady=3
            )
            if name == 'relay_normally_closed':
                widget = ttk.Combobox(
                    settings,
                    textvariable=self.setting_vars[name],
                    values=('Normally open', 'Normally closed'),
                    state='readonly',
                    width=24,
                )
            else:
                widget = ttk.Entry(
                    settings, textvariable=self.setting_vars[name], width=26
                )
            widget.grid(row=row, column=1, sticky='ew', pady=3)
            self.setting_widgets[name] = widget
            ttk.Label(settings, text=spec.unit).grid(
                row=row, column=2, sticky='w', padx=6
            )
            ttk.Label(
                settings,
                text=f'{spec.minimum:g} … {spec.maximum:g}',
                foreground='#666666',
            ).grid(row=row, column=3, sticky='w')
        settings.columnconfigure(1, weight=1)
        controls = ttk.Frame(settings)
        controls.grid(
            row=len(SETTING_ORDER) + 1, column=0, columnspan=4,
            sticky='ew', pady=(12, 0),
        )
        ttk.Button(
            controls, text='Read settings',
            command=lambda: self._commands.put(('refresh_settings', None)),
        ).pack(side='left')
        ttk.Button(
            controls, text='Apply changed settings',
            command=self._apply_changed_settings,
        ).pack(side='left', padx=8)

        ttk.Label(
            maintenance,
            text=(
                'These actions change live meter state. They are never run by '
                'the ROS battery node and always require confirmation here.'
            ),
            wraplength=820,
        ).pack(anchor='w', pady=(0, 12))
        percent_frame = ttk.LabelFrame(
            maintenance, text='Remaining capacity', padding=10
        )
        percent_frame.pack(fill='x', pady=6)
        ttk.Spinbox(
            percent_frame, from_=0, to=100,
            textvariable=self.remaining_percent_var, width=8,
        ).pack(side='left')
        ttk.Label(percent_frame, text='%').pack(side='left', padx=5)
        ttk.Button(
            percent_frame, text='Set remaining percentage',
            command=self._set_remaining_percentage,
        ).pack(side='left', padx=10)

        relay_frame = ttk.LabelFrame(
            maintenance, text='Output / data recording', padding=10
        )
        relay_frame.pack(fill='x', pady=6)
        ttk.Button(
            relay_frame, text='Turn ON',
            command=lambda: self._confirm_action(
                'Turn KG-F output/data recording ON?', 10, 1
            ),
        ).pack(side='left')
        ttk.Button(
            relay_frame, text='Turn OFF',
            command=lambda: self._confirm_action(
                'Turn KG-F output/data recording OFF?', 10, 0
            ),
        ).pack(side='left', padx=8)

        dangerous = ttk.LabelFrame(
            maintenance, text='Destructive / calibration actions', padding=10
        )
        dangerous.pack(fill='x', pady=6)
        ttk.Button(
            dangerous, text='Zero current now',
            command=lambda: self._confirm_action(
                'Zero the current reading now? Ensure actual current is 0 A.',
                61, 1,
            ),
        ).pack(side='left')
        ttk.Button(
            dangerous, text='Clear accumulated data',
            command=lambda: self._confirm_action(
                'Clear accumulated Ah, Wh, and runtime data?', 62, 1
            ),
        ).pack(side='left', padx=8)
        ttk.Button(
            dangerous, text='Factory reset',
            command=self._factory_reset,
        ).pack(side='left')

        self.log = tk.Text(log_tab, wrap='word', state='disabled')
        scrollbar = ttk.Scrollbar(
            log_tab, orient='vertical', command=self.log.yview
        )
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

    def _toggle_connection(self):
        if self._worker and self._worker.is_alive():
            self._disconnect()
            return
        self._stop_event.clear()
        port = self.port_var.get().strip()
        address = int(self.address_var.get())
        self._worker = threading.Thread(
            target=self._serial_worker,
            args=(port, address),
            name='junctek-monitor-serial',
            daemon=True,
        )
        self._worker.start()
        self.connection_var.set('Connecting…')
        self.connect_button.configure(text='Disconnect')
        self.port_entry.configure(state='disabled')
        self.address_spin.configure(state='disabled')

    def _disconnect(self):
        self._connected = False
        self._stop_event.set()
        self._commands.put(('disconnect', None))
        self.connection_var.set('Disconnecting…')

    def _serial_worker(self, port, address):
        client = JunctekKGFClient(port=port, address=address, timeout=0.7)
        next_settings = 0.0
        try:
            while not self._stop_event.is_set():
                try:
                    client.open()
                    identity = client.read_identity()
                    self._events.put(('connected', identity))
                    next_settings = 0.0
                    while not self._stop_event.is_set():
                        while True:
                            try:
                                command, payload = self._commands.get_nowait()
                            except queue.Empty:
                                break
                            if command == 'disconnect':
                                return
                            if command == 'refresh_settings':
                                next_settings = 0.0
                            elif command == 'write_settings':
                                for name, value in payload.items():
                                    client.write_setting(name, value)
                                    self._events.put((
                                        'log', f'Wrote {name} = {value}'
                                    ))
                                next_settings = 0.0
                            elif command == 'write_raw':
                                function, value = payload
                                client.write_raw(function, value)
                                self._events.put((
                                    'log', f'Sent W{function:02d} value {value}'
                                ))
                                next_settings = 0.0
                        now = time.monotonic()
                        if now >= next_settings:
                            settings = client.read_settings()
                            self._events.put(('settings', settings))
                            next_settings = float('inf')
                        measurement = client.read_measurement()
                        self._events.put(('measurement', measurement))
                        self._stop_event.wait(1.0)
                except Exception as error:
                    self._events.put((
                        'error', f'{type(error).__name__}: {error}'
                    ))
                    client.close()
                    if self._stop_event.wait(2.0):
                        break
        finally:
            client.close()
            self._events.put(('disconnected', None))

    def _drain_events(self):
        try:
            while True:
                event, payload = self._events.get_nowait()
                if event == 'connected':
                    self._connected = True
                    self.connection_var.set(
                        f'Connected: KG-F address {payload.address}'
                    )
                    self.live_vars['serial'].set(str(payload.serial_raw))
                    self._append_log(
                        f'Connected on {self.port_var.get()} '
                        f'(model {payload.model_code}, firmware {payload.firmware_raw})'
                    )
                elif event == 'measurement':
                    self._show_measurement(payload)
                elif event == 'settings':
                    self._show_settings(payload)
                elif event == 'log':
                    self._append_log(payload)
                elif event == 'error':
                    self._connected = False
                    self.connection_var.set(f'Retrying: {payload}')
                    self._append_log(f'ERROR: {payload}')
                elif event == 'disconnected':
                    self._connected = False
                    self.connection_var.set('Disconnected')
                    self.connect_button.configure(text='Connect')
                    self.port_entry.configure(state='normal')
                    self.address_spin.configure(state='normal')
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    def _show_measurement(self, measurement):
        settings = self._last_settings
        capacity = settings.capacity_ah if settings else 0.0
        percentage = (
            max(0.0, min(100.0, 100.0 * measurement.remaining_ah / capacity))
            if capacity > 0.0 else None
        )
        # Direction 0 is discharge; match ROS BatteryState sign convention.
        current = (
            -measurement.current_magnitude_a
            if measurement.current_direction == 0
            else measurement.current_magnitude_a
        )
        self.live_vars['battery'].set(
            f'{percentage:.1f} %' if percentage is not None else '—'
        )
        self.live_vars['voltage'].set(f'{measurement.voltage_v:.2f} V')
        self.live_vars['current'].set(f'{current:+.2f} A')
        self.live_vars['power'].set(
            f'{measurement.voltage_v * current:+.1f} W'
        )
        self.live_vars['temperature'].set(
            f'{measurement.temperature_c:.1f} °C'
        )
        self.live_vars['remaining_ah'].set(
            f'{measurement.remaining_ah:.3f} Ah'
        )
        self.live_vars['energy_wh'].set(f'{measurement.energy_wh:.2f} Wh')
        hours, remainder = divmod(measurement.runtime_s, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.live_vars['runtime'].set(f'{hours:d}:{minutes:02d}:{seconds:02d}')
        self.live_vars['remaining_time'].set(
            f'{measurement.battery_life_min} min'
        )
        self.live_vars['resistance'].set(
            f'{measurement.internal_resistance_ohm * 1000.0:.2f} mΩ'
        )
        self.live_vars['output'].set(measurement.output_status_name)
        self._voltage_history.append(measurement.voltage_v)
        self._current_history.append(current)
        self._draw_graph()

    def _show_settings(self, settings):
        self._last_settings = settings
        values = settings.as_dict()
        for name in SETTING_ORDER:
            value = values[name]
            if value is None:
                text = 'Unsupported by this firmware'
                self.setting_widgets[name].configure(state='disabled')
            elif name == 'relay_normally_closed':
                text = 'Normally closed' if value else 'Normally open'
                self.setting_widgets[name].configure(state='readonly')
            elif SETTING_SPECS[name].integer:
                text = str(int(value))
                self.setting_widgets[name].configure(state='normal')
            else:
                text = f'{float(value):g}'
                self.setting_widgets[name].configure(state='normal')
            self.setting_vars[name].set(text)
        self.live_vars['capacity_ah'].set(f'{settings.capacity_ah:.1f} Ah')
        self._append_log('Settings refreshed from KG-F')

    def _draw_graph(self):
        canvas = self.graph
        canvas.delete('all')
        width = max(10, canvas.winfo_width())
        height = max(10, canvas.winfo_height())
        canvas.create_text(
            8, 8, anchor='nw', fill='#74d680', text='Voltage'
        )
        canvas.create_text(
            75, 8, anchor='nw', fill='#66c7ff', text='Current'
        )

        def draw(values, color):
            if len(values) < 2:
                return
            minimum = min(values)
            maximum = max(values)
            span = max(0.01, maximum - minimum)
            points = []
            for index, value in enumerate(values):
                x = index * (width - 10) / max(1, len(values) - 1) + 5
                y = height - 8 - ((value - minimum) / span) * (height - 32)
                points.extend((x, y))
            canvas.create_line(*points, fill=color, width=2, smooth=True)

        draw(list(self._voltage_history), '#74d680')
        draw(list(self._current_history), '#66c7ff')

    def _apply_changed_settings(self):
        if not self._connected:
            messagebox.showwarning(
                'Not connected', 'Connect to the KG-F before writing settings.'
            )
            return
        if self._last_settings is None:
            messagebox.showwarning('No settings', 'Read settings before applying changes.')
            return
        previous = self._last_settings.as_dict()
        changes = {}
        try:
            for name in SETTING_ORDER:
                if previous[name] is None:
                    continue
                text = self.setting_vars[name].get().strip()
                if name == 'relay_normally_closed':
                    value = 1 if text == 'Normally closed' else 0
                else:
                    value = float(text)
                spec = SETTING_SPECS[name]
                spec.encode(value)
                prior = int(previous[name]) if spec.integer else float(previous[name])
                if abs(float(value) - float(prior)) > 1e-9:
                    changes[name] = value
        except ValueError as error:
            messagebox.showerror('Invalid setting', str(error))
            return
        if not changes:
            messagebox.showinfo('No changes', 'All fields match the KG-F settings.')
            return
        summary = '\n'.join(
            f'{SETTING_SPECS[name].label}: {previous[name]} → {value}'
            for name, value in changes.items()
        )
        if messagebox.askyesno(
            'Apply KG-F settings?',
            f'Write these changes to the live KG-F?\n\n{summary}',
        ):
            self._commands.put(('write_settings', changes))

    def _set_remaining_percentage(self):
        value = int(round(float(self.remaining_percent_var.get())))
        if value < 0 or value > 100:
            messagebox.showerror('Invalid percentage', 'Percentage must be 0 to 100.')
            return
        self._confirm_action(
            f'Set the KG-F remaining capacity to {value}%?', 60, value
        )

    def _confirm_action(self, prompt, function, value):
        if not self._connected:
            messagebox.showwarning(
                'Not connected', 'Connect to the KG-F before sending this action.'
            )
            return
        if messagebox.askyesno('Confirm live KG-F action', prompt):
            self._commands.put(('write_raw', (function, value)))

    def _factory_reset(self):
        if not self._connected:
            messagebox.showwarning(
                'Not connected', 'Connect to the KG-F before factory reset.'
            )
            return
        typed = simpledialog.askstring(
            'Factory reset',
            'This erases KG-F settings. Type RESET to continue:',
            parent=self.root,
        )
        if typed == 'RESET':
            self._commands.put(('write_raw', (35, 1)))

    def _append_log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        self.log.configure(state='normal')
        self.log.insert('end', f'[{timestamp}] {message}\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _on_close(self):
        self._stop_event.set()
        self._commands.put(('disconnect', None))
        self.root.destroy()


def main():
    root = tk.Tk()
    JunctekMonitorApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
