#!/usr/bin/env python3
"""Graphical health and recovery controls for the Sahabat RPLIDAR S2."""

import os
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext, ttk

import serial

from shbat_pkg.lidar_recovery import (
    cycle_dtr,
    discover_port,
    get_device_info,
    health_summary,
    recover,
    recovery_guidance,
    send_reset,
    send_stop,
    verify_stable_health,
)


DEFAULT_SERIAL_ID = 'A5069RR4'


class LidarControlPanel:
    """A motor-free control panel for LIDAR diagnosis and recovery."""

    def __init__(self) -> None:
        """Create the controls and initialize idle state."""
        self.root = tk.Tk()
        self.root.title('Sahabat RPLIDAR Control Panel')
        self.root.geometry('720x610')
        self.root.minsize(620, 520)

        self.port = tk.StringVar()
        self.baud = tk.StringVar(value='1000000')
        self.attempts = tk.IntVar(value=3)
        self.use_dtr = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='DEVICE HEALTH: Not checked')
        self.scan_status = tk.StringVar(value='Scan stopped')
        self.last_health_ok = False
        self.busy = False
        self.scan_process = None
        self.action_buttons = []

        self._create_widgets()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self._detect_port()

    def _create_widgets(self) -> None:
        """Lay out the settings, actions, status, and log."""
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill='both', expand=True)

        settings = ttk.LabelFrame(main, text='Connection', padding=10)
        settings.pack(fill='x')
        settings.columnconfigure(1, weight=1)

        ttk.Label(settings, text='Serial port').grid(
            row=0, column=0, sticky='w', padx=(0, 8)
        )
        ttk.Entry(settings, textvariable=self.port).grid(
            row=0, column=1, sticky='ew'
        )
        ttk.Button(
            settings, text='Detect', command=self._detect_port
        ).grid(row=0, column=2, padx=(8, 0))

        ttk.Label(settings, text='Baud').grid(
            row=1, column=0, sticky='w', padx=(0, 8), pady=(8, 0)
        )
        ttk.Combobox(
            settings,
            textvariable=self.baud,
            values=('1000000', '115200', '256000'),
            width=14,
        ).grid(row=1, column=1, sticky='w', pady=(8, 0))
        ttk.Label(settings, text='S2 default: 1000000').grid(
            row=1, column=2, sticky='w', padx=(8, 0), pady=(8, 0)
        )

        actions = ttk.LabelFrame(main, text='Health and recovery', padding=10)
        actions.pack(fill='x', pady=(10, 0))

        for column, (label, action) in enumerate((
            ('Check Health', 'check'),
            ('Reset + Verify', 'reset'),
            ('Retry Reset', 'recover'),
        )):
            button = ttk.Button(
                actions,
                text=label,
                command=lambda selected=action: self._start_action(selected),
            )
            button.grid(row=0, column=column, padx=4, sticky='ew')
            self.action_buttons.append(button)
            actions.columnconfigure(column, weight=1)

        ttk.Label(
            actions,
            text=(
                'Reset commands the S2 firmware; Retry Reset repeats it. '
                'Neither button disconnects USB power.'
            ),
            wraplength=650,
        ).grid(row=2, column=0, columnspan=3, sticky='w', pady=(10, 0))

        ttk.Label(actions, text='Recovery attempts').grid(
            row=1, column=0, sticky='e', pady=(10, 0)
        )
        ttk.Spinbox(
            actions,
            from_=1,
            to=10,
            textvariable=self.attempts,
            width=5,
        ).grid(row=1, column=1, sticky='w', padx=8, pady=(10, 0))
        ttk.Checkbutton(
            actions,
            text='Cycle DTR after reset',
            variable=self.use_dtr,
        ).grid(row=1, column=2, sticky='w', pady=(10, 0))

        state = ttk.LabelFrame(main, text='Device state', padding=10)
        state.pack(fill='x', pady=(10, 0))
        self.status_label = ttk.Label(
            state, textvariable=self.status, font=('TkDefaultFont', 12, 'bold')
        )
        self.status_label.pack(side='left')

        scan = ttk.Frame(state)
        scan.pack(side='right')
        ttk.Label(
            scan, textvariable=self.scan_status
        ).pack(side='left', padx=8)
        self.start_scan_button = ttk.Button(
            scan,
            text='Start Scan',
            command=self._start_scan,
            state='disabled',
        )
        self.start_scan_button.pack(side='left', padx=3)
        self.stop_scan_button = ttk.Button(
            scan,
            text='Stop Scan',
            command=self._stop_scan,
            state='disabled',
        )
        self.stop_scan_button.pack(side='left', padx=3)

        warning = (
            'Protection Stop 0x0004: check for 4.9-5.2 V at the LIDAR '
            'connector under load and at least 1.5 A startup capacity. '
            'DTR is not a physical USB power cycle.'
        )
        ttk.Label(
            main,
            text=warning,
            wraplength=680,
            justify='left',
        ).pack(fill='x', pady=(10, 0))

        log_frame = ttk.LabelFrame(main, text='Activity log', padding=6)
        log_frame.pack(fill='both', expand=True, pady=(10, 0))
        self.log_box = scrolledtext.ScrolledText(
            log_frame, height=14, state='disabled', wrap='word'
        )
        self.log_box.pack(fill='both', expand=True)
        self.log_box.tag_configure('device', foreground='#005a9c')
        self.log_box.tag_configure('action', foreground='#6b238e')
        self.log_box.tag_configure('guidance', foreground='#9a5700')
        self.log_box.tag_configure('error', foreground='#b00020')
        self.log_box.tag_configure('ros', foreground='#555555')
        self.log_box.tag_configure('connection', foreground='#176b36')
        self.log_box.tag_configure('helper', foreground='#222222')
        ttk.Button(main, text='Clear Log', command=self._clear_log).pack(
            anchor='e', pady=(6, 0)
        )

    def _log(self, message: str) -> None:
        """Append a message safely from either the UI or worker thread."""
        if threading.current_thread() is not threading.main_thread():
            self.root.after(0, self._log, message)
            return
        tag = None
        for prefix, candidate in (
            ('[DEVICE]', 'device'),
            ('[ACTION]', 'action'),
            ('[GUIDANCE]', 'guidance'),
            ('[ERROR]', 'error'),
            ('[ROS]', 'ros'),
            ('[CONNECTION]', 'connection'),
            ('[HELPER]', 'helper'),
        ):
            if message.startswith(prefix):
                tag = candidate
                break
        self.log_box.configure(state='normal')
        line = message.rstrip() + '\n'
        if tag is None:
            self.log_box.insert('end', line)
        else:
            self.log_box.insert('end', line, tag)
        self.log_box.see('end')
        self.log_box.configure(state='disabled')

    def _clear_log(self) -> None:
        """Clear the activity log."""
        self.log_box.configure(state='normal')
        self.log_box.delete('1.0', 'end')
        self.log_box.configure(state='disabled')

    def _detect_port(self) -> None:
        """Resolve the configured LIDAR USB serial to a stable path."""
        try:
            detected = discover_port(DEFAULT_SERIAL_ID)
            self.port.set(detected)
            self._log(f'[CONNECTION] Detected LIDAR: {detected}')
        except RuntimeError as error:
            self._log(f'[ERROR] Detection failed: {error}')

    def _set_busy(self, busy: bool) -> None:
        """Disable conflicting controls while serial work is active."""
        self.busy = busy
        state = 'disabled' if busy else 'normal'
        for button in self.action_buttons:
            button.configure(state=state)
        if not busy and self.last_health_ok and self.scan_process is None:
            self.start_scan_button.configure(state='normal')

    def _start_action(self, action: str) -> None:
        """Run one serial action in a worker thread."""
        if self.busy or self.scan_process is not None:
            self._log(
                '[HELPER] Stop the scan before using serial recovery controls.'
            )
            return
        try:
            attempts = int(self.attempts.get())
            baud = int(self.baud.get())
            if attempts < 1:
                raise ValueError
        except (ValueError, tk.TclError):
            self._log(
                '[ERROR] Baud and recovery attempts must be valid numbers.'
            )
            return

        port = self.port.get().strip()
        if not port:
            self._detect_port()
            port = self.port.get().strip()
        if not port:
            return

        self._set_busy(True)
        self.status.set(f'ACTION: Running {action}...')
        self.start_scan_button.configure(state='disabled')
        thread = threading.Thread(
            target=self._run_action,
            args=(action, port, baud, attempts, self.use_dtr.get()),
            daemon=True,
        )
        thread.start()

    def _run_action(
        self,
        action: str,
        port: str,
        baud: int,
        attempts: int,
        use_dtr: bool,
    ) -> None:
        """Perform a serial operation and report it back to the GUI."""
        try:
            self._log(f'[CONNECTION] Opening {port} at {baud} baud')
            with serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1.0,
                write_timeout=1.0,
                exclusive=True,
            ) as connection:
                self._log(
                    f'[DEVICE] Identity: {get_device_info(connection)}'
                )
                if action == 'check':
                    health = verify_stable_health(
                        connection,
                        log=self._log,
                    )
                elif action == 'reset':
                    self._log('[ACTION] Sending STOP and RESET')
                    send_stop(connection)
                    send_reset(connection, 2.0)
                    if use_dtr:
                        self._log('[ACTION] Cycling DTR')
                        cycle_dtr(connection, 1.0, 2.0)
                    self._log(
                        '[HELPER] Reset finished; verifying that health '
                        'remains stable.'
                    )
                    health = verify_stable_health(
                        connection,
                        log=self._log,
                    )
                else:
                    health = recover(
                        connection,
                        attempts=attempts,
                        settle_seconds=2.0,
                        use_dtr_cycle=use_dtr,
                        dtr_off_seconds=1.0,
                        log=self._log,
                    )

            self._log(
                '[DEVICE] '
                + health_summary(health, prefix='Verified health')
            )
            for line in recovery_guidance(health):
                self._log(f'[GUIDANCE] {line}')
            self.root.after(0, self._show_health, health.status, health.name)
        except (
            OSError,
            serial.SerialException,
            TimeoutError,
            RuntimeError,
        ) as error:
            self._log(f'[ERROR] Operation failed: {error}')
            self.root.after(0, self._show_error, str(error))
        finally:
            self.root.after(0, self._set_busy, False)

    def _show_health(self, status: int, name: str) -> None:
        """Update health state and scan availability."""
        self.last_health_ok = status == 0
        self.status.set(f'DEVICE HEALTH: {name} ({status})')
        color = '#187a2f' if status == 0 else '#a51d1d'
        self.status_label.configure(foreground=color)
        if self.last_health_ok and not self.busy:
            self.start_scan_button.configure(state='normal')
        else:
            self.start_scan_button.configure(state='disabled')

    def _show_error(self, error: str) -> None:
        """Display a serial or process error."""
        self.last_health_ok = False
        self.status.set(f'HELPER ERROR: {error}')
        self.status_label.configure(foreground='#a51d1d')
        self.start_scan_button.configure(state='disabled')

    def _start_scan(self) -> None:
        """Launch the motor-free ROS LIDAR diagnostic after healthy status."""
        if not self.last_health_ok or self.scan_process is not None:
            return
        command = [
            'ros2',
            'launch',
            'shbat_pkg',
            'lidar_check.launch.py',
            f'lidar_port:={self.port.get().strip()}',
            f'serial_baudrate:={self.baud.get()}',
        ]
        try:
            self.scan_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except OSError as error:
            self._show_error(str(error))
            return

        self.scan_status.set('Scan running')
        self.start_scan_button.configure(state='disabled')
        self.stop_scan_button.configure(state='normal')
        self._log('[ACTION] Started motor-free LIDAR diagnostic launch.')
        threading.Thread(target=self._watch_scan, daemon=True).start()

    def _watch_scan(self) -> None:
        """Copy ROS launch output to the log and notice process exit."""
        process = self.scan_process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self._log(f'[ROS] {line.rstrip()}')
        return_code = process.wait()
        self.root.after(0, self._scan_finished, process, return_code)

    def _scan_finished(
        self,
        process: subprocess.Popen,
        return_code: int,
    ) -> None:
        """Restore scan controls when the launch process exits."""
        if self.scan_process is not process:
            return
        self.scan_process = None
        self.scan_status.set(f'Scan stopped ({return_code})')
        self.stop_scan_button.configure(state='disabled')
        if self.last_health_ok and not self.busy:
            self.start_scan_button.configure(state='normal')
        self._log(
            '[ROS] LIDAR diagnostic launch exited with '
            f'code {return_code}.'
        )

    def _stop_scan(self) -> None:
        """Stop the complete ROS diagnostic launch process group."""
        process = self.scan_process
        if process is None:
            return
        self._log('[ACTION] Stopping LIDAR diagnostic launch...')
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        self.stop_scan_button.configure(state='disabled')

    def _close(self) -> None:
        """Stop a child launch before closing the window."""
        self._stop_scan()
        self.root.destroy()

    def run(self) -> None:
        """Run the Tk event loop."""
        self.root.mainloop()


def main() -> None:
    """Open the RPLIDAR control panel."""
    LidarControlPanel().run()


if __name__ == '__main__':
    main()
