#!/usr/bin/env python3
"""Simple operator-facing controls for saving Sahabat SLAM maps."""

import os
from pathlib import Path
import re
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String


DEFAULT_MAP_DIRECTORY = Path(
    os.environ.get(
        'SAHABAT_MAP_DIR',
        str(Path.home() / 'sahabat_ws' / 'maps'),
    )
)
VALID_MAP_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]*$')


class MappingControlPanel:
    """Present map saving in operator language rather than SLAM internals."""

    def __init__(self) -> None:
        """Create the ROS clients and graphical controls."""
        rclpy.init()
        self.node = Node('mapping_control_panel')
        self.save_client = self.node.create_client(
            SaveMap,
            '/slam_toolbox/save_map',
        )
        self.session_client = self.node.create_client(
            SerializePoseGraph,
            '/slam_toolbox/serialize_map',
        )
        self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            1,
        )

        self.root = tk.Tk()
        self.root.title('Sahabat Mapping')
        self.root.geometry('650x520')
        self.root.minsize(560, 460)

        self.map_name = tk.StringVar(value='gallery_map')
        self.directory = tk.StringVar(value=str(DEFAULT_MAP_DIRECTORY))
        self.save_session = tk.BooleanVar(value=True)
        self.map_status = tk.StringVar(value='Waiting for the live map...')
        self.save_status = tk.StringVar(value='Ready to save')
        self.saving = False
        self.pending_stem = None
        self.pending_save_session = False

        self._create_widgets()
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(self.node,),
            daemon=True,
        )
        self.spin_thread.start()

    def _create_widgets(self) -> None:
        """Lay out the operator workflow and explanations."""
        main = ttk.Frame(self.root, padding=14)
        main.pack(fill='both', expand=True)

        ttk.Label(
            main,
            text='Create and Save a Map',
            font=('TkDefaultFont', 16, 'bold'),
        ).pack(anchor='w')
        ttk.Label(
            main,
            text=(
                'Drive the robot around until the map looks complete in '
                'RViz, give it a meaningful name, then save it here.'
            ),
            wraplength=610,
            justify='left',
        ).pack(anchor='w', pady=(4, 12))

        live = ttk.LabelFrame(main, text='Live mapping status', padding=10)
        live.pack(fill='x')
        ttk.Label(live, textvariable=self.map_status).pack(anchor='w')

        destination = ttk.LabelFrame(
            main,
            text='Where this map will be saved',
            padding=10,
        )
        destination.pack(fill='x', pady=(10, 0))
        destination.columnconfigure(1, weight=1)

        ttk.Label(destination, text='Map name').grid(
            row=0,
            column=0,
            sticky='w',
            padx=(0, 10),
        )
        ttk.Entry(destination, textvariable=self.map_name).grid(
            row=0,
            column=1,
            columnspan=2,
            sticky='ew',
        )
        ttk.Label(
            destination,
            text='Use letters, numbers, hyphens, or underscores.',
        ).grid(row=1, column=1, columnspan=2, sticky='w', pady=(3, 8))

        ttk.Label(destination, text='Folder').grid(
            row=2,
            column=0,
            sticky='w',
            padx=(0, 10),
        )
        ttk.Entry(destination, textvariable=self.directory).grid(
            row=2,
            column=1,
            sticky='ew',
        )
        ttk.Button(
            destination,
            text='Choose...',
            command=self._choose_directory,
        ).grid(row=2, column=2, padx=(8, 0))

        ttk.Checkbutton(
            destination,
            text='Also save an editable mapping session (recommended)',
            variable=self.save_session,
        ).grid(row=3, column=1, columnspan=2, sticky='w', pady=(10, 0))
        ttk.Label(
            destination,
            text=(
                'The map files are used for navigation. The editable session '
                'lets a developer continue or repair this map later.'
            ),
            wraplength=500,
            justify='left',
        ).grid(row=4, column=1, columnspan=2, sticky='w', pady=(3, 0))

        save_row = ttk.Frame(main)
        save_row.pack(fill='x', pady=(12, 0))
        self.save_button = ttk.Button(
            save_row,
            text='Save Map',
            command=self._save,
        )
        self.save_button.pack(side='left')
        ttk.Label(
            save_row,
            textvariable=self.save_status,
            font=('TkDefaultFont', 10, 'bold'),
        ).pack(side='left', padx=12)

        explanation = ttk.LabelFrame(
            main,
            text='Files created',
            padding=8,
        )
        explanation.pack(fill='x', pady=(10, 0))
        ttk.Label(
            explanation,
            text=(
                '.yaml + .pgm  - the finished map used for navigation\n'
                '.posegraph + .data  - the optional editable mapping session'
            ),
            justify='left',
        ).pack(anchor='w')

        log_frame = ttk.LabelFrame(main, text='Save log', padding=6)
        log_frame.pack(fill='both', expand=True, pady=(10, 0))
        self.log_box = scrolledtext.ScrolledText(
            log_frame,
            height=7,
            state='disabled',
            wrap='word',
        )
        self.log_box.pack(fill='both', expand=True)
        self.log_box.tag_configure('success', foreground='#187a2f')
        self.log_box.tag_configure('error', foreground='#b00020')
        self.log_box.tag_configure('info', foreground='#005a9c')

    def _map_callback(self, message: OccupancyGrid) -> None:
        """Show that mapping is active and report the current map size."""
        width = message.info.width * message.info.resolution
        height = message.info.height * message.info.resolution
        text = (
            f'Map is live: {width:.1f} m x {height:.1f} m '
            f'({message.info.resolution:.2f} m/cell)'
        )
        self.root.after(0, self.map_status.set, text)

    def _choose_directory(self) -> None:
        """Let the operator choose a visible output directory."""
        selected = filedialog.askdirectory(
            title='Choose map folder',
            initialdir=self.directory.get(),
        )
        if selected:
            self.directory.set(selected)

    def _validated_stem(self) -> Path | None:
        """Validate operator input and return the extension-free path."""
        name = self.map_name.get().strip()
        if not VALID_MAP_NAME.fullmatch(name):
            messagebox.showerror(
                'Invalid map name',
                'Use letters, numbers, hyphens, and underscores only. '
                'The name cannot be empty.',
            )
            return None

        directory = Path(self.directory.get()).expanduser().resolve()
        try:
            directory.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            messagebox.showerror('Cannot create map folder', str(error))
            return None

        stem = directory / name
        existing = [
            stem.with_suffix(extension)
            for extension in ('.yaml', '.pgm', '.posegraph', '.data')
            if stem.with_suffix(extension).exists()
        ]
        if existing:
            files = '\n'.join(path.name for path in existing)
            overwrite = messagebox.askyesno(
                'Replace existing map?',
                f'These files already exist:\n\n{files}\n\nReplace them?',
            )
            if not overwrite:
                return None
        return stem

    def _save(self) -> None:
        """Save navigation files, then optionally save an editable session."""
        if self.saving:
            return
        stem = self._validated_stem()
        if stem is None:
            return
        if not self.save_client.wait_for_service(timeout_sec=1.0):
            self._failure(
                'SLAM map saving is not available. Start mapping first.'
            )
            return

        self.saving = True
        self.pending_stem = stem
        self.pending_save_session = self.save_session.get()
        self.save_button.configure(state='disabled')
        self.save_status.set('Saving navigation map...')
        self._log(f'Saving map as {stem}', 'info')

        request = SaveMap.Request()
        request.name = String(data=str(stem))
        future = self.save_client.call_async(request)
        future.add_done_callback(self._map_saved)

    def _map_saved(self, future) -> None:
        """Handle map output and continue to session output when requested."""
        try:
            response = future.result()
        except Exception as error:
            self.root.after(0, self._failure, f'Map save failed: {error}')
            return
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            self.root.after(
                0,
                self._failure,
                f'Map save failed with result code {response.result}.',
            )
            return

        self.root.after(
            0,
            self._log,
            'Navigation map saved (.yaml and .pgm).',
            'success',
        )
        if not self.pending_save_session:
            self.root.after(0, self._success)
            return
        if not self.session_client.wait_for_service(timeout_sec=1.0):
            self.root.after(
                0,
                self._failure,
                'Navigation map saved, but editable-session service is '
                'unavailable.',
            )
            return

        self.root.after(0, self.save_status.set, 'Saving editable session...')
        request = SerializePoseGraph.Request()
        request.filename = str(self.pending_stem)
        future = self.session_client.call_async(request)
        future.add_done_callback(self._session_saved)

    def _session_saved(self, future) -> None:
        """Finish the workflow after the editable session service responds."""
        try:
            response = future.result()
        except Exception as error:
            self.root.after(
                0,
                self._failure,
                f'Navigation map saved, but session save failed: {error}',
            )
            return
        if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            self.root.after(
                0,
                self._failure,
                'Navigation map saved, but session save failed with result '
                f'code {response.result}.',
            )
            return
        self.root.after(
            0,
            self._log,
            'Editable mapping session saved (.posegraph and .data).',
            'success',
        )
        self.root.after(0, self._success)

    def _success(self) -> None:
        """Restore controls and show the completed output location."""
        stem = self.pending_stem
        self.saving = False
        self.save_button.configure(state='normal')
        self.save_status.set('Saved successfully')
        if stem is not None:
            self._log(f'Finished: {stem.parent}', 'success')

    def _failure(self, message: str) -> None:
        """Restore controls and show a useful failure message."""
        self.saving = False
        self.save_button.configure(state='normal')
        self.save_status.set('Save failed')
        self._log(message, 'error')
        messagebox.showerror('Map save failed', message)

    def _log(self, message: str, tag: str = 'info') -> None:
        """Append one colored line to the save log."""
        self.log_box.configure(state='normal')
        self.log_box.insert('end', message.rstrip() + '\n', tag)
        self.log_box.see('end')
        self.log_box.configure(state='disabled')

    def _close(self) -> None:
        """Shut down the helper node and close the panel."""
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        """Run the graphical event loop."""
        self.root.mainloop()


def main() -> None:
    """Open the mapping control panel."""
    MappingControlPanel().run()


if __name__ == '__main__':
    main()
