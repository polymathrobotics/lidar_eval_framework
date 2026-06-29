# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import tkinter as tk
from tkinter import filedialog, scrolledtext, ttk

import yaml

from configure_new_run import ConfigureNewRun


class PolySetupApp:

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('PolySetup — Lidar Test Bench')
        self.root.resizable(True, True)

        self._env_cfg: dict | None = None
        self._lidar_cfg: dict | None = None
        self._env_filename = tk.StringVar(value='No file selected')
        self._lidar_filename = tk.StringVar(value='No file selected')

        self._build_ui()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(16, 12, 16, 4))
        header.grid(row=0, column=0, sticky='ew')
        ttk.Label(header, text='PolySetup', font=('TkDefaultFont', 16, 'bold')).pack(anchor='w')
        ttk.Label(header, text='Configure a new lidar test bench run.', foreground='grey').pack(anchor='w')
        ttk.Separator(self.root, orient='horizontal').grid(row=0, column=0, sticky='ews', pady=(52, 0))

        panels = ttk.Frame(self.root, padding=(16, 12))
        panels.grid(row=1, column=0, sticky='nsew')
        panels.columnconfigure(0, weight=1)
        panels.columnconfigure(1, weight=1)
        panels.rowconfigure(0, weight=1)

        self._env_panel = self._build_file_panel(
            panels,
            col=0,
            title='Environment Config',
            hint='e.g. rocinante.yaml — defines zones and world placement',
            command=self._browse_env,
            filename_var=self._env_filename,
        )

        ttk.Separator(panels, orient='vertical').grid(row=0, column=1, sticky='ns', padx=12)

        self._lidar_panel = self._build_file_panel(
            panels,
            col=2,
            title='Lidar Config',
            hint='e.g. hesai.yaml — defines lidar identity, mount pose, and topic',
            command=self._browse_lidar,
            filename_var=self._lidar_filename,
        )

        ttk.Separator(self.root, orient='horizontal').grid(row=2, column=0, sticky='ew')

        footer = ttk.Frame(self.root, padding=(16, 10))
        footer.grid(row=3, column=0, sticky='ew')
        footer.columnconfigure(0, weight=1)

        self._status_label = ttk.Label(footer, text='Waiting for: environment config and lidar config.', foreground='grey')
        self._status_label.grid(row=0, column=0, sticky='w')

        self._configure_btn = ttk.Button(
            footer,
            text='Configure Run',
            state='disabled',
            command=self._on_configure,
        )
        self._configure_btn.grid(row=0, column=1, sticky='e')

    def _build_file_panel(
        self,
        parent: ttk.Frame,
        col: int,
        title: str,
        hint: str,
        command,
        filename_var: tk.StringVar,
    ) -> scrolledtext.ScrolledText:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=col, sticky='nsew', padx=(0, 8) if col == 0 else (8, 0))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(2, weight=1)

        ttk.Label(frame, text=title, font=('TkDefaultFont', 11, 'bold')).grid(row=0, column=0, columnspan=2, sticky='w', pady=(0, 2))
        ttk.Label(frame, text=hint, foreground='grey', wraplength=320).grid(row=1, column=0, sticky='w', pady=(0, 6))
        ttk.Button(frame, text='Browse…', command=command).grid(row=1, column=1, sticky='e', pady=(0, 6))

        text = scrolledtext.ScrolledText(frame, width=44, height=22, state='disabled', wrap='none', font=('Courier', 9))
        text.grid(row=2, column=0, columnspan=2, sticky='nsew')

        ttk.Label(frame, textvariable=filename_var, foreground='grey', font=('TkDefaultFont', 8)).grid(
            row=3, column=0, columnspan=2, sticky='w', pady=(4, 0)
        )

        return text

    def _load_yaml(self, path: str) -> dict | None:
        try:
            with open(path) as f:
                return yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as e:
            self._show_error(f'Failed to load {path}:\n{e}')
            return None

    def _render_to_text(self, widget: scrolledtext.ScrolledText, cfg: dict) -> None:
        widget.configure(state='normal')
        widget.delete('1.0', 'end')
        widget.insert('end', yaml.dump(cfg, sort_keys=False, default_flow_style=False))
        widget.configure(state='disabled')

    def _browse_env(self) -> None:
        path = filedialog.askopenfilename(
            title='Select environment config',
            filetypes=[('YAML files', '*.yaml *.yml')],
        )
        if not path:
            return
        cfg = self._load_yaml(path)
        if cfg is None:
            return
        self._env_cfg = cfg
        self._env_filename.set(path)
        self._render_to_text(self._env_panel, cfg)
        self._refresh_state()

    def _browse_lidar(self) -> None:
        path = filedialog.askopenfilename(
            title='Select lidar config',
            filetypes=[('YAML files', '*.yaml *.yml')],
        )
        if not path:
            return
        cfg = self._load_yaml(path)
        if cfg is None:
            return
        self._lidar_cfg = cfg
        self._lidar_filename.set(path)
        self._render_to_text(self._lidar_panel, cfg)
        self._refresh_state()

    def _refresh_state(self) -> None:
        both = self._env_cfg is not None and self._lidar_cfg is not None
        self._configure_btn.configure(state='normal' if both else 'disabled')

        missing = []
        if self._env_cfg is None:
            missing.append('environment config')
        if self._lidar_cfg is None:
            missing.append('lidar config')

        if missing:
            self._status_label.configure(text=f'Waiting for: {" and ".join(missing)}.', foreground='grey')
        else:
            lidar_name = self._lidar_cfg.get('lidar', {}).get('folder', 'unknown')
            env_name = self._env_cfg.get('name', 'unknown')
            self._status_label.configure(
                text=f'Ready: {lidar_name} lidar in {env_name} environment.',
                foreground='green',
            )

    def _on_configure(self) -> None:
        ConfigureNewRun(lidar_config=self._lidar_cfg, environment_config=self._env_cfg).configure()

    def _show_error(self, message: str) -> None:
        win = tk.Toplevel(self.root)
        win.title('Error')
        ttk.Label(win, text=message, padding=16, wraplength=300).pack()
        ttk.Button(win, text='OK', command=win.destroy).pack(pady=(0, 12))

    def run_app(self) -> None:
        self.root.mainloop()


def main():
    app = PolySetupApp()
    app.run_app()


if __name__ == '__main__':
    main()
