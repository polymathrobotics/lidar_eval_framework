# lidar_eval_orchestrator/tools/bag_runner.py

# debating if i should move this to lidar automation manager

import os
import threading
import subprocess
import time
from pathlib import Path
from typing import Optional


class LidarBagPlayer:


    def __init__(self, bag_path: str, storage_id: str = "mcap", playback_rate: float = 1.0):
        self.bag_path = bag_path
        self.storage_id = storage_id
        self.playback_rate = playback_rate

        self._mcap_files: list[dict] = []
        self._play_queue: list[str] = []
        self._idx: int = 0

        # top down folder names
        self.base_folder= "base"
        self.angle_folder = "angles"
        self.config_folder = "parameter_configs"

        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._last_exit_code: Optional[int] = None

        # Optional: small delay after starting playback so discovery can settle
        self.startup_grace_sec: float = 0.5
        self._last_start_time_monotonic: Optional[float] = None

    # ----------------------------
    # File list / indexing
    # ----------------------------
    def refresh_file_list(self) -> None:

        """(Re)scan directory for .mcap files (non-recursive) and reset index to 0."""
        if not os.path.isdir(self.bag_path):
            self._mcap_files = []
            self._play_queue = []
            self._idx = 0
            return


        """Need to edit the below """

        self._mcap_files = []
        self._play_queue = []

        self.dfs(self.bag_path)



    def _folder_priority(self, folder: Path) -> int:
        name = folder.name
        if name == self.base_folder:
            return 0
        if name == self.config_folder:
            return 1
        if name == self.angle_folder:
            return 2
        return 3

    def _find_mcap_files(self, root: Path) -> list[Path]:
        results = []
        stack = [root]
        while stack:
            node = stack.pop()
            for item in node.iterdir():
                if item.is_dir():
                    stack.append(item)
                elif item.is_file() and item.suffix == '.mcap':
                    results.append(item)
        return results

    def _collect_base(self, base_path: Path) -> None:
        for f in self._find_mcap_files(base_path):
            self._mcap_files.append({self.base_folder: f.name})
            self._play_queue.append(str(f.parent))

    def _collect_configs(self, configs_path: Path) -> None:
        for param_folder in sorted(configs_path.iterdir()):
            if not param_folder.is_dir():
                continue
            mcap_files = self._find_mcap_files(param_folder)
            if mcap_files:
                self._mcap_files.append({param_folder.name: [f.name for f in mcap_files]})
                self._play_queue.extend(str(f.parent) for f in mcap_files)

    def _collect_angles(self, angles_path: Path) -> None:
        for angle_folder in sorted(angles_path.iterdir()):
            if not angle_folder.is_dir():
                continue
            mcap_files = self._find_mcap_files(angle_folder)
            if mcap_files:
                self._mcap_files.append({angle_folder.name: [f.name for f in mcap_files]})
                self._play_queue.extend(str(f.parent) for f in mcap_files)

    def dfs(self, results_path_raw) -> None:
        results_path = Path(results_path_raw)

        top_folders = sorted(
            [f for f in results_path.iterdir() if f.is_dir()],
            key=self._folder_priority
        )

        for folder in top_folders:
            if folder.name == self.base_folder:
                self._collect_base(folder)
            elif folder.name == self.config_folder:
                self._collect_configs(folder)
            elif folder.name == self.angle_folder:
                self._collect_angles(folder)

        self._idx = 0

    def has_next(self) -> bool:
        return self._idx < len(self._play_queue)

    def next_bag_name(self) -> str | None:
        if self._idx >= len(self._play_queue):
            return None
        f = Path(self._play_queue[self._idx])
        relative = f.relative_to(Path(self.bag_path).parent)
        return str(relative).replace(os.sep, '_')

    def next_bag_path(self) -> str | None:
        if self._idx >= len(self._play_queue):
            return None
        return self._play_queue[self._idx]

    def next_bag_report_info(self) -> tuple[str, str] | None:
        """Returns (folder_path, report_stem) for the next bag to be played.
        folder_path is relative to the metrics output root (e.g. 'E1R/angles').
        """
        if not self.has_next():
            return None
        path = Path(self._play_queue[self._idx])
        lidar_type = Path(self.bag_path).name
        parts = path.parts

        if self.base_folder in parts:
            return (f'{lidar_type}/{self.base_folder}', 'report')

        if self.config_folder in parts:
            i = parts.index(self.config_folder)
            param_name = parts[i + 1]
            value_name = parts[i + 2]
            return (f'{lidar_type}/{self.config_folder}/{param_name}', f'{value_name}_report')

        if self.angle_folder in parts:
            i = parts.index(self.angle_folder)
            angle_name = parts[i + 1]
            return (f'{lidar_type}/{self.angle_folder}', f'{angle_name}_report')

        return (lidar_type, 'report')

    def next_bag_angle(self) -> int | None:
        path = self.next_bag_path()
        if path is None:
            return None
        for part in Path(path).parts:
            if part.startswith('angle='):
                try:
                    return int(part.split('=', 1)[1])
                except ValueError:
                    return None
        return None

    # ----------------------------
    # Playback process state
    # ----------------------------
    def is_playing(self) -> bool:
        """True if a ros2 bag play subprocess is currently running."""
        with self._lock:
            if self._proc is None:
                return False
            return self._proc.poll() is None  # None => still running

    def last_exit_code(self) -> Optional[int]:
        """Exit code of the most recently finished bag process, if any."""
        with self._lock:
            return self._last_exit_code

    def in_startup_grace(self) -> bool:
        """True briefly after starting a bag (useful to avoid premature 'finished' logic)."""
        with self._lock:
            if self._last_start_time_monotonic is None:
                return False
            return (time.monotonic() - self._last_start_time_monotonic) < self.startup_grace_sec

    # ----------------------------
    # Playback control
    # ----------------------------
    def stop(self) -> None:
        """Stop current playback process if running."""
        with self._lock:
            proc = self._proc

        if proc is None:
            return

        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        finally:
            with self._lock:
                self._proc = None

    def play_next_async(self) -> bool:
        """
        Start playing the next bag in a background thread.
        Returns False if no bag left. Returns True if started (or already playing).
        """
        with self._lock:
            # If already playing, do not start a second one
            if self._proc is not None and self._proc.poll() is None:
                return True

            if not self.has_next():
                return False

            mcap_file = self._play_queue[self._idx]
            self._idx += 1

            # Set grace timestamp before the thread starts so is_playing()/in_startup_grace()
            # never see a window where both return False while the bag is starting up.
            self._last_start_time_monotonic = time.monotonic()

            # Spawn background thread that launches and waits on the process
            self._thread = threading.Thread(target=self._play_mcap_proc, args=(mcap_file,), daemon=True)
            self._thread.start()
            return True

    def _play_mcap_proc(self, mcap_file: str) -> None:
        time.sleep(0.5)  # allow TF broadcast to propagate before bag starts
        cmd = [
            "ros2", "bag", "play", mcap_file,
            "--storage", self.storage_id,
            "--rate", str(self.playback_rate),
        ]

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=None,   # inherit
                stderr=None,   # inherit
            )
            with self._lock:
                self._proc = proc
                self._last_exit_code = None
                # _last_start_time_monotonic already set in play_next_async before thread launch

            rc = proc.wait()  # blocks in THIS thread only

            with self._lock:
                self._last_exit_code = rc
                # leave _proc set until next check, but it's finished now

        except Exception as e:
            print(f"Error during playback of {mcap_file}: {e}")
            with self._lock:
                self._last_exit_code = -1
                self._proc = None
                self._last_start_time_monotonic = None
