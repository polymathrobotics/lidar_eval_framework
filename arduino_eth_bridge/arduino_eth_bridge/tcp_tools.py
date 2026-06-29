import socket
import threading


class TcpClient:
    def __init__(self, ip: str, port: int, timeout_sec: float = 2.0):
        self._ip = ip
        self._port = port
        self._timeout_sec = timeout_sec

        self._sock = None
        self._lock = threading.Lock()

    def send_command(self, value):
        command = f"{int(value)}\n".encode("utf-8")

        with self._lock:
            try:
                self._connect()
                self._sock.sendall(command)
            except OSError:
                self._close()
                self._connect()
                self._sock.sendall(command)

    def _connect(self):
        if self._sock is not None:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self._timeout_sec)
        sock.connect((self._ip, self._port))
        self._sock = sock

    def close(self):
        with self._lock:
            self._close()

    def _close(self):
        if self._sock is None:
            return

        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

        try:
            self._sock.close()
        except OSError:
            pass

        self._sock = None
