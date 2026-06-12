"""HTTP client for the Eimdall Edge local service."""
import json
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional

_LOOPBACK_HOSTS = {"localhost", "127.0.0.1", "::1"}


def _validate_edge_url(url: str) -> str:
    """Reject non-loopback URLs to prevent SSRF and token exfiltration."""
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme not in ("http", "https"):
        raise ValueError(f"edge_url scheme must be http or https, got '{parsed.scheme}'")
    host = (parsed.hostname or "").lower()
    if host not in _LOOPBACK_HOSTS:
        raise ValueError(
            f"edge_url host '{host}' is not a loopback address. "
            "The Edge service runs locally — remote URLs are rejected."
        )
    return url.rstrip("/")


class EdgeClient:
    def __init__(
        self,
        edge_url: str = "http://127.0.0.1:8787",
        token_file: str = "/etc/eimdall/eimdall-local-service.token",
        ca_cert: Optional[str] = None,
        timeout_s: float = 2.0,
    ) -> None:
        self._base = _validate_edge_url(edge_url)
        self._timeout = timeout_s
        self._ssl_ctx = self._build_ssl(ca_cert)

        token_path = Path(token_file)
        if not token_path.exists():
            raise FileNotFoundError(f"Edge token file not found: {token_file}")
        self._token = token_path.read_text().strip()
        if not self._token:
            raise ValueError(f"Edge token file is empty: {token_file}")

    def ingest(
        self,
        robot_id: str,
        bridge_id: str,
        sensor_id: str,
        family: str,
        values: Dict[str, float],
        ts_unix_ms: Optional[int] = None,
    ) -> bool:
        return self._post("/v1/local/bridge/ingest", {
            "robot_id": robot_id,
            "bridge_id": bridge_id,
            "source": "ros2",
            "sensor_id": sensor_id,
            "family": family,
            "ts_unix_ms": ts_unix_ms or int(time.time() * 1000),
            "values": values,
        })

    def heartbeat(self, robot_id: str, bridge_id: str, uptime_s: int, errors_5m: int) -> bool:
        return self._post("/v1/local/bridge/heartbeat", {
            "robot_id": robot_id,
            "bridge_id": bridge_id,
            "ts_unix_ms": int(time.time() * 1000),
            "status": "ok",
            "uptime_s": uptime_s,
            "errors_5m": errors_5m,
        })

    def ping(self) -> bool:
        try:
            req = urllib.request.Request(
                self._base + "/v1/local/ping",
                headers={"X-Eimdall-Bridge-Token": self._token},
            )
            with urllib.request.urlopen(req, timeout=self._timeout, context=self._ssl_ctx):
                return True
        except Exception:
            return False

    def _post(self, path: str, payload: Dict[str, Any]) -> bool:
        req = urllib.request.Request(
            self._base + path,
            data=json.dumps(payload).encode(),
            headers={
                "Content-Type": "application/json",
                "X-Eimdall-Bridge-Token": self._token,
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout, context=self._ssl_ctx) as resp:
                return 200 <= resp.status < 300
        except (urllib.error.URLError, OSError):
            return False

    @staticmethod
    def _build_ssl(ca_cert: Optional[str]) -> Optional[ssl.SSLContext]:
        if not ca_cert:
            return None
        ca_path = Path(ca_cert)
        if not ca_path.exists():
            raise FileNotFoundError(
                f"CA cert '{ca_cert}' not found — refusing to disable TLS verification."
            )
        ctx = ssl.create_default_context()
        ctx.load_verify_locations(cafile=ca_cert)
        return ctx
