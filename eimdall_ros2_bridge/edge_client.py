"""HTTP client for the Eimdall Edge local service."""
import json
import ssl
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional


class EdgeClient:
    def __init__(
        self,
        edge_url: str = "http://127.0.0.1:8787",
        token_file: str = "/etc/eimdall/eimdall-local-service.token",
        ca_cert: Optional[str] = None,
        timeout_s: float = 2.0,
    ) -> None:
        self._base = edge_url.rstrip("/")
        self._timeout = timeout_s
        self._ssl_ctx = self._build_ssl(ca_cert)

        token_path = Path(token_file)
        if not token_path.exists():
            raise FileNotFoundError(f"Edge token file not found: {token_file}")
        self._token = token_path.read_text().strip()

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
