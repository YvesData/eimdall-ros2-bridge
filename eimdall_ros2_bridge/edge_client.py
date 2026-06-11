"""HTTP client for the Eimdall Edge local service (127.0.0.1:8787)."""
from __future__ import annotations

import json
import logging
import ssl
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class EdgeClient:
    """Thin HTTP client for the Eimdall Edge local API.

    Connects to the Edge runtime running on the robot at the configured address.
    Authentication uses a static token from a file on disk.
    """

    def __init__(
        self,
        edge_url: str = "https://127.0.0.1:8787",
        token_file: str = "/etc/eimdall/eimdall-local-service.token",
        ca_cert: Optional[str] = "/etc/eimdall/tls/edge-local-ca.crt",
        timeout_s: float = 2.0,
    ) -> None:
        self._base = edge_url.rstrip("/")
        self._timeout = timeout_s
        self._ssl_ctx = self._build_ssl(ca_cert)

        token_path = Path(token_file)
        if not token_path.exists():
            raise FileNotFoundError(f"Edge token file not found: {token_file}")
        self._token = token_path.read_text().strip()

    # ── Public API ──────────────────────────────────────────────────────────

    def ingest(
        self,
        robot_id: str,
        bridge_id: str,
        sensor_id: str,
        family: str,
        values: Dict[str, float],
        source: str = "ros2",
        ts_unix_ms: Optional[int] = None,
    ) -> bool:
        payload = {
            "robot_id": robot_id,
            "bridge_id": bridge_id,
            "source": source,
            "sensor_id": sensor_id,
            "family": family,
            "ts_unix_ms": ts_unix_ms or int(time.time() * 1000),
            "values": values,
        }
        return self._post("/v1/local/bridge/ingest", payload)

    def heartbeat(
        self,
        robot_id: str,
        bridge_id: str,
        uptime_s: int = 0,
        errors_5m: int = 0,
    ) -> bool:
        payload = {
            "robot_id": robot_id,
            "bridge_id": bridge_id,
            "ts_unix_ms": int(time.time() * 1000),
            "status": "ok",
            "uptime_s": uptime_s,
            "errors_5m": errors_5m,
        }
        return self._post("/v1/local/bridge/heartbeat", payload)

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

    # ── Internal ────────────────────────────────────────────────────────────

    def _post(self, path: str, payload: Dict[str, Any]) -> bool:
        body = json.dumps(payload).encode()
        req = urllib.request.Request(
            self._base + path,
            data=body,
            headers={
                "Content-Type": "application/json",
                "X-Eimdall-Bridge-Token": self._token,
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout, context=self._ssl_ctx) as resp:
                return 200 <= resp.status < 300
        except (urllib.error.URLError, OSError) as exc:
            logger.debug("Edge ingest failed: %s", exc)
            return False

    @staticmethod
    def _build_ssl(ca_cert: Optional[str]) -> Optional[ssl.SSLContext]:
        if not ca_cert or not Path(ca_cert).exists():
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            return ctx
        ctx = ssl.create_default_context()
        ctx.load_verify_locations(cafile=ca_cert)
        return ctx
