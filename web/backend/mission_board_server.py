"""Background web server exposing the mission board frontend and state API."""
from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Callable, Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from web.backend.state_builder import MissionBoardStateBuilder


class MissionBoardServer:
    """Launches a FastAPI server that streams the blackboard state to the UI."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8000,
        *,
        instruction_handler: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._state_builder = MissionBoardStateBuilder()
        self._logger = logging.getLogger(__name__)
        self._instruction_handler = instruction_handler

        self._app = FastAPI(title="Mission Board", docs_url=None, redoc_url=None)
        self._configure_routes()

        self._thread: Optional[threading.Thread] = None
        self._server: Optional[uvicorn.Server] = None
        self._lock = threading.Lock()

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return

            config = uvicorn.Config(
                self._app,
                host=self._host,
                port=self._port,
                log_level="info",
                access_log=False,  # Suppress per-request access logs; the UI polls frequently.
            )
            self._server = uvicorn.Server(config)

            self._thread = threading.Thread(
                target=self._run_server,
                name="mission-board-web",
                daemon=True,
            )
            self._thread.start()
            self._logger.info("Mission board server started on %s:%s", self._host, self._port)

    def stop(self) -> None:
        with self._lock:
            if not self._server:
                return
            self._server.should_exit = True

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        with self._lock:
            self._server = None
            self._thread = None
            self._logger.info("Mission board server stopped")

    def _run_server(self) -> None:
        assert self._server is not None
        self._server.run()

    def _configure_routes(self) -> None:
        base_dir = Path(__file__).resolve().parent.parent
        static_dir = base_dir / "frontend"
        static_dir.mkdir(parents=True, exist_ok=True)

        audio_dir = base_dir.parent / "natural_language_processing" / "audio"
        audio_dir.mkdir(parents=True, exist_ok=True)

        @self._app.get("/api/state")
        async def get_state() -> dict:
            try:
                return self._state_builder.build_state()
            except Exception as exc:  # noqa: BLE001
                self._logger.exception("Failed to build mission board state")
                raise HTTPException(status_code=500, detail=str(exc))

        @self._app.post("/api/chat")
        async def post_chat(request: Request) -> dict:
            try:
                payload = await request.json()
            except Exception as exc:  # noqa: BLE001
                raise HTTPException(status_code=400, detail=f"Invalid JSON: {exc}") from exc

            message = str(payload.get("message", "")).strip()

            if not message:
                raise HTTPException(status_code=400, detail="Message is required")

            handler = self._instruction_handler
            if handler is None:
                raise HTTPException(status_code=503, detail="LLM instruction handler unavailable")

            try:
                handler(message)
            except Exception as exc:  # noqa: BLE001
                self._logger.exception("Failed to forward chat instruction")
                raise HTTPException(status_code=500, detail=str(exc))

            return {"status": "queued"}

        @self._app.get("/", response_class=HTMLResponse)
        async def read_index() -> HTMLResponse:
            index_path = static_dir / "index.html"
            print(index_path)
            if not index_path.exists():
                raise HTTPException(status_code=404, detail="Mission board UI not found")
            return HTMLResponse(index_path.read_text(encoding="utf-8"))

        self._app.mount("/static", StaticFiles(directory=static_dir), name="static")
        self._app.mount("/audio", StaticFiles(directory=audio_dir), name="tts-audio")


__all__ = ["MissionBoardServer"]
