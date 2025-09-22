"""FastAPI application exposing the TurtleBot control panel endpoints."""
from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, Dict, List

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles

from code.web.backend import AgentBackend


def create_control_panel_app(backend: AgentBackend, assets_dir: Path) -> FastAPI:
    if not assets_dir.exists():
        raise FileNotFoundError(f"Static assets directory missing: {assets_dir}")

    index_path = assets_dir / "index.html"
    if not index_path.exists():
        raise FileNotFoundError(f"Missing index.html in {assets_dir}")

    app = FastAPI(title="TurtleBot Control Panel")
    app.mount("/assets", StaticFiles(directory=assets_dir, html=False), name="assets")

    @app.on_event("shutdown")
    async def _shutdown() -> None:
        await asyncio.to_thread(backend.shutdown)

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return index_path.read_text(encoding="utf-8")

    @app.get("/api/tools")
    async def tools() -> Dict[str, List[str]]:
        return {"tools": backend.tools}

    @app.get("/api/snapshot")
    async def snapshot() -> Dict[str, Any]:
        return await backend.get_snapshot()

    @app.get("/api/map")
    async def map_view() -> Dict[str, Any]:
        return await backend.get_map_view()

    @app.post("/api/clear-map")
    async def clear_map() -> Dict[str, str]:
        await backend.clear_map()
        return {"status": "cleared"}

    @app.get("/api/camera/latest")
    async def camera_frame() -> Response:
        frame = await backend.get_camera_frame()
        if frame is None:
            return Response(status_code=204)
        headers = {
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
        }
        return Response(content=frame, media_type="image/jpeg", headers=headers)

    @app.post("/api/chat")
    async def chat(payload: Dict[str, Any]) -> Dict[str, Any]:
        message = (payload.get("message") or "").strip()
        if not message:
            raise HTTPException(status_code=400, detail="Message cannot be empty.")
        try:
            result = await backend.invoke_agent(message)
            return result
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    return app


__all__ = ["create_control_panel_app"]
