"""Entry point for the TurtleBot web control panel."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import uvicorn

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from code.web import AgentBackend, create_control_panel_app


def main() -> None:
    parser = argparse.ArgumentParser(description="Launch the TurtleBot web control panel.")
    parser.add_argument("--model", default="gemini-2.5-flash", help="Google Gemini model identifier.")
    parser.add_argument("--temperature", type=float, default=0.1, help="Sampling temperature for the LLM.")
    parser.add_argument(
        "--system-prompt-file",
        help="Optional path to override the default system prompt instructions.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host interface for the web server.")
    parser.add_argument("--port", type=int, default=8000, help="Port for the web server.")
    parser.add_argument(
        "--access-log",
        action="store_true",
        help="Enable HTTP access logging (disabled by default).",
    )
    args = parser.parse_args()

    system_prompt_path = Path(args.system_prompt_file).resolve() if args.system_prompt_file else None
    if system_prompt_path and not system_prompt_path.is_file():
        raise FileNotFoundError(f"System prompt file not found: {system_prompt_path}")

    backend = AgentBackend(
        model=args.model,
        temperature=args.temperature,
        system_prompt_path=system_prompt_path,
    )

    assets_dir = Path(__file__).resolve().parents[1] / "web" / "assets"
    app = create_control_panel_app(backend, assets_dir)

    config = uvicorn.Config(
        app,
        host=args.host,
        port=args.port,
        log_level="info",
        access_log=args.access_log,
    )
    server = uvicorn.Server(config)

    try:
        server.run()
    finally:
        backend.shutdown()


if __name__ == "__main__":
    main()
