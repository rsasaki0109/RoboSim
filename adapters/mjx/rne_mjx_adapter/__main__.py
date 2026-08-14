"""Command-line entry point for ``python -m rne_mjx_adapter``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .protocol import ProtocolError, canonical_json, error_response
from .server import create_backend, serve


def main(argv: list[str] | None = None) -> int:
    """Runs the JSONL adapter with MJX-Warp selected by default."""

    parser = argparse.ArgumentParser(description="RNE MJX-Warp JSONL adapter")
    parser.add_argument("--backend", choices=("mjx_warp", "fake"), default="mjx_warp")
    parser.add_argument("--allow-test-backend", action="store_true")
    args = parser.parse_args(argv)
    adapter_root = Path(__file__).resolve().parent.parent
    try:
        backend = create_backend(args.backend, adapter_root, args.allow_test_backend)
    except ProtocolError as error:
        sys.stdout.write(canonical_json(error_response(None, error)) + "\n")
        return 2
    return serve(backend)


if __name__ == "__main__":
    raise SystemExit(main())
