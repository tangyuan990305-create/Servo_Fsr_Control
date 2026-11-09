#!/usr/bin/env python3
"""Run idf.py monitor while saving the complete output to a timestamped file."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime


def _find_idf_py() -> list[str]:
    idf_path = os.environ.get("IDF_PATH")
    if idf_path:
        candidate = Path(idf_path) / "tools" / "idf.py"
        if candidate.is_file():
            return [sys.executable, str(candidate)]

    idf_py = shutil.which("idf.py")
    if idf_py:
        return [idf_py]

    raise RuntimeError(
        "idf.py was not found. Run this script from an ESP-IDF terminal."
    )


def main() -> int:
    project_dir = Path(__file__).resolve().parent.parent
    log_dir = project_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    #2026.10.10 修改：文件名增加日期中的day字段
    filename = datetime.now().strftime("%Y-%m-%d-%H-%M-%S.txt")
    log_path = log_dir / filename

    command = _find_idf_py() + ["monitor", *sys.argv[1:]]

    print(f"Saving complete monitor output to: {log_path}")
    with log_path.open("wb") as log_file:
        process = subprocess.Popen(
            command,
            cwd=project_dir,
            stdin=sys.stdin,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
        )

        assert process.stdout is not None
        try:
            while True:
                chunk = process.stdout.read(4096)
                if not chunk:
                    break
                log_file.write(chunk)
                log_file.flush()
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
        except KeyboardInterrupt:
            process.terminate()
        finally:
            process.wait()

    print(f"Monitor log saved: {log_path}")
    return process.returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
