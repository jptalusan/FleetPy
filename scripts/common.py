"""Small helpers shared by our scripts: reading `.env` and setting up logging.

Every script in `scripts/` and `studies/av_vs_human/` runs from the repo root, so paths here are relative to it.
"""
import logging
import os
import sys
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_env(path=os.path.join(ROOT, ".env")):
    """Read KEY=VALUE lines from `.env` into os.environ.

    Variables already set in the shell win, so `OSRM_URL=... uv run ...` overrides the file.
    Blank lines and lines starting with `#` are skipped. A missing file is fine.
    """
    if not os.path.isfile(path):
        return
    with open(path) as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError(f"{path}:{line_no}: expected KEY=VALUE, got {line!r}")
            key, value = line.split("=", 1)
            os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def setup_logging(program):
    """Log to `log/<program>_<start time>.log` and to the console.

    The file level comes from the LOG_LEVEL environment variable (default DEBUG).
    The console shows INFO and above so runs stay readable.
    """
    level = os.environ.get("LOG_LEVEL", "DEBUG").upper()
    if level not in ("DEBUG", "INFO", "WARNING", "ERROR"):
        raise ValueError(f"LOG_LEVEL must be DEBUG, INFO, WARNING or ERROR, got {level!r}")
    log_dir = os.path.join(ROOT, "log")
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{program}_{datetime.now():%Y%m%d_%H%M%S}.log")

    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(level)
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(max(logging.INFO, logging.getLevelName(level)))
    logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console],
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger(program)
    log.info(f"logging to {log_file} at {level}")
    return log
