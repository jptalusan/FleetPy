"""Phase 2 smoke test: run FleetPy's solver-free insertion-heuristic example.

Run from the repo root:  uv run python scripts/run_smoke_test.py
Expected: studies/example_study/results/example_pool_irsonly_sc_1/standard_eval.csv
with about 93 of 100 requests served.
"""
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT)
from run_scenarios import run_scenarios

sc_dir = os.path.join(ROOT, "studies", "example_study", "scenarios")
run_scenarios(
    os.path.join(sc_dir, "constant_config_ir.csv"),
    os.path.join(sc_dir, "example_ir_only.csv"),
    log_level="info", n_cpu_per_sim=1, n_parallel_sim=1,
)
