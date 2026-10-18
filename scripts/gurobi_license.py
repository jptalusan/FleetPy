"""Check the Gurobi license: which file is used, whether it works, and how many days are left.

Run from the repo root:  uv run --extra gurobi python scripts/gurobi_license.py [--warn-days 14]
The license file is the one GRB_LICENSE_FILE names in .env (gurobi.lic in the repo root, git-ignored).

Exit code 0 when the license works and has more than --warn-days left, 1 when it is close to expiry, 2 when it does
not work (expired, missing or rejected). Secrets in the license file are never printed.

Academic WLS licenses last 90 days. Gurobi has no API to extend them: click Extend next to the expiration date at
https://license.gurobi.com/manager/licenses from a network address of your academic institution (campus or VPN).
The license id and credentials stay the same, so the file does not need replacing; an extension can
take 15 to 30 minutes to reach the license servers.

Gurobi reports no expiry for WLS licenses (they are checked online at every start), so the date shown on the portal
goes in GUROBI_LICENSE_EXPIRES=YYYY-MM-DD in .env; update it after each Extend.
"""
import argparse
import datetime as dt
import os
import sys

import gurobipy as gp

from common import load_env

NO_EXPIRY = 99999999  # what Gurobi reports for licenses checked online (WLS)


def license_file():
    """The license file Gurobi reads: GRB_LICENSE_FILE (from the shell or .env), otherwise ~/gurobi.lic."""
    return os.environ.get("GRB_LICENSE_FILE") or os.path.expanduser("~/gurobi.lic")


def license_id(path):
    """LICENSEID from the file, the only field printed (the access id and secret stay private)."""
    with open(path) as f:
        for line in f:
            if line.strip().startswith("LICENSEID="):
                return line.split("=", 1)[1].strip()
    return "unknown"


def expiry_date():
    """Start a Gurobi environment and return the license's expiry date (raises GurobiError if the license fails).

    For a WLS license the date comes from GUROBI_LICENSE_EXPIRES in .env; None if that is not set.
    """
    env = gp.Env(params={"OutputFlag": 0})
    try:
        model = gp.Model(env=env)
        reported = model.LicenseExpiration
        model.dispose()
    finally:
        env.dispose()
    if reported != NO_EXPIRY:
        return dt.datetime.strptime(str(reported), "%Y%m%d").date()
    configured = os.environ.get("GUROBI_LICENSE_EXPIRES")
    if not configured:
        return None
    try:
        return dt.date.fromisoformat(configured)
    except ValueError:
        raise ValueError(f"GUROBI_LICENSE_EXPIRES in .env must be YYYY-MM-DD, got {configured!r}") from None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--warn-days", type=int, default=14, help="exit 1 when fewer days than this are left")
    args = ap.parse_args()
    load_env()  # GRB_LICENSE_FILE and GUROBI_LICENSE_EXPIRES, before Gurobi starts

    path = license_file()
    if not os.path.isfile(path):
        print(f"no license file at {path}")
        return 2
    print(f"license file {path}, license id {license_id(path)}")
    try:
        expiry = expiry_date()
    except gp.GurobiError as err:
        print(f"license does not work: {err}")
        return 2
    if expiry is None:
        print("license works; it is a WLS license, so set GUROBI_LICENSE_EXPIRES=YYYY-MM-DD in .env from the portal")
        return 1
    left = (expiry - dt.date.today()).days
    print(f"license works, expires {expiry} ({left} days left)")
    if left < args.warn_days:
        print(f"fewer than {args.warn_days} days left: extend it at https://license.gurobi.com/manager/licenses")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
