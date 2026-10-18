# Changes to FleetPy

This fork is FleetPy `main` at c813728 (TUM-VT/FleetPy) plus the CARTA GO AV vs human-driver study in `studies/carta_go/`.

## Core files changed

- `src/misc/init_modules.py` gains three registration lines, `HumanDriverPoolingIRS` in `get_src_fleet_control_modules()`, `ShiftChargingFS` in `get_src_dynamic_fleet_sizing_strategies()` and `PolicyCharging` in `get_src_charging_strategies()`.
- `src/fleetctrl/planning/VehiclePlan.py` has three changes (CARTA GO version 2).
  - `VehiclePlan.plan_rules`, a class attribute that defaults to None, lets a module add feasibility rules to every plan check: the lowest charge allowed on arriving at a stop (`lowest_soc`) and a check of the whole plan (`check_plan`). With None every plan behaves as before (charge at or above 0).
  - The planned charge after a charging stop is capped with `min(c_soc, 1.0)`. FleetPy had `max`, so every plan counted a vehicle as at least full after any charging stop.
  - A charging stop credits only charging still to come: from its start (waiting before it gains nothing), or from now if it has started, since the vehicle's charge already holds what it gained. FleetPy credited the whole time from arrival, so waiting counted as charging and a charge in progress was counted twice.
- `src/fleetctrl/pooling/batch/BatchAssignmentAlgorithmBase.py`: `SimulationVehicleStruct.compute_soc_charging` divides by 3600 like `SimulationVehicle`. Without it AlonsoMora counted every charge 3600 times too large, so any charge looked like a full battery.
- `src/fleetctrl/pooling/batch/AlonsoMora/V2RB.py`: `createLowerV2RB` copies a locked stop without riders (a charge or a break) as it is. It used to rebuild it as a plain stop and drop its earliest start, latest start and earliest end.
- `src/infra/ChargingInfrastructure.py` has two fixes.
  - `Depot.pick_vehicle_to_be_active` returns None when no deactivated vehicle has reached the depot yet. It used to raise `ValueError: max() arg is an empty sequence`, which crashed a run when the shift file brought vehicles back while one sent off duty was still driving to the depot.
  - `Depot.refill_charging` skips a charge that would last under one second. A vehicle that finishes a charge on a time step can sit a hair below full (0.99999), and FleetPy only skipped exactly 1.0. It then booked a fraction-of-a-second charge every step. Booking ids are built from whole seconds, so two such bookings got the same id, and the schedule lost one of them (a KeyError or an "uncancelled bookings" assertion).
- `src/simulation/Vehicles.py` has three fixes.
  - `set_initial_state` casts the node index from an initial state file to an integer. FleetPy reads each vehicle's row as one float Series, and `NetworkTTMatrix` cannot index its node list with a float (`TypeError: list indices must be integers`).
  - `_start_next_leg_stationary_object` ends a charge at its booked duration if the battery is not full by then. FleetPy set the charging time to "until full" from the charge the vehicle arrived with. A vehicle arriving emptier than its plan assumed (it drove more before charging than planned) charged longer than booked and reached later stops after their latest pickup times: 5 of about 1,000 served CARTA GO requests in the first runs, 1 to 5.4 minutes late. A vehicle may now leave a charge slightly short of full and charge again sooner.
  - `assign_vehicle_plan` accepts a locked leg in progress that the plan rebuilt with a new planned start time, through `_is_started_leg_rebuilt`. When a vehicle is driving to a pickup or charger whose window opens later, the plan rebuilds that drive as "wait first, then go", and the vehicle rejected it as a new locked leg ("Trying to assign new VRLs instead of a locked VRL"), which stopped the run. A start time stops mattering once a leg has started. The new branch only runs where FleetPy raised before (a locked leg, no force), so every run that worked before behaves the same.
- `src/fleetctrl/fleetsizing/DynamicFleetSizingBase.py` changes `activate_vehicle` to retry a failed activation after `REACTIVATE_TIME` (15 min) whether or not a depot was named, and `time_triggered_activate` to stop counting a failed activation as an activated vehicle. The old retry only ran when a depot was named, so with the first fix alone a vehicle the shift file brought back would have stayed off duty for the rest of the day.

These address two FleetPy defects that our scenarios exposed. Two other problems were our scenarios' own needs, and those live in our module `ShiftChargingFS`, outside FleetPy's core.
- **Charging lock.** A vehicle brought back on duty while charging stays until full. That is FleetPy's design, and it is now the `finish` policy.
- **Drivers sent home on the way to a charger.** FleetPy rebuilds that vehicle's locked leg slightly differently and refuses the plan. FleetPy's own example combining shifts and charging (`example_depot_charge`) is commented out in its scenario file. Our rule lets that driver finish the charge before going off duty.

## Checks of the core edits

The edits were run against FleetPy's own solver-free examples on the original commit and on this tree. The time-based and utilization-based depot examples and the charging example give the same served requests and waiting times. One difference shows up in `example_depot_time_pool_irsonly_sc_1`, where the original run gave a full vehicle a 0-second charging record at 5760 s and this tree does not. That shifts four `standard_eval.csv` values in their 16th digit (floating-point summation order).

FleetPy's module tests (all 45 scenarios, with Gurobi 13.0.3 and the compiled C++ routers) were run on the original commit and on this tree on the same machine.
- 43 scenarios give identical results on both.
- 9 of those already differ from FleetPy's stored `benchmark.csv` on the original code (`mt_broker_broker`, `mt_fc_perf`, `mt_fc_perf_dist`, `mt_repo_LH`, `mt_repo_SampleRes`, `mt_res_roll_ContBatch`, `mt_sim_env_imm_dec`, `pos_nw_partproc`, `pos_nw_withstore`), so those differences come from the Gurobi version or platform.
- `mt_dyn_fs_util` changes because of the activation fix. On the original code the activation crash happens once and `UtilizationBasedFS` swallows it in a bare `try/except` ("error in activate list"), so that vehicle never comes back. With the fix it is retried 15 minutes later. 194 users served become 197, and the mean wait drops from 145.7 s to 134.7 s.
- `mt_dyn_fs_time` changes because of the depot top-up guard (any width from 1 s up gives the same change, and switching the guard off restores the original exactly). The original books four depot charges shorter than 60 s for nearly full vehicles. Without them, 151 users served become 154 and the mean wait drops from 150.4 s to 143.8 s, while total charging time stays at 7.8 h.

The module tests were run again on 2026-10-17 after the two newer `Vehicles.py` edits (rebuilt locked leg, charge cap), with nothing else changed since the run above.
- 44 scenarios give identical results to that run.
- `mt_charging` changes because of the charge cap. 68 users served (the stored benchmark) become 72, and total vkm drops from 205.3 to 201.4. Switching the cap off for one run restores 68 exactly. The lowest charge with the cap is 0.1%, so no vehicle goes below empty.
- The rebuilt-leg branch only runs where FleetPy used to stop with "Trying to assign new VRLs instead of a locked VRL", so it cannot change a run that finished before.

## Files added

- `src/fleetctrl/charging/PolicyCharging.py`: the version 2 charging strategy (`threshold`, `idle`, `scheduled`), the charge reserve, driver lunch breaks at charging sites, and the end-of-day return to the garage. `studies/carta_go/SPEC.md` defines its rules.
- `src/fleetctrl/HumanDriverPoolingIRS.py` is the human-driver fleet control (a subclass of `PoolingInsertionHeuristicOnly`, behavior unchanged until its stubs are filled in).
- `src/fleetctrl/fleetsizing/ShiftChargingFS.py` is time-based fleet sizing with a shift-start charge policy (`finish`, `leave`, `min_soc`) and the rule that a driver whose shift ends during a charge finishes it first.
- `scripts/` holds helper scripts (`build_osm_network.py`, `match_demand.py`), a smoke test (`run_smoke_test.py`), the OSRM network builder (`build_osrm_network.py`, `osrm_client.py`), the Gurobi license check (`gurobi_license.py`) and shared helpers (`common.py`).
- The CARTA GO study lives in its own repository (jptalusan/carta-go-av-study), which uses this branch as a submodule and links itself in as `studies/carta_go/`.
- `pyproject.toml` and `uv.lock` define the uv environment, with `gurobipy` as an optional extra. `.env.example` lists the settings the scripts and the study read (OSRM server, Gurobi license file, CARTA GO request export).

## Built locally

The C++ road router (`src/routing/road/cpp_router`) and transit router (`src/routing/pt/cpp_raptor_router`) are compiled with `uv run --with setuptools python setup.py build_ext --inplace` in each folder. The compiled files are git-ignored.

## Helper scripts

- `scripts/make_grid.py` from the setup guide was removed with the earlier grid and Nashville study. `build_osm_network.py` and `match_demand.py` are kept.
- `scripts/build_osrm_network.py` lays its grid over a GeoJSON polygon (`--zone`, `--buffer`), always keeps given points such as a depot (`--keep-point`, recorded in `base/keep_points.csv`), and drops road points that OSRM can only leave or reach by a long loop (`--outlier-threshold`, `--outlier-share`).

## Problems found in version 1, fixed in version 2

- The `max(c_soc, 1.0)` in `VehiclePlan` (see above).
- A battery could go below empty on the way to the charger. In 9 of 100 CARTA GO version 1 runs a van reached the charger at -0.02% to -4.6% after an 11 to 18 km drive, with sockets free. FleetPy's plan check does count the drive to a charger, but `ChargingThresholdPublicInfrastructure` books the charge after the van's last planned stop (2 to 3 hours later with rides booked ahead) without looking at the result. `PolicyCharging` keeps a charge reserve (the drive to the nearest charger plus 5%) through `VehiclePlan.plan_rules`, so the optimizer never gives a van a ride that would strand it, and adds a charge only to a plan that passes the check.

## Other

- `.gitignore` tracks `studies/carta_go/` except its `results/` and `scenarios/generated/`, and ignores `log/` and `*.lic`.
