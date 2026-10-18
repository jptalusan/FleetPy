import logging

import numpy as np

from src.fleetctrl.PoolingIRSOnly import PoolingInsertionHeuristicOnly
from src.misc.globals import *

LOG = logging.getLogger(__name__)

# Human-driver parameters. They are read from the scenario parameters (FleetPy only copies "op_" keys into the
# operator attributes). Every default reproduces the AV fleet exactly.
G_HD_ACCEPT_DELAY_MEAN = "hd_accept_delay_mean"  # seconds before a driver accepts an assigned trip
G_HD_ACCEPT_DELAY_DIST = "hd_accept_delay_dist"  # "fixed" or "lognormal"
G_HD_REJECT_PROB = "hd_reject_prob"  # chance a driver declines an assignment
G_HD_REPO_COMPLIANCE = "hd_repo_compliance"  # chance a driver follows a repositioning order
G_HD_WAGE_PER_HOUR = "hd_wage_per_hour"  # driver cost per on-duty hour; used in post-processing only
HD_DEFAULTS = {
    G_HD_ACCEPT_DELAY_MEAN: 0.0,
    G_HD_ACCEPT_DELAY_DIST: "fixed",
    G_HD_REJECT_PROB: 0.0,
    G_HD_REPO_COMPLIANCE: 1.0,
    G_HD_WAGE_PER_HOUR: 0.0,
}
HD_DELAY_DISTRIBUTIONS = ("fixed", "lognormal")

INPUT_PARAMETERS_HumanDriverPoolingIRS = {
    "doc": """Ride-pooling operator with human drivers. Same insertion heuristic as PoolingIRSOnly; the hd_ parameters
        describe driver behavior. With all defaults the results equal PoolingIRSOnly. Driver shifts use the built-in
        dynamic fleet sizing (op_dyn_fs_method=TimeBasedFS with op_act_fs_file).
        Implemented: parameter parsing and checks, a seeded random generator, the wage (read in post-processing).
        Stubs: acceptance delay, rejection, repositioning compliance.""",
    "inherit": "PoolingInsertionHeuristicOnly",
    "input_parameters_mandatory": [],
    "input_parameters_optional": list(HD_DEFAULTS),
    "mandatory_modules": [],
    "optional_modules": [],
}


def _parse_hd_parameters(scenario_parameters):
    """Read the hd_ parameters with their defaults and check their ranges.

    Empty cells in a scenario CSV arrive as None or NaN and mean "use the default".

    :return: dict hd_ name -> checked value
    """
    params = {}
    for key, default in HD_DEFAULTS.items():
        value = scenario_parameters.get(key)
        if value is None or (isinstance(value, float) and np.isnan(value)):
            value = default
        params[key] = value if isinstance(default, str) else float(value)
    if params[G_HD_ACCEPT_DELAY_MEAN] < 0:
        raise ValueError(f"{G_HD_ACCEPT_DELAY_MEAN} must be >= 0 seconds, got {params[G_HD_ACCEPT_DELAY_MEAN]}")
    if params[G_HD_ACCEPT_DELAY_DIST] not in HD_DELAY_DISTRIBUTIONS:
        raise ValueError(f"{G_HD_ACCEPT_DELAY_DIST} must be one of {HD_DELAY_DISTRIBUTIONS}, "
                         f"got {params[G_HD_ACCEPT_DELAY_DIST]!r}")
    for key in (G_HD_REJECT_PROB, G_HD_REPO_COMPLIANCE):
        if not 0.0 <= params[key] <= 1.0:
            raise ValueError(f"{key} is a probability and must be in [0, 1], got {params[key]}")
    if params[G_HD_WAGE_PER_HOUR] < 0:
        raise ValueError(f"{G_HD_WAGE_PER_HOUR} must be >= 0, got {params[G_HD_WAGE_PER_HOUR]}")
    return params


class HumanDriverPoolingIRS(PoolingInsertionHeuristicOnly):
    """PoolingIRSOnly with hooks for human-driver behavior.

    Behavior is identical to PoolingInsertionHeuristicOnly until the stubs below are filled in. All randomness for
    driver behavior must come from self.hd_rng, which is seeded from the scenario's random_seed, so AV and human runs
    with the same seed see the same demand and comparable draws.
    """

    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, op_charge_depot_infra=None, list_pub_charging_infra=[]):
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names, op_charge_depot_infra=op_charge_depot_infra,
                         list_pub_charging_infra=list_pub_charging_infra)
        self.hd_params = _parse_hd_parameters(scenario_parameters)
        self.hd_rng = np.random.default_rng(scenario_parameters.get(G_RANDOM_SEED))
        LOG.info(f"HumanDriverPoolingIRS parameters: {self.hd_params}")

    def user_request(self, rq, sim_time, max_wait_time=None):
        # TODO acceptance delay and rejection: after the insertion heuristic picks a vehicle, draw whether that
        #  driver rejects (hd_reject_prob) and, if so, retry without that vehicle. Draw the acceptance delay
        #  (hd_accept_delay_mean, hd_accept_delay_dist) and shift the offered pickup time by it.
        return super().user_request(rq, sim_time, max_wait_time=max_wait_time)

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign=False, assigned_charging_task=None,
                            add_arg=None):
        # TODO acceptance delay: hold the new plan until the driver accepts and release it in time_trigger.
        # TODO repositioning compliance: repositioning modules assign their plans through this method; with
        #  probability 1 - hd_repo_compliance keep the vehicle's previous plan instead.
        super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign=force_assign,
                                    assigned_charging_task=assigned_charging_task, add_arg=add_arg)

    def time_trigger(self, simulation_time):
        # TODO acceptance delay: assign held plans whose acceptance time has passed.
        super().time_trigger(simulation_time)
