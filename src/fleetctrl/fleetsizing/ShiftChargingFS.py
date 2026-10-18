from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np

from src.fleetctrl.fleetsizing.TimeBasedFS import TimeBasedFS
from src.fleetctrl.planning.VehiclePlan import VehiclePlan
from src.misc.globals import *

if TYPE_CHECKING:
    from src.fleetctrl.FleetControlBase import FleetControlBase
    from src.infra.ChargingInfrastructure import Depot
    from src.simulation.Vehicles import SimulationVehicle

LOG = logging.getLogger(__name__)

# What a vehicle that is charging (or booked to charge) at the depot does when its shift starts.
G_OP_DYFS_CHARGE_POLICY = "op_dyfs_charge_policy"
G_OP_DYFS_LEAVE_MIN_SOC = "op_dyfs_leave_min_soc"
CHARGE_POLICIES = ("finish", "leave", "min_soc")
RECHECK_S = 60  # how soon a min_soc activation looks again when the vehicle is not charging yet (waiting for a socket)

INPUT_PARAMETERS_ShiftChargingFS = {
    "doc": """TimeBasedFS with a choice of what a charging vehicle does when its shift starts.
        finish: stay until the battery is full (TimeBasedFS behavior).
        leave: stop charging and leave with the current state of charge.
        min_soc: leave at once if the state of charge is at least op_dyfs_leave_min_soc, otherwise charge to that
        level first. FleetPy activates the parked vehicle with the highest state of charge.
        A driver whose shift ends while driving to or at a charger finishes that charge and then goes off duty. If
        the fleet grows again first, that driver stays on duty instead of a parked vehicle being activated.""",
    "inherit": "TimeBasedFS",
    "input_parameters_mandatory": [G_OP_ACT_FLEET_SIZE],
    "input_parameters_optional": [G_OP_DYFS_CHARGE_POLICY, G_OP_DYFS_LEAVE_MIN_SOC],
    "mandatory_modules": [],
    "optional_modules": [],
}


def _parse_policy(operator_attributes):
    """Read and check the charge policy and its minimum state of charge.

    :return: (policy, min_soc); min_soc is None unless the policy is min_soc
    """
    policy = operator_attributes.get(G_OP_DYFS_CHARGE_POLICY) or "finish"
    if policy not in CHARGE_POLICIES:
        raise ValueError(f"{G_OP_DYFS_CHARGE_POLICY} must be one of {CHARGE_POLICIES}, got {policy!r}")
    if policy != "min_soc":
        return policy, None
    min_soc = operator_attributes.get(G_OP_DYFS_LEAVE_MIN_SOC)
    if min_soc is None or (isinstance(min_soc, float) and np.isnan(min_soc)):
        raise ValueError(f"{G_OP_DYFS_CHARGE_POLICY}=min_soc needs {G_OP_DYFS_LEAVE_MIN_SOC}")
    min_soc = float(min_soc)
    if not 0.0 < min_soc <= 1.0:
        raise ValueError(f"{G_OP_DYFS_LEAVE_MIN_SOC} must be in (0, 1], got {min_soc}")
    return policy, min_soc


class ShiftChargingFS(TimeBasedFS):
    """Time-based fleet sizing (driver shifts) where the charge policy decides when a charging vehicle leaves."""

    def __init__(self, fleetctrl: FleetControlBase, operator_attributes: dict, solver: str = "Gurobi"):
        super().__init__(fleetctrl, operator_attributes, solver=solver)
        self.charge_policy, self.leave_min_soc = _parse_policy(operator_attributes)
        self._off_after_charge = []  # vids whose shift ended during a charge; they go off duty once it is done
        LOG.info(f"ShiftChargingFS: charge policy {self.charge_policy}, min soc {self.leave_min_soc}")

    def check_and_change_fleet_size(self, sim_time):
        """TimeBasedFS's shift changes, then send off duty the drivers whose charge has finished since their shift
        ended."""
        change = super().check_and_change_fleet_size(sim_time)
        for vid in [v for v in self._off_after_charge if not self._has_charging_task(self.fleetctrl.sim_vehicles[v])]:
            self._off_after_charge.remove(vid)
            depot, _ = super().deactivate_vehicle(self.fleetctrl.sim_vehicles[vid], sim_time)
            if depot is None:
                LOG.warning(f"vid {vid} finished charging after its shift but no depot has a free parking spot")
        return change

    def deactivate_vehicle(self, veh_obj: SimulationVehicle, sim_time: int, depot: Depot = None):
        """Send a vehicle off duty; a vehicle driving to or at a charger finishes that charge first.

        FleetPy picks charging vehicles only after idle and plain busy ones, so this covers the case where no other
        driver was free. The deactivation counts now (the depot that will take the vehicle is returned with no plan
        stop) and happens in check_and_change_fleet_size once the charge is done.
        """
        if veh_obj.vid in self._off_after_charge:
            return None, None  # already going off duty after its charge; FleetPy then picks another vehicle
        if not self._has_charging_task(veh_obj):
            return super().deactivate_vehicle(veh_obj, sim_time, depot)
        target = depot or self.op_charge_depot_infra.find_nearest_free_depot(veh_obj.pos, veh_obj.op_id)
        if target is None:
            return None, None
        self._off_after_charge.append(veh_obj.vid)
        LOG.info(f"vid {veh_obj.vid} goes off duty after its charge (plan time: {sim_time})")
        return target, None

    def activate_vehicle(self, sim_time: int, depot: Depot = None) -> SimulationVehicle:
        """Activate the parked vehicle with the highest state of charge, applying the charge policy first.

        A driver waiting to go off duty after a charge stays on duty instead, which needs no parked vehicle.
        Vehicles without a charging task, and the finish policy, use TimeBasedFS's activation unchanged.
        """
        if self._off_after_charge:
            vid = self._off_after_charge.pop(0)
            LOG.info(f"vid {vid} stays on duty: the fleet grows again before its charge ended (plan time: {sim_time})")
            return self.fleetctrl.sim_vehicles[vid]
        if self.charge_policy == "finish":
            return super().activate_vehicle(sim_time, depot)
        chosen_depot = depot if depot is not None else self._depot_with_most_parked()
        veh_obj = chosen_depot.pick_vehicle_to_be_active() if chosen_depot is not None else None
        if veh_obj is None or not self._has_charging_task(veh_obj):
            return super().activate_vehicle(sim_time, depot)

        if self.charge_policy == "min_soc" and veh_obj.soc < self.leave_min_soc:
            wait = self._time_to_min_soc(veh_obj, sim_time)
            LOG.debug(f"vid {veh_obj.vid} at soc {veh_obj.soc:.3f} charges {wait:.0f} s more before its shift")
            self.add_time_triggered_activate(sim_time + wait, 1, depot=depot)
            return None

        self._stop_charging(veh_obj, sim_time)
        chosen_depot.schedule_active(veh_obj)
        LOG.info(f"Activating vehicle {veh_obj} from depot {chosen_depot} after stopping its charge (plan time: {sim_time})")
        return veh_obj

    def _depot_with_most_parked(self):
        """The depot TimeBasedFS activates from when none is named: the one with the most parked vehicles."""
        depots = [d for d in self.op_charge_depot_infra.depot_by_id.values() if d.parking_vehicles > 0]
        return max(depots, key=lambda d: d.parking_vehicles) if depots else None

    def _has_charging_task(self, veh_obj):
        """True if the vehicle is charging or holds a booked charge in its plan."""
        if veh_obj.status == VRL_STATES.CHARGING:
            return True
        return any(ps.get_charging_task_id() is not None for ps in self.fleetctrl.veh_plans[veh_obj.vid].list_plan_stops)

    def _time_to_min_soc(self, veh_obj, sim_time):
        """Seconds until the vehicle reaches leave_min_soc at its current charging power (RECHECK_S if not charging)."""
        if veh_obj.status != VRL_STATES.CHARGING or not veh_obj.assigned_route[0].power:
            return RECHECK_S
        power = veh_obj.assigned_route[0].power
        return max(1, int(np.ceil(veh_obj.get_charging_duration(power, veh_obj.soc, self.leave_min_soc))))

    def _stop_charging(self, veh_obj, sim_time):
        """End a running charge and drop a booked one, leaving the vehicle idle at the depot.

        A running charge is ended through the vehicle (which detaches the socket once and records the leg), then the
        fleet control is told the leg finished, which also clears its booking. The empty plan assigned afterwards ends
        the inactive leg and makes FleetControlBase cancel a booking that had not started yet.
        """
        if veh_obj.status == VRL_STATES.CHARGING:
            _, charging_vrl = veh_obj.end_current_leg(sim_time)
            veh_obj.start_next_leg(sim_time)
            self.fleetctrl.receive_status_update(veh_obj.vid, sim_time, [charging_vrl])
        empty_plan = VehiclePlan(veh_obj, sim_time, self.fleetctrl.routing_engine, [])
        self.fleetctrl.assign_vehicle_plan(veh_obj, empty_plan, sim_time, force_assign=True)
        self.fleetctrl.receive_status_update(veh_obj.vid, sim_time, [], force_update=True)
