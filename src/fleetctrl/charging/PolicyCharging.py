"""Charging policies with a charge reserve, and driver lunch breaks taken at charging sites.

Written for studies/carta_go (SPEC.md there defines every rule); the module is generic FleetPy otherwise.

Every plan of the run must keep, after each leg, at least the charge needed to reach the nearest charging site plus a
margin (the reserve), so the optimizer never strands a vehicle. The reserve and the "lunch must still fit" rule are
enforced through VehiclePlan.plan_rules, which this module sets at the start of the run.

Policies (op_charge_policy):
    threshold: when the planned charge at the end of a vehicle's plan falls below op_min_soc_after_planstop, book a
        charge after its last stop at the site whose booking finishes first.
    idle: threshold, plus 15-minute charges whenever a vehicle below op_charge_idle_below_soc is free to charge now.
    scheduled: threshold, plus one charge of up to op_break_duration per vehicle in a staggered window (no driver).
With op_break_duration and policy threshold or idle, every vehicle's driver takes one lunch of op_break_duration at a
charging site in a staggered window, charging there if a socket is free.
"""
from __future__ import annotations

import csv
import logging
import math
import os
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from src.fleetctrl.charging.ChargingBase import ChargingBase
from src.fleetctrl.planning.VehiclePlan import ChargingPlanStop, RoutingTargetPlanStop, VehiclePlan
from src.misc.globals import *

if TYPE_CHECKING:
    from src.fleetctrl.FleetControlBase import FleetControlBase
    from src.infra.ChargingInfrastructure import ChargingStation

LOG = logging.getLogger(__name__)

G_OP_CH_POLICY = "op_charge_policy"
G_OP_CH_IDLE_BELOW = "op_charge_idle_below_soc"
G_OP_CH_MIN_S = "op_charge_min_duration"
G_OP_CH_RESERVE_MARGIN = "op_charge_reserve_margin"
G_OP_CH_GARAGE_RESERVE_FROM = "op_charge_garage_reserve_from"
G_OP_RETURN_TIME = "op_return_to_garage_time"
G_OP_BREAK_S = "op_break_duration"
G_OP_BREAK_FIRST = "op_break_first_slot"
G_OP_BREAK_SPAN = "op_break_slot_span"
G_OP_BREAK_BEFORE = "op_break_window_before"
G_OP_BREAK_AFTER = "op_break_window_after"
G_OP_BREAK_SHARE = "op_break_max_share"
POLICIES = ("threshold", "idle", "scheduled")
CHECK_STEP_S = 60  # commitment runs every minute; the planning check keeps this much slack before the latest start
# set POLICY_CHARGING_VERIFY_GAP=1 to check every direct gap evaluation against a full trial plan (slow; for tests)
VERIFY_GAP = os.environ.get("POLICY_CHARGING_VERIFY_GAP") == "1"

INPUT_PARAMETERS_PolicyCharging = {
    "doc": __doc__,
    "inherit": "ChargingBase",
    "input_parameters_mandatory": [G_OP_CH_POLICY, G_OP_APS_SOC, G_OP_CH_MIN_S, G_OP_CH_RESERVE_MARGIN,
                                   G_OP_CH_GARAGE_RESERVE_FROM, G_OP_RETURN_TIME],
    "input_parameters_optional": [G_OP_CH_IDLE_BELOW, G_OP_BREAK_S, G_OP_BREAK_FIRST, G_OP_BREAK_SPAN,
                                  G_OP_BREAK_BEFORE, G_OP_BREAK_AFTER, G_OP_BREAK_SHARE],
    "mandatory_modules": [],
    "optional_modules": [],
}


def _number(attrs, key, low=None, high=None):
    """A required numeric operator attribute, checked against its bounds."""
    value = attrs.get(key)
    if value is None or (isinstance(value, float) and math.isnan(value)):
        raise ValueError(f"PolicyCharging needs {key}")
    value = float(value)
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f"{key} must be in [{low}, {high}], got {value}")
    return value


@dataclass
class Break:
    """One vehicle's lunch (or scheduled charge): its window, and what happened."""
    slot: float
    start_min: float
    start_max: float
    state: str = "pending"  # pending -> committed -> done
    site_id: Optional[int] = None
    committed_at: Optional[float] = None
    arrival: Optional[float] = None
    end: Optional[float] = None
    charge_start: Optional[float] = None
    charge_end: Optional[float] = None


def break_windows(n_vehicles, first_slot, span, before, after):
    """Slot and allowed start window per vehicle: slot_i = first + i * span / n, rounded to the minute."""
    out = []
    for i in range(n_vehicles):
        slot = first_slot + 60 * round(i * span / n_vehicles / 60)
        out.append((slot, slot - before, slot + after))
    return out


class PolicyCharging(ChargingBase):
    def __init__(self, fleetctrl: FleetControlBase, operator_attributes: dict, solver="Gurobi"):
        super().__init__(fleetctrl, operator_attributes, solver=solver)
        a = operator_attributes
        self.policy = a.get(G_OP_CH_POLICY)
        if self.policy not in POLICIES:
            raise ValueError(f"{G_OP_CH_POLICY} must be one of {POLICIES}, got {self.policy!r}")
        self.threshold_soc = _number(a, G_OP_APS_SOC, 0, 1)
        self.min_charge_s = _number(a, G_OP_CH_MIN_S, 60)
        self.margin = _number(a, G_OP_CH_RESERVE_MARGIN, 0, 1)
        self.garage_reserve_from = _number(a, G_OP_CH_GARAGE_RESERVE_FROM, 0)
        self.return_time = _number(a, G_OP_RETURN_TIME, 0)
        self.idle_below = _number(a, G_OP_CH_IDLE_BELOW, 0, 1) if self.policy == "idle" else None
        break_s = a.get(G_OP_BREAK_S)
        has_break = break_s is not None and not (isinstance(break_s, float) and math.isnan(break_s))
        if self.policy == "scheduled" and not has_break:
            raise ValueError(f"{G_OP_CH_POLICY}=scheduled needs {G_OP_BREAK_S} and the break window parameters")
        self.lunch = has_break and self.policy != "scheduled"
        self.breaks: Dict[int, Break] = {}
        if has_break:
            self.break_s = _number(a, G_OP_BREAK_S, 60)
            windows = break_windows(len(fleetctrl.sim_vehicles), _number(a, G_OP_BREAK_FIRST, 0),
                                    _number(a, G_OP_BREAK_SPAN, 0), _number(a, G_OP_BREAK_BEFORE, 0),
                                    _number(a, G_OP_BREAK_AFTER, 0))
            self.breaks = {veh.vid: Break(*w) for veh, w in zip(fleetctrl.sim_vehicles, windows)}
            self.break_cap = max(1, math.ceil(_number(a, G_OP_BREAK_SHARE, 0, 1) * len(fleetctrl.sim_vehicles)))
        if VehiclePlan.plan_rules is not None:
            raise ValueError("PolicyCharging supports one operator per run (another operator already set plan rules)")
        if getattr(fleetctrl, "n_cpu", 1) != 1:
            raise ValueError("PolicyCharging needs n_cpu_per_sim = 1: parallel optimizer workers do not see its rules")
        reopt = a.get(G_RA_REOPT_TS)
        if reopt is not None and int(reopt) != CHECK_STEP_S:
            raise ValueError(f"PolicyCharging commits breaks every {CHECK_STEP_S} s, so {G_RA_REOPT_TS} must be "
                             f"{CHECK_STEP_S}, got {reopt}")
        if self.breaks and (_number(a, G_OP_BREAK_BEFORE, 0) + _number(a, G_OP_BREAK_AFTER, 0) <= 2 * CHECK_STEP_S
                            or max(b.start_max for b in self.breaks.values()) + self.break_s
                            > self.garage_reserve_from):
            raise ValueError("break windows must be wider than 2 minutes and every break must end before "
                             f"{G_OP_CH_GARAGE_RESERVE_FROM}")
        if self.op_charge_depot_infra is None or self.list_pub_charging_infra:
            raise ValueError("PolicyCharging uses the operator's depot file for every site and no public stations")
        self.infra = self.op_charge_depot_infra
        self.sites: List[ChargingStation] = sorted(self.infra.station_by_id.values(), key=lambda s: s.id)
        garages = [s for s in self.sites if s.id in self.infra.depot_by_id]
        if len(garages) != 1:
            raise ValueError(f"PolicyCharging needs exactly one site with parking (the garage), got {len(garages)}")
        self.garage = garages[0]
        self.returned = False
        self._build_reserve_tables()
        VehiclePlan.plan_rules = self
        out_dir = self.fleetctrl.dir_names[G_DIR_OUTPUT]
        self.bookings_file = os.path.join(out_dir, "charge_bookings.csv")
        with open(self.bookings_file, "w", newline="") as f:
            csv.writer(f).writerow(["vehicle_id", "booked_at", "kind", "site_id", "start", "end", "soc", "rescue"])
        with open(os.path.join(out_dir, "charging_sites_loaded.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["site_id", "node_index", "sockets", "power_kw", "garage"])
            for st in self.sites:
                powers = sorted({sk.max_socket_power for sk in st._sockets.values()})
                w.writerow([st.id, st.pos[0], len(st._sockets), ";".join(map(str, powers)), st is self.garage])
        if self.breaks:
            self._write_break_log()
        LOG.info(f"PolicyCharging: policy {self.policy}, lunch {self.lunch}, sites {[s.id for s in self.sites]}, "
                 f"garage {self.garage.id}, breaks {len(self.breaks)}")

    # ------------------------------------------------------------------------------------------------------------
    # reserve and plan rules (called from VehiclePlan.update_tt_and_check_plan)
    # ------------------------------------------------------------------------------------------------------------
    def _build_reserve_tables(self):
        """Per road point: distance and charge to the nearest site and to the garage, plus the nearest sites in order."""
        veh = self.fleetctrl.sim_vehicles[0]
        site_pos = [s.pos for s in self.sites]
        self.site_order: Dict[int, List[Tuple[float, float, ChargingStation]]] = {}
        self.reserve_site: Dict[int, float] = {}
        self.reserve_garage: Dict[int, float] = {}
        by_pos = {s.pos: s for s in self.sites}
        for node in range(self.routing_engine.get_number_network_nodes()):
            pos = self.routing_engine.return_node_position(node)
            costs = self.routing_engine.return_travel_costs_1toX(pos, site_pos)
            order = sorted(((dis, tt, by_pos[d]) for d, _, tt, dis in costs), key=lambda x: (x[0], x[2].id))
            if len(order) != len(self.sites):
                raise ValueError(f"road point {node} reaches {len(order)} of {len(self.sites)} charging sites")
            self.site_order[node] = order
            self.reserve_site[node] = veh.compute_soc_consumption(order[0][0]) + self.margin
            garage_dis = next(dis for dis, _, s in order if s is self.garage)
            self.reserve_garage[node] = veh.compute_soc_consumption(garage_dis) + self.margin
        LOG.info(f"reserve tables for {len(self.reserve_site)} road points: site reserve "
                 f"{min(self.reserve_site.values()):.3f}-{max(self.reserve_site.values()):.3f}, garage reserve "
                 f"{min(self.reserve_garage.values()):.3f}-{max(self.reserve_garage.values()):.3f}")

    def reserve(self, node, planned_time):
        """Charge a vehicle must keep at a road point at a planned time (energy rule 1)."""
        table = self.reserve_garage if planned_time >= self.garage_reserve_from else self.reserve_site
        return table[node]

    def lowest_soc(self, plan, stop_index, pstop, arrival_time, prev_pos):
        """Lowest planned charge allowed on arriving at a plan stop from prev_pos: the reserve, or 0 for the rescue
        (energy rule 3: driving straight to the nearest site to charge is always allowed)."""
        node = pstop.get_pos()[0]
        if pstop.get_charging_power() > 0:
            if stop_index == 0 and node == self._nearest_site_pos(prev_pos)[0]:
                return 0.0
            return self.reserve_site[node]  # at a charger the van charges next, whatever the time (energy rule 7)
        return self.reserve(node, arrival_time)

    def _nearest_site_pos(self, pos):
        """Position of the site nearest (by driving distance) to a position, which may lie between road points."""
        if pos[1] is None:
            return self.site_order[pos[0]][0][2].pos
        return min(((self.routing_engine.return_travel_costs_1to1(pos, st.pos)[2], st.id, st.pos) for st in self.sites))[2]

    def check_plan(self, plan, veh_obj, sim_time, routing_engine):
        """A plan of a vehicle whose lunch is pending must leave a gap for it (lunch rule "Planning")."""
        brk = self.breaks.get(veh_obj.vid)
        if brk is None or brk.state != "pending" or sim_time > brk.start_max:
            return True
        on_board = veh_obj.get_nr_pax_without_currently_boarding()  # boarding riders are counted at the stop
        if on_board + sum(ps.get_change_nr_pax() for ps in plan.list_plan_stops) > 0:
            return True  # an incomplete plan (insertion adds the pickup first); the full plan is checked again
        fits = self._break_gap(plan, veh_obj, sim_time, brk) is not None
        if VERIFY_GAP and fits != (self._break_gap_by_trials(plan, veh_obj, sim_time, brk) is not None):
            raise AssertionError(f"lunch check says {fits} for vid {veh_obj.vid} at {sim_time}, trial plans disagree: "
                                 f"{[str(ps) for ps in plan.list_plan_stops]}")
        if fits:
            return True
        if LOG.isEnabledFor(logging.DEBUG):
            LOG.debug(f"plan of vid {veh_obj.vid} at {sim_time} leaves no gap for its break "
                      f"({brk.start_min}-{brk.start_max}): {[str(ps) for ps in plan.list_plan_stops]}")
        return False

    def _break_gap(self, plan, veh_obj, sim_time, brk):
        """Index of a gap of the plan where the break fits, or None.

        A gap before stop k (k = len means after the last stop) needs no rider on board and must come after any locked
        stop and after any booked charge (a van with a booked charge cannot commit, energy rule 8). The break is tried
        there as a virtual stop at the site nearest the point before the gap. A van commits only once its window is
        open and drives to the site from there, so the break starts no earlier than the window start plus that drive
        and no later than start_max - 2 steps; it lasts 2 steps longer than the break (the van commits up to a step
        after the gap opens). The plan with it must pass the normal checks. No charge is credited for the break.

        The gap after the last stop is tested first without a trial plan: the stops before it are already feasible,
        and the drive to the nearest site keeps the reserve (the last stop's reserve covers that drive, and every
        window ends before the garage reserve applies), so only the start time decides.
        """
        stops = plan.list_plan_stops
        latest = brk.start_max - 2 * CHECK_STEP_S  # slack for the one-minute commitment step (see _try_break)
        last_dep = stops[-1].get_planned_arrival_and_departure_time()[1]
        last_soc = stops[-1].get_planned_arrival_and_departure_soc()[1]
        tt_end = self.site_order[stops[-1].get_pos()[0]][0][1]
        if last_dep is not None and last_soc is not None and last_soc >= self.reserve_site[stops[-1].get_pos()[0]] \
                and max(last_dep, brk.start_min) + tt_end <= latest:
            return len(stops)
        on_board = veh_obj.get_nr_pax_without_currently_boarding()  # boarding riders are counted at the stop
        first_open = 0
        for k, ps in enumerate(stops):
            if ps.is_locked() or ps.is_locked_end() or ps.is_infeasible_locked() or ps.get_charging_power() > 0:
                first_open = k + 1
        for k in range(len(stops)):  # the gap after the last stop failed above
            if k > 0:
                on_board += stops[k - 1].get_change_nr_pax()
            if k < first_open or on_board > 0:
                continue
            prev_time = sim_time if k == 0 else stops[k - 1].get_planned_arrival_and_departure_time()[1]
            if prev_time is None or prev_time > latest:
                break  # later gaps start even later
            if stops[k].get_latest_start_time(plan.pax_info) < brk.start_min + self.break_s:
                continue  # the next stop must start before a break in this gap could end
            if k == 0:  # the van may be between road points
                prev_pos, prev_soc = veh_obj.pos, veh_obj.soc
                tt, site, dis = self._sites_from(veh_obj, with_distance=True)[0]
            else:
                prev_pos, prev_soc = stops[k - 1].get_pos(), stops[k - 1].get_planned_arrival_and_departure_soc()[1]
                dis, tt, site = self.site_order[prev_pos[0]][0]
            if self._gap_fits(plan, veh_obj, k, brk, prev_pos, prev_time, prev_soc, site, tt, dis):
                return k
        return None

    def _break_gap_by_trials(self, plan, veh_obj, sim_time, brk):
        """Reference for _break_gap (VERIFY_GAP only): every allowed gap tried as a full trial plan, no shortcuts."""
        stops = plan.list_plan_stops
        latest = brk.start_max - 2 * CHECK_STEP_S
        on_board = veh_obj.get_nr_pax_without_currently_boarding()
        first_open = max([k + 1 for k, ps in enumerate(stops) if ps.is_locked() or ps.is_locked_end()
                          or ps.is_infeasible_locked() or ps.get_charging_power() > 0] or [0])
        for k in range(len(stops) + 1):
            if k > 0:
                on_board += stops[k - 1].get_change_nr_pax()
            if k < first_open or on_board > 0:
                continue
            if k == 0:
                tt, site = self._sites_from(veh_obj)[0]
            else:
                _, tt, site = self.site_order[stops[k - 1].get_pos()[0]][0]
            virtual = RoutingTargetPlanStop(site.pos, earliest_start_time=brk.start_min + tt, latest_start_time=latest,
                                            duration=self.break_s + 2 * CHECK_STEP_S)
            trial = VehiclePlan(veh_obj, sim_time, self.routing_engine,
                                [ps.copy() for ps in stops[:k]] + [virtual] + [ps.copy() for ps in stops[k:]],
                                copy=True, external_pax_info={})
            if self._feasible_without_break_rule(trial, veh_obj, sim_time):
                return k
        return None

    def _gap_fits(self, plan, veh_obj, k, brk, prev_pos, prev_time, prev_soc, site, tt1, dis1):
        """Whether the break fits in the gap before stop k, computed directly from the plan's planned times and
        charges (the same arithmetic as update_tt_and_check_plan on the plan with the break inserted, without
        building it): the break starts at max(prev_time, start_min) + tt1 and lasts 2 steps longer than the break,
        stops k.. shift later (waiting before an earliest start absorbs part of the shift), and every one of them must
        still start by its latest start (boarding times shift too, for the ride-time limits) and arrive with its
        reserve, after the extra driving of the detour to the site."""
        stops = plan.list_plan_stops
        start = max(prev_time, brk.start_min) + tt1
        if start > brk.start_max - 2 * CHECK_STEP_S:
            return False
        if prev_pos != site.pos and \
                prev_soc - veh_obj.compute_soc_consumption(dis1) < self.reserve(site.pos[0], prev_time + tt1):
            return False  # (the plan check tests the reserve only after a drive)
        nxt = stops[k].get_pos()
        _, tt2, dis2 = self.routing_engine.return_travel_costs_1to1(site.pos, nxt)
        dis0 = 0.0 if prev_pos == nxt else self.routing_engine.return_travel_costs_1to1(prev_pos, nxt)[2]
        extra = veh_obj.compute_soc_consumption(dis1 + dis2) - veh_obj.compute_soc_consumption(dis0)
        pax = {rid: list(v) for rid, v in plan.pax_info.items()}
        arrival = start + self.break_s + 2 * CHECK_STEP_S + tt2
        prev_dep_old = prev_dep_new = None
        for j in range(k, len(stops)):
            ps = stops[j]
            old_arr, old_dep = ps.get_planned_arrival_and_departure_time()
            if j > k:
                arrival = old_arr + (prev_dep_new - prev_dep_old)
            before = site.pos if j == k else stops[j - 1].get_pos()
            if ps.get_pos() != before and ps.get_planned_arrival_and_departure_soc()[0] - extra < self.lowest_soc(
                    plan, j + 1, ps, arrival, before):
                return False  # (the plan check tests the reserve only after a drive)
            begin = max(arrival, ps.get_earliest_start_time())
            for rid in ps.get_list_boarding_rids():
                pax[rid] = [begin]
            if begin > ps.get_latest_start_time(pax):
                return False
            prev_dep_old, prev_dep_new = old_dep, ps.get_departure_time(begin)
        return True

    def _feasible_without_break_rule(self, plan, veh_obj, sim_time):
        """Normal plan check (times, capacity, reserve) with the break rule switched off for this vehicle."""
        brk = self.breaks[veh_obj.vid]
        state, brk.state = brk.state, "checking"
        try:
            return plan.update_tt_and_check_plan(veh_obj, sim_time, self.routing_engine, keep_feasible=False)
        finally:
            brk.state = state

    # ------------------------------------------------------------------------------------------------------------
    # time trigger
    # ------------------------------------------------------------------------------------------------------------
    def time_triggered_charging_processes(self, sim_time):
        done = [b for b in self.breaks.values() if b.state == "committed" and sim_time >= b.end]
        for brk in done:
            brk.state = "done"
        if done:
            self._write_break_log()
        if sim_time >= self.return_time:
            if not self.returned:
                self._send_home(sim_time)
            return
        free = [veh for veh in self.fleetctrl.sim_vehicles
                if veh.status not in {VRL_STATES.OUT_OF_SERVICE, VRL_STATES.BLOCKED_INIT}]
        # 1) break commitments for every van before any other charge takes a socket this minute
        open_window = set()
        for veh in free:
            brk = self.breaks.get(veh.vid)
            if brk is not None and brk.state == "pending" and brk.start_min <= sim_time <= brk.start_max:
                open_window.add(veh.vid)
                if not (self._has_charge(veh) or self._first_stop_locked(veh)):
                    self._try_break(veh, brk, sim_time)
        # 2) threshold and idle charges
        for veh in free:
            if self._has_charge(veh) or self._first_stop_locked(veh):
                continue
            brk = self.breaks.get(veh.vid)
            window_open = veh.vid in open_window and brk.state == "pending"
            if self._plan_end(veh, sim_time)[2] < self.threshold_soc:
                # in an open window the break charge is the threshold charge; if it cannot be committed, charge anyway
                if not window_open or self._try_break(veh, brk, sim_time, over_cap=True) == "impossible":
                    self._threshold_charge(veh, brk, sim_time)
                continue
            if self.policy == "idle" and not window_open and veh.soc < self.idle_below and not veh.pax:
                self._idle_charge(veh, sim_time)

    def _has_charge(self, veh):
        return self.fleetctrl._vid_to_assigned_charging_process.get(veh.vid) is not None

    def _first_stop_locked(self, veh):
        stops = self.fleetctrl.veh_plans[veh.vid].list_plan_stops
        return bool(stops) and (stops[0].is_locked() or stops[0].is_locked_end())

    def _plan_end(self, veh, sim_time):
        """Position, time and planned charge at the end of the vehicle's current plan."""
        stops = self.fleetctrl.veh_plans[veh.vid].list_plan_stops
        if not stops:
            return veh.pos, sim_time, veh.soc
        _, dep = stops[-1].get_planned_arrival_and_departure_time()
        _, soc = stops[-1].get_planned_arrival_and_departure_soc()
        return stops[-1].get_pos(), dep, soc

    # ------------------------------------------------------------------------------------------------------------
    # sockets and bookings
    # ------------------------------------------------------------------------------------------------------------
    @staticmethod
    def _schedules(station):
        """Busy (start, end) spans per socket: running charges until their booked end (vehicles leave then, see
        Vehicles._start_next_leg_stationary_object), and every future booking."""
        running, booked = station.get_running_processes()
        spans = {socket_id: [] for socket_id in station._sockets}
        for socket_id, proc in running.items():
            if proc is not None:  # a late start shifts the end, since the van charges its booked duration
                late_end = proc.veh.cl_start_time + proc.end_time - proc.start_time
                spans[socket_id].append((proc.start_time, max(proc.end_time, late_end)))
        for socket_id, procs in booked.items():
            spans[socket_id] += [(proc.start_time, proc.end_time) for proc in procs]
        return spans

    @classmethod
    def _earliest_slot(cls, station, sim_time, start, duration):
        """(socket id, start) of the earliest booking of `duration` seconds at the station from `start` on."""
        best = None
        for socket_id, schedule in sorted(cls._schedules(station).items()):
            t = start
            for b_start, b_end in sorted(schedule):
                if t + duration <= b_start:
                    break
                t = max(t, b_end)
            if best is None or t < best[1]:
                best = (socket_id, t)
        return best

    @staticmethod
    def _charge_stop(station, socket_id, start, duration, task_id=None):
        """Locked charging plan stop for a booking from start to start + duration; it may start at most one
        commitment step late."""
        power = station._sockets[socket_id].max_socket_power
        return ChargingPlanStop(station.pos, earliest_start_time=start, latest_start_time=start + CHECK_STEP_S,
                                duration=duration, charging_power=power, charging_task_id=task_id, locked=True)

    def _book(self, station, socket_id, veh, start, duration, sim_time, kind):
        """Book the socket, record the booking in charge_bookings.csv and return the charging plan stop with its
        (task id, booking). A rescue is a booking made while the vehicle is below its reserve."""
        booking = self.infra.book_station(sim_time, veh, station.id, socket_id, start, start + duration)
        b_start, b_end = booking.get_scheduled_start_end_times()
        if (b_start, b_end) != (start, start + duration):
            raise RuntimeError(f"booking for vid {veh.vid} at site {station.id} came back as {b_start}-{b_end}, "
                               f"asked {start}-{start + duration}")
        task_id = (self.infra.ch_op_id, booking.id)
        rescue = veh.soc < self.reserve_site[veh.pos[0]]  # below the charge needed to reach the nearest site
        with open(self.bookings_file, "a", newline="") as f:
            csv.writer(f).writerow([veh.vid, sim_time, kind, station.id, start, start + duration, veh.soc, rescue])
        return self._charge_stop(station, socket_id, start, duration, task_id), (task_id, booking)

    def _assign(self, veh, stops, sim_time, charging_task, lock_first):
        """Assign a plan and lock its first `lock_first` stops, with the optimizer's bookkeeping."""
        plan = VehiclePlan(veh, sim_time, self.routing_engine, stops)
        for ps in plan.list_plan_stops[:lock_first]:
            ps.set_locked(True)
        self.fleetctrl.assign_vehicle_plan(veh, plan, sim_time, assigned_charging_task=charging_task)
        rpbo = getattr(self.fleetctrl, "RPBO_Module", None)
        if rpbo is not None:  # what the batch lock_current_vehicle_plan does, for the locked stops only
            assigned = self.fleetctrl.veh_plans[veh.vid]
            rpbo.set_assignment(veh.vid, assigned, is_external_vehicle_plan=True)
            rpbo.delete_vehicle_database_entries(veh.vid)
            for ps in assigned.list_plan_stops[:lock_first]:
                for rid in ps.get_list_boarding_rids() + ps.get_list_alighting_rids():
                    rpbo.lock_request_to_vehicle(rid, veh.vid)  # else the optimizer may give it to another vehicle

    def _trial(self, veh, stops, sim_time):
        """True if a plan made of these stops passes the plan check (the stops are not modified)."""
        plan = VehiclePlan(veh, sim_time, self.routing_engine, [ps.copy() for ps in stops], copy=True,
                           external_pax_info={})
        return plan.update_tt_and_check_plan(veh, sim_time, self.routing_engine, keep_feasible=False)

    # ------------------------------------------------------------------------------------------------------------
    # policies
    # ------------------------------------------------------------------------------------------------------------
    def _threshold_charge(self, veh, brk, sim_time):
        """Book a charge after the last stop at the site whose booking finishes first (energy rule 7)."""
        current = self.fleetctrl.veh_plans[veh.vid].list_plan_stops
        end_pos, end_time, end_soc = self._plan_end(veh, sim_time)
        step = int(self.min_charge_s)
        full = {}  # site id -> charge time to 100 % from the charge on arrival there
        for dis, _, station in self.site_order[end_pos[0]]:
            power = max(sk.max_socket_power for sk in station._sockets.values())
            full_s = veh.get_charging_duration(power, max(end_soc - veh.compute_soc_consumption(dis), 0.0), 1.0)
            full[station.id] = max(step, 60 * math.ceil(full_s / 60))
        levels = [None]  # None: the full charge per site
        if brk is not None and brk.state == "pending":
            # shorter charges in 15-minute steps so the break still fits; nothing if even the shortest does not
            top = max(full.values())
            levels += [d for d in range(top - top % step, step - 1, -step) if d < top]
        for level in levels:
            offers = []
            for dis, tt, station in self.site_order[end_pos[0]]:
                duration = full[station.id] if level is None else min(level, full[station.id])
                socket_id, start = self._earliest_slot(station, sim_time, end_time + tt, duration)
                offers.append((start + duration, dis, station.id, station, socket_id, start, duration))
            for _, _, _, station, socket_id, start, duration in sorted(offers, key=lambda o: o[:3]):
                stops = [s.copy() for s in current] + [self._charge_stop(station, socket_id, start, duration)]
                if self._trial(veh, stops, sim_time):
                    ps, task = self._book(station, socket_id, veh, start, duration, sim_time, "threshold")
                    stops[-1] = ps
                    self._assign(veh, stops, sim_time, task, lock_first=len(stops))
                    LOG.debug(f"threshold charge vid {veh.vid} at site {station.id} {start}-{start + duration}")
                    return True
        LOG.debug(f"threshold charge for vid {veh.vid} rejected at {sim_time}")
        return False

    def _idle_charge(self, veh, sim_time):
        """A 15-minute charge starting now at the nearest site, if the plan with it first passes the check."""
        current = self.fleetctrl.veh_plans[veh.vid].list_plan_stops
        tt, station = self._sites_from(veh)[0]
        socket_id, start = self._earliest_slot(station, sim_time, sim_time + tt, self.min_charge_s)
        if start > sim_time + tt:
            return False  # no socket free on arrival
        stops = [self._charge_stop(station, socket_id, start, self.min_charge_s)] + [s.copy() for s in current]
        if not self._trial(veh, stops, sim_time):
            return False
        stops[0], task = self._book(station, socket_id, veh, start, self.min_charge_s, sim_time, "idle")
        self._assign(veh, stops, sim_time, task, lock_first=1)
        LOG.debug(f"idle charge vid {veh.vid} at site {station.id} {start}-{start + self.min_charge_s}")
        return True

    def _sites_from(self, veh, with_distance=False):
        """(travel time, site), or (travel time, site, distance), from the vehicle's current position (it may be
        between road points), nearest by driving distance first."""
        out = []
        for st in self.sites:
            _, tt, dis = self.routing_engine.return_travel_costs_1to1(veh.pos, st.pos)
            out.append((dis, st.id, tt, st))
        out.sort(key=lambda x: x[:2])
        return [(tt, st, dis) if with_distance else (tt, st) for dis, _, tt, st in out]

    def _break_options(self, veh, brk, sim_time):
        """Ways to take the break now, in preference order: (site, start, charge seconds, socket, last resort).

        Sites with a socket free on arrival come first, nearest first. A lunch then falls back to the nearest site
        without charging. A scheduled charge falls back to the site whose earliest free slot, waited for, starts first
        within the window; that is a last resort, used only when the break would no longer fit a step later.
        """
        options, waits = [], []
        sites = self._sites_from(veh, with_distance=True)
        for tt, st, dis in sites:
            arrival = max(sim_time + tt, brk.start_min)
            if arrival > brk.start_max:
                continue
            soc_on_arrival = max(veh.soc - veh.compute_soc_consumption(dis), 0.0)
            power = max(sk.max_socket_power for sk in st._sockets.values())
            charge_s = min(self.break_s, veh.get_charging_duration(power, soc_on_arrival, 1.0))
            if self.policy == "scheduled":
                charge_s = max(charge_s, self.min_charge_s)  # a nearly full van still books; it leaves when full
            if charge_s < self.min_charge_s:
                continue
            socket_id, start = self._earliest_slot(st, sim_time, arrival, charge_s)
            if start <= arrival:
                options.append((st, arrival, charge_s, socket_id, False))
            elif start <= brk.start_max:
                waits.append((start + charge_s, st.id, st, start, charge_s, socket_id))
        tt, nearest, _ = sites[0]
        if self.lunch and sim_time + tt <= brk.start_max:
            options.append((nearest, max(sim_time + tt, brk.start_min), 0, None, False))
        elif waits:
            _, _, st, start, charge_s, socket_id = min(waits, key=lambda w: w[:2])
            options.append((st, start, charge_s, socket_id, True))
        return options

    def _break_stops(self, station, start, charge_s, socket_id, delay=0):
        """The break's plan stops if it starts `delay` seconds after `start`: a charge (if any) and, for a lunch, a
        locked wait to start + break duration."""
        start = start + delay
        stops = []
        if charge_s > 0:
            stops.append(self._charge_stop(station, socket_id, start, charge_s))
        if self.lunch:
            stops.append(RoutingTargetPlanStop(station.pos, earliest_start_time=start if charge_s == 0 else None,
                                               duration=0, earliest_end_time=start + self.break_s, locked=True,
                                               planstop_state=G_PLANSTOP_STATES.REPO_TARGET))
        return stops

    def _try_break(self, veh, brk, sim_time, over_cap=False):
        """Commit to the lunch or scheduled charge now if it fits now (lunch rule "Commitment"). Returns "committed",
        "wait" (it fits now and still will later, and the van waits) or "impossible".

        With the limit on vans at a break reached (and not over_cap), or for a last-resort option, the van commits
        only if the break would no longer fit later: starting two steps later (the planning check's slack) would be
        past the window or would fail the plan check.
        """
        if veh.pax:
            return "impossible"
        current = [s.copy() for s in self.fleetctrl.veh_plans[veh.vid].list_plan_stops]
        capped = not over_cap and sum(1 for b in self.breaks.values() if b.state == "committed") >= self.break_cap
        brk.state = "committed"  # a plan with the break in it no longer needs a gap for it
        try:
            for station, start, charge_s, socket_id, last_resort in self._break_options(veh, brk, sim_time):
                if not self._trial(veh, self._break_stops(station, start, charge_s, socket_id) + current, sim_time):
                    continue
                later = 2 * CHECK_STEP_S
                if (capped or last_resort) and start + later <= brk.start_max and self._trial(
                        veh, self._break_stops(station, start, charge_s, socket_id, later) + current, sim_time):
                    return "wait"  # it still fits later
                self._commit_break(veh, brk, station, start, charge_s, socket_id, current, sim_time)
                return "committed"
            return "impossible"
        finally:
            if brk.committed_at is None:
                brk.state = "pending"

    def _commit_break(self, veh, brk, station, start, charge_s, socket_id, current, sim_time):
        """Book the break's charge (if any) and assign the plan with the break first."""
        stops, task = self._break_stops(station, start, charge_s, socket_id), None
        if charge_s > 0:
            kind = "lunch" if self.lunch else "scheduled"
            stops[0], task = self._book(station, socket_id, veh, start, charge_s, sim_time, kind)
            brk.charge_start, brk.charge_end = start, start + charge_s
        self._assign(veh, stops + current, sim_time, task, lock_first=len(stops))
        brk.site_id, brk.committed_at, brk.arrival = station.id, sim_time, start
        brk.end = start + (self.break_s if self.lunch else charge_s)
        self._write_break_log()
        LOG.debug(f"break vid {veh.vid} committed at {sim_time}: site {station.id}, {start}-{brk.end}, "
                  f"charge {brk.charge_start}-{brk.charge_end}")

    def _send_home(self, sim_time):
        """Append a closing garage stop to every plan and lock it (end of day)."""
        for veh in self.fleetctrl.sim_vehicles:
            current = self.fleetctrl.veh_plans[veh.vid].list_plan_stops
            stops = [s.copy() for s in current]
            if not (stops and stops[-1].get_pos() == self.garage.pos and stops[-1].is_locked_end()):
                stops.append(RoutingTargetPlanStop(self.garage.pos, locked=True, locked_end=True))
            plan = VehiclePlan(veh, sim_time, self.routing_engine, stops)
            self.fleetctrl.assign_vehicle_plan(veh, plan, sim_time)
            self.fleetctrl.lock_current_vehicle_plan(veh.vid)
        self.returned = True
        LOG.info(f"end of day at {sim_time}: {len(self.fleetctrl.sim_vehicles)} vehicles sent to the garage")

    # ------------------------------------------------------------------------------------------------------------
    # records
    # ------------------------------------------------------------------------------------------------------------
    def _write_break_log(self):
        """breaks.csv in the run's output folder: one row per vehicle with its break window and what happened,
        rewritten (atomically) at every change, for the study's checks."""
        path = os.path.join(self.fleetctrl.dir_names[G_DIR_OUTPUT], "breaks.csv")
        tmp = path + ".tmp"
        with open(tmp, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["vehicle_id", "slot", "start_min", "start_max", "state", "site_id", "committed_at",
                        "arrival", "end", "charge_start", "charge_end"])
            for vid, b in sorted(self.breaks.items()):
                w.writerow([vid, b.slot, b.start_min, b.start_max, b.state, b.site_id, b.committed_at, b.arrival,
                            b.end, b.charge_start, b.charge_end])
        os.replace(tmp, path)
