import json
import math
import time
from pathlib import Path


def load_scenario_specs(path, scenario_name):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    scenario = data["scenarios"][scenario_name]
    robots = []
    for robot_id, specification in scenario.get("robots", {}).items():
        robots.append({
            "robot_id": int(robot_id),
            "start": tuple(float(value) for value in specification["start"]),
            "goal": tuple(float(value) for value in specification["goal"]),
        })
    if not robots:
        raise ValueError(f"El escenario {scenario_name!r} no contiene robots")
    return scenario, robots


def minimum_pair_distance(states, robot_ids):
    distances = []
    ids = [robot_id for robot_id in robot_ids if states.get(robot_id) is not None]
    for index, first_id in enumerate(ids):
        for second_id in ids[index + 1:]:
            first = states[first_id]
            second = states[second_id]
            distances.append(math.hypot(first["x"] - second["x"], first["y"] - second["y"]))
    return min(distances) if distances else float("inf")


def point_to_segment_distance(point, segment_start, segment_end):
    px, py = point
    ax, ay = segment_start
    bx, by = segment_end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    projection = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    closest = (ax + projection * dx, ay + projection * dy)
    return math.hypot(px - closest[0], py - closest[1])


def order_positioning_specs(robot_specs, states):
    """Ordena recolocaciones para alejar primero la ruta menos conflictiva."""
    remaining = [dict(specification) for specification in robot_specs]
    virtual_positions = {
        robot_id: (float(state["x"]), float(state["y"]))
        for robot_id, state in states.items()
        if state is not None
    }
    ordered = []
    while remaining:
        scored = []
        for specification in remaining:
            robot_id = specification["robot_id"]
            current = virtual_positions.get(robot_id)
            destination = tuple(specification["start"])
            if current is None:
                clearance = -1.0
                move_distance = float("inf")
            else:
                other_positions = [
                    position
                    for other_id, position in virtual_positions.items()
                    if other_id != robot_id
                ]
                clearance = min(
                    (
                        point_to_segment_distance(position, current, destination)
                        for position in other_positions
                    ),
                    default=float("inf"),
                )
                move_distance = math.hypot(
                    destination[0] - current[0], destination[1] - current[1]
                )
            scored.append((clearance, -move_distance, -robot_id, specification))
        _, _, _, selected = max(scored, key=lambda item: item[:3])
        remaining.remove(selected)
        ordered.append(selected)
        virtual_positions[selected["robot_id"]] = tuple(selected["start"])
    return ordered


def select_scenario_direction(robot_specs, states):
    """Elige el sentido que requiere menor recolocacion antes de la prueba."""
    forward = [dict(specification) for specification in robot_specs]
    reverse = [
        {
            **specification,
            "start": tuple(specification["goal"]),
            "goal": tuple(specification["start"]),
        }
        for specification in robot_specs
    ]

    def positioning_cost(specifications):
        cost = 0.0
        for specification in specifications:
            state = states.get(specification["robot_id"])
            if state is None:
                return float("inf")
            start_x, start_y = specification["start"]
            cost += math.hypot(float(state["x"]) - start_x, float(state["y"]) - start_y)
        return cost

    forward_cost = positioning_cost(forward)
    reverse_cost = positioning_cost(reverse)
    if reverse_cost < forward_cost:
        return "reverse", reverse, forward_cost, reverse_cost
    return "forward", forward, forward_cost, reverse_cost


class ScenarioRunAutomation:
    """Ejecuta una repeticion fisica con preparacion y parada supervisadas."""

    def __init__(
        self,
        root,
        app,
        scenario_file,
        scenario_name,
        controller_mode,
        condition,
        replicate,
        on_done,
        scenario_timeout_s=90.0,
        positioning_timeout_s=90.0,
        start_accuracy_m=0.035,
        safety_distance_m=0.12,
        positioning_safety_distance_m=0.08,
        pose_abort_timeout_s=1.20,
        close_pose_timeout_s=0.45,
        close_pose_distance_m=0.35,
        stall_timeout_s=6.0,
        network_abort_timeout_s=30.0,
        scenario_direction="auto",
    ):
        self.root = root
        self.app = app
        self.scenario_name = str(scenario_name)
        self.controller_mode = str(controller_mode)
        self.condition = str(condition)
        self.replicate = int(replicate)
        self.on_done = on_done
        self.scenario_timeout_s = float(scenario_timeout_s)
        self.positioning_timeout_s = float(positioning_timeout_s)
        self.start_accuracy_m = float(start_accuracy_m)
        self.safety_distance_m = float(safety_distance_m)
        self.positioning_safety_distance_m = min(
            self.safety_distance_m,
            float(positioning_safety_distance_m),
        )
        self.pose_abort_timeout_s = float(pose_abort_timeout_s)
        self.close_pose_timeout_s = float(close_pose_timeout_s)
        self.close_pose_distance_m = float(close_pose_distance_m)
        self.stall_timeout_s = float(stall_timeout_s)
        self.network_abort_timeout_s = float(network_abort_timeout_s)
        self.requested_direction = str(scenario_direction)
        if self.requested_direction not in {"auto", "forward", "reverse"}:
            raise ValueError(f"Sentido no valido: {self.requested_direction}")
        self.scenario, self.base_robots = load_scenario_specs(scenario_file, scenario_name)
        self.robots = [dict(specification) for specification in self.base_robots]
        self.robot_ids = [item["robot_id"] for item in self.robots]
        self.scenario_direction = "forward"
        self.direction_selected = False
        self.positioning_costs = {"forward_m": None, "reverse_m": None}
        self.phase = "waiting"
        self.phase_started = time.monotonic()
        self.position_index = 0
        self.position_commanded = False
        self.run_dir = None
        self.done = False
        self.last_progress_t = time.monotonic()
        self.progress_reference = {}
        self.last_progress_by_robot = {}
        self.last_discovery_refresh_t = 0.0

    def start(self):
        print(
            f"[PAPER_RUN] preparando {self.scenario_name} | "
            f"{self.controller_mode} | rep={self.replicate}",
            flush=True,
        )
        self.root.after(200, self._tick)

    def _snapshot(self):
        with self.app.lock:
            states = {
                robot_id: (
                    None if self.app.robot_state.get(robot_id) is None
                    else dict(self.app.robot_state[robot_id])
                )
                for robot_id in self.robot_ids
            }
            targets = {robot_id: self.app.targets.get(robot_id) for robot_id in self.robot_ids}
            homography_valid = self.app.homography is not None
            homography_age = (
                time.monotonic() - self.app.homography_t
                if homography_valid else float("inf")
            )
        return states, targets, homography_valid, homography_age

    def _infrastructure_ready(self):
        states, _, homography_valid, homography_age = self._snapshot()
        if not homography_valid or homography_age > self.app.homography_hold_s:
            return False, "homografia no disponible"
        for robot_id in self.robot_ids:
            state = states.get(robot_id)
            if state is None or time.time() - state.get("t", 0.0) > 0.45:
                return False, f"R{robot_id} sin pose reciente"
            _, network_status, _ = self.app.robot_net_status(robot_id)
            if network_status != "OK":
                return False, f"R{robot_id} sin UDP reciente"
        return True, "OK"

    def _safety_reason(self, states, check_distance=True):
        now = time.time()
        distance = minimum_pair_distance(states, self.robot_ids)
        for robot_id, state in states.items():
            if state is None:
                return f"pose_lost_R{robot_id}"
            pose_age = now - state.get("t", 0.0)
            if pose_age > self.pose_abort_timeout_s:
                return f"pose_lost_R{robot_id}"
            if (
                pose_age > self.close_pose_timeout_s
                and distance < self.close_pose_distance_m
            ):
                return f"pose_lost_close_R{robot_id}_{distance:.4f}m"
        if check_distance:
            if distance < self.safety_distance_m:
                return f"critical_separation_{distance:.4f}m"
        return None

    def _network_safety_reason(self):
        for robot_id in self.robot_ids:
            info, _, age = self.app.robot_net_status(robot_id)
            if info is None or age is None:
                return f"network_lost_R{robot_id}"
            if age > self.network_abort_timeout_s:
                return f"network_lost_R{robot_id}_{age:.2f}s"
        return None

    def _abort(self, reason):
        if self.done:
            return
        self.done = True
        self.app._record_event("automation_abort", payload={"reason": reason})
        self.app.stop_all()
        if self.app.recorder.active:
            self.run_dir = self.app.recorder.stop(reason)
        print(
            f"[PAPER_RUN] ABORTADO reason={reason} "
            f"run_dir={self.run_dir or '-'}",
            flush=True,
        )
        self.root.after(500, lambda: self.on_done(False, reason, self.run_dir))

    def _complete(self):
        if self.done:
            return
        self.done = True
        self.app.stop_all()
        self.run_dir = self.app.recorder.stop("completed")
        print(f"[PAPER_RUN] COMPLETADO run_dir={self.run_dir}", flush=True)
        self.root.after(500, lambda: self.on_done(True, "completed", self.run_dir))

    def _begin_positioning(self):
        if not self.direction_selected:
            states, _, _, _ = self._snapshot()
            direction, specifications, forward_cost, reverse_cost = (
                select_scenario_direction(self.base_robots, states)
            )
            if self.requested_direction == "forward":
                direction = "forward"
                specifications = [dict(item) for item in self.base_robots]
            elif self.requested_direction == "reverse":
                direction = "reverse"
                specifications = [
                    {
                        **item,
                        "start": tuple(item["goal"]),
                        "goal": tuple(item["start"]),
                    }
                    for item in self.base_robots
                ]
            self.scenario_direction = direction
            self.robots = order_positioning_specs(specifications, states)
            self.positioning_costs = {
                "forward_m": float(forward_cost),
                "reverse_m": float(reverse_cost),
            }
            self.direction_selected = True
            print(
                f"[PAPER_RUN] sentido={direction} "
                f"coste_forward={forward_cost:.3f}m "
                f"coste_reverse={reverse_cost:.3f}m",
                flush=True,
            )
            print(
                "[PAPER_RUN] orden_posicionamiento="
                + ",".join(f"R{item['robot_id']}" for item in self.robots),
                flush=True,
            )
        self.phase = "positioning"
        self.phase_started = time.monotonic()
        self.position_index = 0
        self.position_commanded = False

    def _tick_positioning(self, states, targets):
        if self.position_index >= len(self.robots):
            self.phase = "pre_run_settle"
            self.phase_started = time.monotonic()
            self.app.stop_all()
            self.app.force_robot_discovery()
            self.last_discovery_refresh_t = time.monotonic()
            print("[PAPER_RUN] posiciones iniciales listas; asentando", flush=True)
            return

        specification = self.robots[self.position_index]
        robot_id = specification["robot_id"]
        start_x, start_y = specification["start"]
        state = states[robot_id]
        distance = math.hypot(state["x"] - start_x, state["y"] - start_y)

        if not self.position_commanded:
            if distance <= self.start_accuracy_m:
                self.position_index += 1
                self.phase_started = time.monotonic()
                return
            self.app.set_planned_target(robot_id, start_x, start_y, update_label=False)
            self.app.control_active.set(True)
            self.position_commanded = True
            self.phase_started = time.monotonic()
            print(
                f"[PAPER_RUN] posicionando R{robot_id} -> "
                f"({start_x:.3f},{start_y:.3f}) desde ({state['x']:.3f},{state['y']:.3f})",
                flush=True,
            )
            return

        if time.monotonic() - self.phase_started > self.positioning_timeout_s:
            self._abort(f"positioning_timeout_R{robot_id}")
            return

        if targets.get(robot_id) is None:
            if distance <= self.start_accuracy_m:
                self.app.send_robot_cmd(robot_id, 0, 0)
                print(
                    f"[PAPER_RUN] R{robot_id} inicio OK; error={distance:.4f}m",
                    flush=True,
                )
                self.position_index += 1
                self.position_commanded = False
                self.phase_started = time.monotonic()
            else:
                self.position_commanded = False

    def _start_experiment(self):
        self.app.experiment_scenario.set(self.scenario_name)
        self.app.experiment_condition.set(self.condition)
        self.app.experiment_replicate.set(self.replicate)
        self.app.experiment_controller_mode.set(self.controller_mode)
        self.app.start_experiment_recording()
        if not self.app.recorder.active:
            self._abort("recorder_start_failed")
            return
        self.run_dir = self.app.recorder.session_dir
        self.app._record_event(
            "automation_config",
            payload={
                "safety_distance_m": self.safety_distance_m,
                "pose_abort_timeout_s": self.pose_abort_timeout_s,
                "close_pose_timeout_s": self.close_pose_timeout_s,
                "close_pose_distance_m": self.close_pose_distance_m,
                "stall_timeout_s": self.stall_timeout_s,
                "network_abort_timeout_s": self.network_abort_timeout_s,
                "scenario_direction": self.scenario_direction,
                "requested_direction": self.requested_direction,
                "positioning_costs": self.positioning_costs,
                "robots": self.robots,
            },
        )
        assignments = []
        for specification in self.robots:
            robot_id = specification["robot_id"]
            goal_x, goal_y = specification["goal"]
            assignments.append((robot_id, float(goal_x), float(goal_y)))
        self.app._record_event(
            "scenario_started",
            payload={
                "scenario": self.scenario_name,
                "condition": self.condition,
                "replicate": self.replicate,
                "direction": self.scenario_direction,
                "robots": [robot_id for robot_id, _, _ in assignments],
                "specifications": self.robots,
            },
        )
        for robot_id, goal_x, goal_y in assignments:
            self.app.set_planned_target(robot_id, goal_x, goal_y, update_label=False)
        self.app.control_active.set(True)
        self.app.experiment_status.set(
            f"Ejecutando: {self.scenario_name} ({self.scenario_direction})"
        )
        with self.app.lock:
            active_targets = [self.app.targets.get(robot_id) for robot_id in self.robot_ids]
        if not any(target is not None for target in active_targets):
            self._abort("scenario_start_failed")
            return
        self.phase = "running"
        self.phase_started = time.monotonic()
        self.last_progress_t = time.monotonic()
        self.progress_reference = {}
        self.last_progress_by_robot = {
            robot_id: self.phase_started for robot_id, _, _ in assignments
        }
        print(f"[PAPER_RUN] escenario iniciado run_dir={self.run_dir}", flush=True)

    def _update_progress(self, states, targets):
        active_ids = [robot_id for robot_id in self.robot_ids if targets.get(robot_id) is not None]
        if not active_ids:
            self.last_progress_t = time.monotonic()
            return
        if not hasattr(self, "last_progress_by_robot"):
            self.last_progress_by_robot = {}
        now = time.monotonic()
        moved = False
        for robot_id in active_ids:
            state = states.get(robot_id)
            if state is None:
                continue
            current = (float(state["x"]), float(state["y"]), float(state["yaw"]))
            previous = self.progress_reference.get(robot_id)
            robot_moved = previous is None
            if previous is None:
                moved = True
            else:
                position_delta = math.hypot(current[0] - previous[0], current[1] - previous[1])
                yaw_delta = abs(math.atan2(
                    math.sin(current[2] - previous[2]),
                    math.cos(current[2] - previous[2]),
                ))
                if position_delta >= 0.015 or yaw_delta >= math.radians(4.0):
                    robot_moved = True
                    moved = True
            if robot_moved:
                self.progress_reference[robot_id] = current
                self.last_progress_by_robot[robot_id] = now
        if moved:
            self.last_progress_t = now

    def _stalled_robot_reason(self, targets, now=None):
        if now is None:
            now = time.monotonic()
        for robot_id in self.robot_ids:
            if targets.get(robot_id) is None:
                continue
            last_progress = self.last_progress_by_robot.get(
                robot_id, self.phase_started
            )
            if now - last_progress > self.stall_timeout_s:
                return f"scenario_stalled_R{robot_id}"
        return None

    def _tick(self):
        if self.done:
            return
        try:
            states, targets, homography_valid, homography_age = self._snapshot()
            if self.phase == "waiting":
                ready, reason = self._infrastructure_ready()
                if ready:
                    self._begin_positioning()
                elif time.monotonic() - self.phase_started > 30.0:
                    self._abort(f"preflight_timeout_{reason.replace(' ', '_')}")

            elif self.phase == "positioning":
                if not homography_valid or homography_age > self.app.homography_hold_s:
                    self._abort("homography_lost_during_positioning")
                else:
                    reason = self._network_safety_reason() or self._safety_reason(
                        states,
                        check_distance=False,
                    )
                    positioning_distance = minimum_pair_distance(states, self.robot_ids)
                    if (
                        reason is None
                        and positioning_distance < self.positioning_safety_distance_m
                    ):
                        reason = (
                            "critical_positioning_separation_"
                            f"{positioning_distance:.4f}m"
                        )
                    if reason:
                        self._abort(reason)
                    else:
                        self._tick_positioning(states, targets)

            elif self.phase == "pre_run_settle":
                elapsed = time.monotonic() - self.phase_started
                if elapsed >= 1.5:
                    ready, reason = self._infrastructure_ready()
                    if ready:
                        self._start_experiment()
                    elif elapsed > 10.0:
                        self._abort(f"pre_run_timeout_{reason.replace(' ', '_')}")
                    elif time.monotonic() - self.last_discovery_refresh_t >= 1.0:
                        self.app.force_robot_discovery()
                        self.last_discovery_refresh_t = time.monotonic()

            elif self.phase == "running":
                self._update_progress(states, targets)
                if not homography_valid or homography_age > self.app.homography_hold_s:
                    self._abort("homography_lost")
                else:
                    reason = self._network_safety_reason() or self._safety_reason(states)
                    if reason:
                        self._abort(reason)
                    elif time.monotonic() - self.phase_started > self.scenario_timeout_s:
                        self._abort("scenario_timeout")
                    elif stalled_reason := self._stalled_robot_reason(targets):
                        self._abort(stalled_reason)
                    elif all(targets.get(robot_id) is None for robot_id in self.robot_ids):
                        self.phase = "post_run_settle"
                        self.phase_started = time.monotonic()
                        print("[PAPER_RUN] objetivos alcanzados; asentando", flush=True)

            elif self.phase == "post_run_settle":
                if time.monotonic() - self.phase_started >= 1.0:
                    self._complete()
        except Exception as exc:
            self._abort(f"automation_exception_{type(exc).__name__}_{exc}")
            return

        if not self.done:
            self.root.after(100, self._tick)
