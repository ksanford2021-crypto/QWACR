# Navigation Autonomy Testing Guide

**Date:** March 26, 2026  
**Status:** Updated for non-RTK GNSS + staged Nav2 rollout  
**Scope:** Safe progression from fusion validation to waypoint missions

---

## 1. Preconditions

Before any autonomous motion:
- `~/qwacr_ws` built and sourced.
- Motor control stack validated manually.
- Sensor fusion stages passed per `SENSOR_FUSION_EKF_TESTING_GUIDE.md`.
- Open, controlled test area with E-stop readiness.

Required minimum fusion state:
- Local EKF stable (`/odometry/local`).
- Global EKF stable (`/odometry/global`) under non-RTK GNSS.
- GNSS yaw axis validated if GNSS yaw is fused.

---

## 2. Stage 1 — Nav2 Stack Bringup (No Motion Validation)

Goal: Verify planning and costmaps before real movement.

1. Launch localization stack.
2. Launch Nav2 test bringup.
3. In RViz, verify:
   - Fixed frame `map`
   - Global and local costmaps updating
   - Robot pose is reasonable and not jumping
4. Send short goals in RViz while keeping motors disabled (or robot safely constrained).

Checks:
```bash
ros2 topic echo /cmd_vel --once
ros2 topic hz /odometry/global
```

Pass criteria:
- Planner generates feasible paths.
- Controller outputs sane velocity commands.
- No repeated planner/controller failures at short ranges.

---

## 3. Stage 2 — Short-Range Autonomous Motion (0.5–2 m)

Goal: Validate closed-loop motion at low risk.

1. Enable motor stack.
2. Command small goals in open space.
3. Repeat 10+ trials across headings.

Pass criteria:
- Robot reaches goals without oscillatory runaway.
- Stops near goals consistently.
- Heading at goal is reasonable (consider non-RTK limits).

If poor behavior:
- Check global EKF stability first.
- Reduce controller aggressiveness in Nav2.
- Re-check GNSS yaw transform if heading bias appears.

---

## 4. Stage 3 — Obstacle Interaction & Recovery Behavior

Goal: Ensure safe path replanning and recoveries.

1. Place known obstacles along planned routes.
2. Send goals that require detours.
3. Observe behavior tree transitions and recovery actions.

Pass criteria:
- Costmaps detect obstacles reliably.
- Robot replans around obstacles.
- Recovery behaviors trigger only when needed.
- No unsafe spinning/drive commands.

---

## 5. Stage 4 — Outdoor Waypoint Trials (Non-RTK)

Goal: Execute realistic mission sequences with current GNSS mode.

1. Use conservative waypoint spacing first (5–15 m).
2. Run waypoint follower and monitor path tracking.
3. Increase route complexity only after consistent passes.

Important for non-RTK:
- Expect larger absolute position error than RTK.
- Use wider acceptance radius initially.
- Evaluate repeatability, not cm-level endpoint precision.

Suggested acceptance thresholds (starting point):
- Cross-track error generally bounded and non-divergent.
- Endpoint tolerance on the order of a few meters (site dependent).
- No persistent heading offset indicating wrong GNSS yaw transform.

---

## 6. Stage 5 — Mission-Flow Integration (Operator Driven)

Goal: Validate end-to-end mission sequence with operator controls.

Sequence:
1. System starts in `idle`.
2. Operator selects mission mode (`manual`, `waypoint:*`, `rtb`, `loiter`).
3. Robot transitions safely and executes behavior.
4. Operator can override to `manual` or `estop` at any time.

Pass criteria:
- Mode transitions are deterministic.
- Autonomous motion only occurs in appropriate modes.
- Manual override preempts autonomy immediately.

---

## 7. Logging Requirements

Record bags for all Stage 2+ tests:

```bash
mkdir -p ~/nav2_logs
cd ~/nav2_logs

ros2 bag record -o nav_stage_run_$(date +%Y%m%d_%H%M%S) \
  /cmd_vel \
  /diff_cont/odom \
  /odometry/local \
  /odometry/global \
  /gps/enu_odom \
  /imu/data \
  /slamware_ros_sdk_server_node/scan \
  /tf /tf_static
```

Post-run review focus:
- Fusion stability before/during motion.
- Commanded vs actual motion.
- Heading consistency during turns.
- Recovery behavior quality.

---

## 8. Exit Criteria for Production-Like Trials

Move to larger, mission-like runs only when all are true:
- [ ] Repeated Stage 2 passes with no unsafe events.
- [ ] Stage 3 obstacle/recovery behavior validated.
- [ ] Stage 4 waypoint runs repeatable within non-RTK tolerance.
- [ ] Stage 5 mission-mode transitions verified with operator-in-the-loop.

If any item fails, return to the previous stage and retune before proceeding.
