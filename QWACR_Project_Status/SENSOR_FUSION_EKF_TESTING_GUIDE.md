# Sensor Fusion EKF Testing Guide

**Date:** March 26, 2026  
**Status:** Updated for non-RTK GNSS operations  
**Scope:** Dual-EKF validation before autonomous waypoint missions

---

## 1. Fusion Architecture (Current Plan)

### Local EKF (`/odometry/local`, world frame = `odom`)
Use for smooth short-horizon motion tracking:
- Wheel odometry
- IMU angular velocity + linear acceleration

Purpose:
- Smooth, continuous estimate for control loops.
- No absolute GNSS jumps.

### Global EKF (`/odometry/global`, world frame = `map`)
Use for global localization and Nav2:
- Wheel odometry
- IMU
- GNSS ENU odometry (`/gps/enu_odom`)

Purpose:
- Bound long-term drift with GNSS corrections.
- Provide map-frame pose for waypoint/autonomy.

---

## 2. Non-RTK GNSS Implications

Because RTK corrections are not currently used:
- Position uncertainty is larger and more variable.
- Heading quality can degrade at low speed or poor signal geometry.
- Global EKF must trust GNSS less aggressively than RTK assumptions.

### Practical tuning rules
- Keep local EKF unchanged as primary smooth source.
- In global EKF, increase process noise in x/y and yaw to avoid rigid lock to noisy GNSS.
- Start with conservative GNSS covariance and decrease only if behavior is stable.
- Never fuse GNSS yaw until yaw axis alignment is validated (Section 4).

---

## 3. Stage A — Local EKF Verification (No GNSS dependency)

Goal: Confirm local odometry is smooth, stable, and physically consistent.

1. Launch IMU + wheel odom + local EKF.
2. Keep robot stationary for 60 s.
3. Drive short straight lines and figure-8 manually.

Check:
```bash
ros2 topic hz /odometry/local
ros2 topic echo /odometry/local --once
```

Success criteria:
- `/odometry/local` stable at expected EKF frequency.
- Stationary drift is low and does not diverge.
- Forward motion primarily increases x in `base_link` heading direction.

---

## 4. Stage B — GNSS Yaw Axis Validation (+/- 90° check)

Your dual-antenna baseline is mounted side-to-side (vehicle Y-axis), so GNSS heading may not be aligned with vehicle forward (+X).

Test method:
1. Drive straight forward in open sky.
2. Compare GNSS heading vs. track direction from position change.
3. If consistent 90° offset exists, apply fixed transform in GNSS heading pipeline:
   - `yaw_base = yaw_gnss - pi/2` or
   - `yaw_base = yaw_gnss + pi/2`
4. Choose sign that makes forward drive align with forward yaw.

Acceptance criteria:
- No persistent ±90° heading bias during straight runs.
- Yaw increases/decreases with expected turn direction.

---

## 5. Stage C — Global EKF with Non-RTK GNSS (Position only first)

Goal: Verify GNSS corrections improve long-term drift without oscillation.

1. Run local + global EKF.
2. Fuse GNSS position and velocity first.
3. Keep GNSS yaw disabled if alignment/quality is not yet validated.

Check:
```bash
ros2 topic hz /odometry/global
ros2 topic echo /odometry/global --once
```

Field behavior to verify:
- Global pose slowly corrects drift; it does not jump erratically.
- Local and global odometry stay close over short windows.
- During brief GNSS degradation, estimate remains controllable.

If unstable:
- Increase GNSS covariance (or reduce trust in GNSS heading).
- Increase map-frame process noise slightly rather than forcing hard corrections.

---

## 6. Stage D — Add GNSS Yaw to Global EKF (Only after Stage B/C pass)

Goal: Improve global heading without introducing yaw snaps.

1. Enable GNSS yaw input to global EKF.
2. Start with conservative yaw covariance.
3. Test straight-line and repeated left/right turns.

Success criteria:
- No heading discontinuities or 180° flips.
- Nav2 path following improves or remains stable.
- Robot heading in RViz aligns with real vehicle heading.

---

## 7. Recommended Test Data Capture

Record bags for each stage:

```bash
mkdir -p ~/ekf_logs
cd ~/ekf_logs

ros2 bag record -o ekf_stage_run_$(date +%Y%m%d_%H%M%S) \
  /odometry/local \
  /odometry/global \
  /gps/enu_odom \
  /imu/data \
  /odometry/wheel \
  /tf /tf_static
```

Review:
- Pose continuity
- Yaw consistency
- GNSS correction smoothness

---

## 8. Go/No-Go for Autonomy Testing

Proceed to autonomy and waypoint tests only when:
- [ ] Stage A passes (local EKF stable)
- [ ] Stage B passes (GNSS yaw axis validated)
- [ ] Stage C passes (non-RTK global EKF stable)
- [ ] Stage D passes (GNSS yaw fusion stable, if enabled)

If any stage fails, fix fusion behavior before Nav2 tuning.
