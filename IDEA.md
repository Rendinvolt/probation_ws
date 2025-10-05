# Robot Auto Gate (ROS 2) — Mecatron Probation Task

**Goal:** Go through a designated gate

This IDEA documents the **thought process**, the **trial‑and‑error changes**, and the **final finite state machine (FSM)** implemented in the provided `robot_auto_gate` node.

---

## What each function does

1. **DESCEND** to the gate's depth (e.g., −1.85 m in ENU) with slew‑limited vertical velocity.
2. **SEARCH**: *strafe‑only* (no yaw) to center the gate in the camera image without changing its apparent width.
3. **ALIGN**: *yaw‑only* (no strafe) to maximize gate width (i.e., face the gate more orthogonally).
4. **FORWARD**: drive through the gate; continue minor center corrections while the gate is visible; if the gate disappears for long enough, commit and pass through.

This two‑step visual alignment (“center” then “maximize width”) proved robust in sim and avoids the common pitfall of mixing yaw and strafe at the same time, which makes the bounding‑box width unstable.

---

## System & Topics

- **Framework:** ROS 2 (rclpy)
- **Autopilot bridge:** MAVROS
- **Publisher (cmd):** `/mavros/setpoint_velocity/cmd_vel_unstamped` (`geometry_msgs/Twist`)
- **Subscribers:**
  - `/mavros/global_position/rel_alt` (`std_msgs/Float64`) — used as **depth** (note: negative target → down in ENU)
  - `/main_camera/detection/bounding_boxes` (`vision_msgs/BoundingBoxArray`) — YOLO detections (expects boxes with fields `label_name`, `x`, `y`, `w`, `h`, normalized to [0,1])

> **Mode & arming (pre‑flight):** the robot should be in **GUIDED MODE** for autonomous control via topic.

---

## Coordinate & Conventions

- **ENU body frame** (as used by MAVROS setpoint velocity):
  - `+x` → forward
  - `+y` → left (strafing left); `−y` → right
  - `+z` → up; `−z` → down
  - `angular.z` → yaw CCW (left)
- **Image coordinates (normalized to [0,1])**:
  - `(x, y)` = coordinate of the gate's center
  - `(w, h)` = coordinate of the gate's width and height respectively.
  - Calibrated “ideal” image center stored as `IDEAL_X_CENTER`, `IDEAL_Y_CENTER`
  > The Ideal X and Y coordinate of the gate's center is achieved through echoing related topic.

---

## Final Finite State Machine (FSM)

```text
            +-------------------+
            |     DESCEND       |
            |  (to target z)    |
            +----------+--------+
                       |
                       v
            +----------+--------+
            |   SEARCH (strafe) |
            | center bbox (cx)  |
            +----------+--------+
                       |
                       v
            +----------+--------+
            |  ALIGN (yaw-only) |
            | maximize width (w) |
            +----------+--------+
                       |
                       v
            +----------+--------+
            |    FORWARD        |
            | drive through     |
            +-------------------+
```

### Why split SEARCH and ALIGN?

Early versions tried to **rotate while centering**. That changes the gate’s apparent width even when you’re doing the correct thing, so any “maximize width” logic becomes noisy. By **strafe‑only** in SEARCH, width stays roughly constant while you center `cx`. Then **yaw‑only** in ALIGN changes width significantly and monotonically near the optimum, making width maximization reliable.

---

## Key Design Choices (and the trial‑and‑error that led to them)

### 1) **Guided/Offboard control** + Slew limits
- We publish body‑frame velocity setpoints through MAVROS. Slew limiting (`_slew_*`) prevents step changes that can trip attitude/velocity limiters or cause oscillations.

### 2) **Depth sign & target**
- Using `/mavros/global_position/rel_alt` as depth proxy (Float64). In our sim, a **more negative** value means “deeper.” We set `target_depth = -1.85` and a small deadband `tolerance = 0.1`.
- Early tests revealed confusing sign conventions; logging helped confirm whether “down” was negative. Final code uses `DESCEND_SPEED = -0.5` (**negative z** → down).

### 3) **Don’t block the timer callback**
- A common mistake was attempting timed forward motion inside a `while` loop (blocking). ROS 2 timers must remain **non‑blocking**; otherwise no commands are published during the block. All timed transitions are done with timestamps and per‑tick checks.

### 4) **Searcher stability (gate flicker)**
- Object detectors occasionally drop frames; commanding “missing” on a single bad frame causes chattering.
- We cache the **last detection time** (`_last_gate_stamp`) and only rotate to search if the gate has been missing for > 1.0 s. This basically fixes the glitching.

### 5) **Separate objectives per state**
- **SEARCH:** *strafe‑only*, no yaw. This keeps `w` stable while we push `cx → IDEAL_X_CENTER`. Adjusting `z` here is okay because vertical does not affect width strongly.
- **ALIGN:** *yaw‑only*, no strafe. This makes width a monotonic cue for “facing the plane.” We track the **best observed width** and a **stability counter** to detect plateau.
- **FORWARD:** drive forward; minor lateral nudges to keep centered if gate is still visible. If the gate disappears long enough (based on `_last_gate_stamp`), we commit through.

### 6) **Speed scaling by perceived size**
- Forward speed uses a small tiered table (`FWD_TIER`) based on `max(w,h)` as a crude range proxy: farther → slower, nearer → faster.
- We clamp absolute velocities (`MAX_X_RATE`, `MAX_Z_RATE`) to respect the vehicle’s limits.

### 7) **Tolerances & ratios**
- The search center tolerances are tight in the final code (`TOL_X = TOL_Y = 0.005`) based on empirical calibration.
- A `width_height_ratio` (e.g., 0.55) is used in SEARCH as a “good enough” heuristic to skip ALIGN when the gate already looks near‑orthogonal **at center**. At first, I tried to compare the current coordinate directly to the pre-determined center coordinate. However, when running the script in the simulation, I realize that as the robot moved forward or backward, the center coordinate changed. So I came with an idea to calculate the width and height ratio 🙂.

> **Some things that I learned:** 
>- Mixing yaw and strafe produced unstable width signals
>- Blocking loops prevented messages from being published 
>- Because the problem states that objects are only detected approximately 70% of the time, I tried to add some detection persistence.

---

## Parameters & What to Tune

| Parameter | Purpose | Typical Value | Notes |
|---|---|---:|---|
| `target_depth` | Desired depth (m, ENU) | `-1.85` | Negative is deeper in this setup |
| `tolerance` | Depth deadband | `0.1` | Don’t chase noise |
| `DESCEND_SPEED` | z‑vel down | `-0.5` | Negative in ENU → down |
| `ASCEND_SPEED` | z‑vel up | `+0.5` | |
| `ALIGN_Z_STEP` | z “nudge” while centering | `0.5` | Slew‑limited |
| `MAX_Z_RATE` | z‑vel clamp | `0.6` | Safety |
| `FWD_TIER` | Forward speed vs. size | see code | Tune per vehicle |
| `MAX_X_RATE` | Forward clamp | `0.9` | |
| `IDEAL_X_CENTER` | Calibrated cx | `0.33` | From your camera |
| `IDEAL_Y_CENTER` | Calibrated cy | `0.34` | |
| `TOL_X`, `TOL_Y` | Center tolerances | `0.005` | Tight; relax if chattery |
| `WIDTH_STABLE_FRAMES` | Width plateau frames | `7` | ~0.7 s at 10 Hz |
| `WIDTH_DELTA` | Minimal width growth | `0.01` | Hysteresis vs noise |
| `width_height_ratio` | “Enough width” heuristic | `0.55` | Skip ALIGN if the ratio is wide enough for the robot to pass through |

---

## State Logic (annotated excerpts)

### DESCEND
- Move down/up until within `tolerance` of `target_depth`. Slew‑limited to avoid jerks.

### SEARCH (strafe‑only)
```python
cx, cy, w, h = box.x, box.y, box.w, box.h
ex = cx - IDEAL_X_CENTER
# Center check ignores ey by design (vertical handled via depth)
centered = abs(ex) < TOL_X

if centered and not enough_width:
    state = "ALIGN"
elif centered and enough_width:
    state = "FORWARD"
else:
    # Strafe to reduce ex, do NOT rotate
    cmd.linear.y = k * sign(-ex)
```

- If the gate isn’t seen for > 1.0 s, rotate slowly to scan.
- The search state will check  if the gate is **CENTERED** and **WIDTH AND HEIGHT RATIO** is good enough.
- If the width and height ratio still needs some improvement, it will go to **ALIGN** state.

### ALIGN (yaw‑only)
```python
# Track best width and plateau
if w > best_w + WIDTH_DELTA:
    best_w = w
    stable = 0
else:
    stable += 1

if stable > WIDTH_STABLE_FRAMES:
    # Re‑check centering; if drifted, go back to SEARCH
    state = "SEARCH"
else:
    # Yaw only towards center, no strafe
    cmd.angular.z = k * (IDEAL_X_CENTER - cx)
```

### FORWARD
- Drive forward; optionally keep small lateral correction using cx.
- If gate disappears for > 1.0 s, increase forward speed briefly to commit through.

---

## Common Pitfalls & Fixes

- **“Robot never moved in FORWARD”** → A blocking `while` loop inside the timer prevented publishing. Use timestamp‑based non‑blocking checks.
- **“Gate missing even though visible”** → Detector flicker. Cache `_last_gate_stamp` and require a sustained miss before declaring “search.”
- **“Oscillation near center”** → Tolerances too tight. Increase `TOL_X/TOL_Y` or apply proportional gains with clamping on strafe/yaw commands.
- **“Depth jumps”** → Slew limits on z were too loose or sign wrong. Confirm ENU signs and clamp with `MAX_Z_RATE`.

---

## How The Script is "Debugged"

1. Connect the script with the unity sim : `ros2 run ros_tcp_endpoint default_server_endpoint`
2. Connect with Foxglove : `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
3. Change the state of the vehicle in Foxglove to **Guided** mode.
3. Make sure that the simulator is running with the robot's Point of View(PoV) by clicking `Tab` on the keyboard (so that the gate's bounding-box is visible)
4. Launch the script:
   ```bash
   ros2 run robot_auto_gate velocity_publisher.py
   ```
5. Watch the logs to verify the FSM transitions: `DESCEND → SEARCH → ALIGN → FORWARD`.

