#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from vision_msgs.msg import BoundingBoxArray


class AutoGate(Node):
    def __init__(self):
        super().__init__('auto_gate')

        # ===== Mission States =====
        self.state = "DESCEND"
        self.target_depth = -1.85
        self.tolerance = 0.1
        self.current_depth = 0.0
        self.gate_detected = False
        self.gate_box = None
        self.had_gate = False

        # ===== Tuning: vertical control =====
        self.DESCEND_SPEED = -0.5   # m/s (negative = down in ENU)
        self.ASCEND_SPEED  =  0.5   # m/s
        self.ALIGN_Z_STEP  =  0.5   # m/s when nudging depth during alignment
        self.MAX_Z_RATE    =  0.6   # clamp vertical cmd

        # ===== Tuning: forward control (tiered by gate size) =====
        # Use the larger of (w, h) as a "size proxy" in normalized image units.
        self.FWD_TIER = [
            (0.15, 0.25),  # size < 0.15 -> 0.25 m/s (far)
            (0.25, 0.40),  # 0.15–0.25 -> 0.40 m/s (approaching)
            (0.40, 0.60),  # 0.25–0.40 -> 0.60 m/s (close)
            (1.01, 0.80),  # >=0.40     -> 0.80 m/s (very close, commit)
        ]
        self.MAX_X_RATE = 0.9   # clamp forward cmd (absolute speed cap)

        # ===== Camera center calibration (from your tests) =====
        self.IDEAL_X_CENTER = 0.33
        self.IDEAL_Y_CENTER = 0.34
        self.TOL_X = 0.005
        self.TOL_Y = 0.005

        # ===== Internal slew state =====
        self._z_prev = 0.0
        self._x_prev = 0.0

        # ===== ROS wiring =====
        self.pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )
        self.sub_depth = self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self.depth_callback, 10
        )
        self.sub_gate = self.create_subscription(
            BoundingBoxArray, '/main_camera/detection/bounding_boxes',
            self.gate_callback, 10
        )

        time.sleep(1.0)  # give MAVROS a moment to connect
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self._best_w = 0.0
        self._stable_counter = 0
        self.WIDTH_STABLE_FRAMES = 7   # how many frames to wait
        self.WIDTH_DELTA = 0.01        # min change to consider as "growth"
        self._last_gate_stamp = time.time()
        self.width_height_ratio = 0.55

    # ---------- helpers ----------
    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _slew(self, prev, target, max_step):
        """Limit the step between previous and target per control tick."""
        step = self._clamp(target - prev, -max_step, max_step)
        return prev + step

    def _slew_z(self, target, step=0.2):
        self._z_prev = self._slew(self._z_prev, target, step)
        return self._clamp(self._z_prev, -self.MAX_Z_RATE, self.MAX_Z_RATE)

    def _slew_x(self, target, step=0.2):
        self._x_prev = self._slew(self._x_prev, target, step)
        return self._clamp(self._x_prev, 0.0, self.MAX_X_RATE)

    def _forward_speed_from_size(self, w, h):
        size = max(w, h)
        for threshold, speed in self.FWD_TIER:
            if size < threshold:
                return speed
        return self.FWD_TIER[-1][1]

    # ---------- callbacks ----------
    def depth_callback(self, msg: Float64):
        self.current_depth = msg.data

    def gate_callback(self, msg):
        self.gate_detected = False
        self.gate_box = None
        for box in msg.bounding_boxes:
            if box.label_name == "gate":
                self.gate_detected = True
                self.gate_box = box
                self._last_gate_stamp = time.time()
                break

    # ---------- FSM ----------
    def control_loop(self):
        cmd = Twist()

        # 1) DESCEND to target depth
        if self.state == "DESCEND":
            if abs(self.current_depth - self.target_depth) < self.tolerance:
                cmd.linear.z = 0.0
                self.state = "SEARCH"
                self.get_logger().info("[DESCEND] Target depth reached → SEARCHING...")
            elif self.current_depth > self.target_depth + self.tolerance:
                cmd.linear.z = self._slew_z(self.DESCEND_SPEED)
                self.get_logger().info("[DESCEND] Moving DOWN fast")
            elif self.current_depth < self.target_depth - self.tolerance:
                cmd.linear.z = self._slew_z(self.ASCEND_SPEED)
                self.get_logger().info("[DESCEND] Moving UP")

        # 2) SEARCH: rotate until gate is seen & centered
        elif self.state == "SEARCH":  # Step 1: strafe to center the gate
            if self.gate_detected and self.gate_box:
                cx, cy, w, h = self.gate_box.x, self.gate_box.y, self.gate_box.w, self.gate_box.h
                ex = cx - self.IDEAL_X_CENTER
                ey = cy - self.IDEAL_Y_CENTER

                centered = abs(ex) < self.TOL_X # and abs(ey) < self.TOL_Y
                enough_width = abs((w/h) - self.width_height_ratio) < 0.07

                if centered and not enough_width:
                    self.state = "ALIGN"
                    self._best_w = self.gate_box.w
                    self._stable_counter = 0
                    self.get_logger().info("[SEARCH] Gate centered → ALIGN")
                elif centered and enough_width:
                    self.state = "FORWARD"
                    self.get_logger().info(f"[Search] Width stabilized at center: {ex, ey} and width:{w:.3f}, centered → FORWARD")
                else:
                    # Strafe left/right for horizontal alignment
                    if ex > 0:
                        cmd.linear.y = -1 * max((1.5 * abs(ex)), 0.07) # move left
                    else:
                        cmd.linear.y = max((1.5 *abs(ex)), 0.07) # move right

                    self.get_logger().info(f"[SEARCH] Strafing to center (ex={ex:.3f}, ey={ey:.3f})")
            else:
                if time.time() - self._last_gate_stamp > 1.0:
                    cmd.angular.z = 0.5
                    self.get_logger().info("[SEARCH] Looking for gate...")


        elif self.state == "ALIGN":  # Step 2: rotate to maximize width
            if self.gate_detected and self.gate_box:
                cx, cy, w, h = self.gate_box.x, self.gate_box.y, self.gate_box.w, self.gate_box.h

                # Track width growth
                if w > self._best_w + self.WIDTH_DELTA:
                    self._best_w = w
                    self._stable_counter = 0
                else:
                    self._stable_counter += 1

                if self._stable_counter > self.WIDTH_STABLE_FRAMES:
                    # Once width stable, check if still centered
                    ex = cx - self.IDEAL_X_CENTER
                    ey = cy - self.IDEAL_Y_CENTER
                    centered = abs(ex) < self.TOL_X and abs(ey) < self.TOL_Y

                    self.state = "SEARCH"
                    self.get_logger().info("[ALIGN] Width stable but not centered → back to SEARCH")
                else:
                    # Rotate to adjust yaw, NO strafing here
                    cmd.angular.z = 0.2

                    self.get_logger().info(f"[ALIGN] Rotating to maximize width (w={w:.3f}, best={self._best_w:.3f}, ratio:{w/h} stable={self._stable_counter})")
            else:
                self.state = "SEARCH"
                self.get_logger().info("[ALIGN] Lost gate → back to SEARCH")


        # 3) FORWARD: drive forward with speed scaling by gate size; stop when gate disappears
        elif self.state == "FORWARD":
            if self.gate_detected and self.gate_box:
                cx, cy, w, h = self.gate_box.x, self.gate_box.y, self.gate_box.w, self.gate_box.h
                ex = cx - self.IDEAL_X_CENTER
                ey = cy - self.IDEAL_Y_CENTER

                cmd.linear.x = 1.0
                if ex > 0:
                    cmd.linear.y = ex * 5.0
                else:
                    cmd.linear.y = -ex * 5.0
            else:
                if time.time() - self._last_gate_stamp > 1.0:
                    cmd.linear.x = 2.0 #pass through the gate


        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
