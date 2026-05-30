#!/usr/bin/env python3
"""
line_follow.py
==============
Autonomous line-following ROS 2 node with PID control and output smoothing.

Subscribes : /sensor_data/ir_sensors  (std_msgs/Int32)
Publishes  : /cmd_joy  (std_msgs/Float32MultiArray [steering, throttle])

Error scale:
  100  → -2.0  hard left
  110  → -1.0  soft left
  010  →  0.0  center
  011  → +1.0  soft right
  001  → +2.0  hard right
  000  →  last known error (line lost)
  111  →  turn right (junction)

Tuning:
  Kp     — how hard it reacts to current error
  Ki     — corrects persistent drift
  Kd     — dampens oscillation / overshooting
  SMOOTH — output lerp factor (0=instant snap, 1=never moves; 0.2–0.35 recommended)

Run:
  ros2 run line_follower line_follow
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


# ── PID Tuning ───────────────────────────────────────────────
# Start with Kp only, add Kd to reduce wobble, Ki last.
Kp = 0.10    # proportional gain
Ki = 0.00    # integral gain   (keep at 0 until straight-line drift appears)
Kd = 0.05    # derivative gain (dampens oscillation)

# ── Output Smoothing ─────────────────────────────────────────
# Lerp factor applied to steering each cycle.
# Higher = smoother but laggier. Lower = snappier but jerkier.
# Good starting range: 0.20 – 0.35
SMOOTH = 0.25

# ── Speed ────────────────────────────────────────────────────
BASE_SPEED  = 0.40   # throttle when going straight
MIN_SPEED   = 0.20   # minimum throttle in sharp turns
# Reduce MAX_STEER to compensate for increased Arduino STEER_MIX
# If Arduino's STEER_MIX was raised (e.g. 160), halve MAX_STEER so
# the physical differential torque remains similar.
MAX_STEER   = 0.40   # clamp on steering output (was 0.80)

# ── Integral windup limit ────────────────────────────────────
INTEGRAL_MAX = 2.0

# ── IR error map ─────────────────────────────────────────────
IR_ERROR = {
    10:   0.0,    # 010  center
    11:   1.0,    # 011  soft right
    1:    2.0,    # 001  hard right
    110: -1.0,    # 110  soft left
    100: -2.0,    # 100  hard left
}


class LineFollowerPID(Node):

    def __init__(self):
        super().__init__('line_follower_pid')

        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(
            Float32MultiArray, 'cmd_joy', pub_qos)

        self.ir_sub = self.create_subscription(
            Int32, 'sensor_data/ir_sensors',
            self.ir_callback, sub_qos)

        # PID state
        self.prev_error    = 0.0
        self.integral      = 0.0
        self.prev_time     = time.time()

        # Smoothing state — holds the last published steering value
        self.smooth_steer  = 0.0

        # Line-lost state
        self.last_error    = 0.0

        self.get_logger().info(
            f'PID line follower started  '
            f'Kp={Kp} Ki={Ki} Kd={Kd} SMOOTH={SMOOTH}')

    # ── IR callback ──────────────────────────────────────────
    def ir_callback(self, msg: Int32):
        now = time.time()
        dt  = now - self.prev_time
        if dt <= 0:
            dt = 1e-3
        self.prev_time = now

        raw = msg.data

        # ── Junction (111) → turn right ──────────────────────
        if raw == 111:
            self.get_logger().info('Junction — turning right')
            # Drive smoothing toward max steer so junction entry is gradual
            self.smooth_steer += (MAX_STEER - self.smooth_steer) * (1.0 - SMOOTH)
            self.publish(self.smooth_steer, MIN_SPEED)
            return

        # ── Line lost (000) → hold last error, pause integral ─
        if raw == 0:
            error = self.last_error
            self.get_logger().debug(f'Line lost — using last error {error:+.1f}')
        else:
            error = IR_ERROR.get(raw)
            if error is None:
                self.get_logger().warn(f'Unknown IR state: {raw}')
                self.publish(0.0, 0.0)
                return
            self.last_error = error

        # ── PID ──────────────────────────────────────────────

        # Proportional
        P = Kp * error

        # Integral (with windup clamp; paused on line lost to avoid drift)
        if raw != 0:
            self.integral += error * dt
            self.integral  = max(-INTEGRAL_MAX,
                             min( INTEGRAL_MAX, self.integral))
        I = Ki * self.integral

        # Derivative (clamped to prevent spike on line recovery)
        derivative  = (error - self.prev_error) / dt
        derivative  = max(-5.0, min(5.0, derivative))
        D           = Kd * derivative
        self.prev_error = error

        # Raw steering output
        raw_steer = P + I + D
        raw_steer = max(-MAX_STEER, min(MAX_STEER, raw_steer))

        # ── Smooth the steering output ────────────────────────
        # Exponential moving average — removes sudden motor lurches.
        # Formula: smooth += (target - smooth) * (1 - SMOOTH)
        # SMOOTH=0.0 → instant (no filtering)
        # SMOOTH=0.9 → very slow to respond
        self.smooth_steer += (raw_steer - self.smooth_steer) * (1.0 - SMOOTH)
        steering = self.smooth_steer

        # ── Throttle — reduce speed with steering effort ──────
        throttle = BASE_SPEED - (abs(steering) * (BASE_SPEED - MIN_SPEED))
        throttle = max(MIN_SPEED, throttle)

        self.get_logger().debug(
            f'IR:{raw:03d}  err:{error:+.1f}  '
            f'P:{P:+.3f} I:{I:+.3f} D:{D:+.3f}  '
            f'raw_steer:{raw_steer:+.3f}  '
            f'smooth_steer:{steering:+.3f}  '
            f'throttle:{throttle:.3f}')

        self.publish(steering, throttle)

    # ── Helpers ──────────────────────────────────────────────
    def reset_pid(self):
        """Call this if you change tracks or after a manual override."""
        self.integral      = 0.0
        self.prev_error    = 0.0
        self.smooth_steer  = 0.0
        self.prev_time     = time.time()

    def publish(self, steering: float, throttle: float):
        # Arduino cmd_callback expects [steering, throttle]
        # and does its own differential drive mixing.
        msg = Float32MultiArray()
        msg.data = [float(steering), float(throttle)]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)   # stop motors
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()