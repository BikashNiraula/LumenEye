#!/usr/bin/env python3
"""
Box Mover — RViz version (no Gazebo).
Publishes box position to /lumeneye/box_pose and /lumeneye/box_state only.

CLI control via:
  ros2 topic pub --once /lumeneye/box_cmd std_msgs/msg/String "data: 'goto 0.4 0.1'"

Supported commands:
  goto <x> <y>       — move box to absolute x,y position
  scenario <n>       — jump to scenario index 0-4
  next               — advance to next scenario
  stop               — freeze box in place
  resume             — resume auto sequence
  reset              — go back to scenario 0
  pos                — print current position (check terminal output)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String

BOX_Z = 0.09

def eq(r, p, y):
    cr,sr = math.cos(r/2), math.sin(r/2)
    cp,sp = math.cos(p/2), math.sin(p/2)
    cy,sy = math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)

SCENARIOS = [
    (0.35,  0.38, 0.35, 0.01, eq(0,0,0),      'Flat from +Y'),
    (0.35, -0.38, 0.35,-0.01, eq(0.40,0,0),   'Tilted 23deg from -Y'),
    (0.54,  0.28, 0.35, 0.01, eq(0,0,0.65),   'Diagonal 37deg yaw'),
    (0.35,  0.38, 0.35, 0.01, eq(0,0.30,0),   'Pitched 17deg fwd'),
    (0.14,  0.02, 0.34, 0.00, eq(0.25,0,0.25),'Combined tilt -X'),
]
SW = 0.018
RT = 0.12

class BoxMover(Node):
    def __init__(self):
        super().__init__('box_mover')
        self.allow = False
        self.state = 'SWEEP'
        self.si = 0
        self.sc = SCENARIOS[0]
        self.cx = self.sc[0]
        self.cy = self.sc[1]

        # CLI override state
        self.frozen = False          # True = stop auto sequence, hold position
        self.cli_target = None       # (x, y) manual goto target, or None

        self.pos_pub = self.create_publisher(Point,  '/lumeneye/box_pose',  10)
        self.st_pub  = self.create_publisher(String, '/lumeneye/box_state', 10)
        self.create_subscription(Bool,   '/lumeneye/plate_allow', self._al,  10)
        self.create_subscription(String, '/lumeneye/box_cmd',     self._cmd, 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info('Box mover ready (RViz mode)')
        self._print_help()

    # ── helpers ────────────────────────────────────────────────────────
    def _print_help(self):
        self.get_logger().info('')
        self.get_logger().info('┌─ CLI commands via /lumeneye/box_cmd ────────────────────┐')
        self.get_logger().info('│  goto <x> <y>   — move box to absolute position        │')
        self.get_logger().info('│  scenario <n>   — jump to scenario 0-4                 │')
        self.get_logger().info('│  next           — advance to next scenario              │')
        self.get_logger().info('│  stop           — freeze box in place                  │')
        self.get_logger().info('│  resume         — resume auto sequence                 │')
        self.get_logger().info('│  reset          — restart from scenario 0              │')
        self.get_logger().info('│  pos            — print current position               │')
        self.get_logger().info('│  help           — show this message                    │')
        self.get_logger().info('└─────────────────────────────────────────────────────────┘')
        self.get_logger().info('')
        self.get_logger().info('Example:')
        self.get_logger().info('  ros2 topic pub --once /lumeneye/box_cmd '
                               'std_msgs/msg/String "data: \'goto 0.4 0.1\'"')
        self.get_logger().info('')

    def _al(self, m):
        was = self.allow
        self.allow = m.data
        if m.data and not was:
            self.get_logger().info('[BOX] plate_allow=TRUE → sweeping')
        elif not m.data and was:
            self.get_logger().info('[BOX] plate_allow=FALSE → holding')

    def _load(self, i):
        self.sc = SCENARIOS[i]
        self.cx = self.sc[0]
        self.cy = self.sc[1]
        self.state = 'SWEEP'
        self.get_logger().info(f'[BOX] Scenario {i}: {self.sc[5]}')

    # ── CLI command handler ─────────────────────────────────────────────
    def _cmd(self, m):
        raw = m.data.strip()
        parts = raw.split()
        if not parts:
            return
        cmd = parts[0].lower()

        if cmd == 'help':
            self._print_help()

        elif cmd == 'pos':
            self.get_logger().info(
                f'[BOX] Current position: x={self.cx:.3f} y={self.cy:.3f} z={BOX_Z:.3f}'
                f'  state={self.state}  frozen={self.frozen}  scenario={self.si}')

        elif cmd == 'stop':
            self.frozen = True
            self.cli_target = None
            self.get_logger().info(
                f'[BOX] STOPPED at ({self.cx:.3f}, {self.cy:.3f})')

        elif cmd == 'resume':
            self.frozen = False
            self.cli_target = None
            self.get_logger().info('[BOX] RESUMED auto sequence')

        elif cmd == 'reset':
            self.frozen = False
            self.cli_target = None
            self.si = 0
            self._load(0)
            self.get_logger().info('[BOX] RESET to scenario 0')

        elif cmd == 'next':
            self.frozen = False
            self.cli_target = None
            nxt = (self.si + 1) % len(SCENARIOS)
            self.si = nxt
            self._load(nxt)
            self.get_logger().info(f'[BOX] Advanced to scenario {nxt}: {SCENARIOS[nxt][5]}')

        elif cmd == 'scenario':
            if len(parts) < 2:
                self.get_logger().warn('[BOX] Usage: scenario <n>  (0-4)')
                return
            try:
                n = int(parts[1])
                if not 0 <= n < len(SCENARIOS):
                    self.get_logger().warn(
                        f'[BOX] scenario index must be 0-{len(SCENARIOS)-1}')
                    return
                self.frozen = False
                self.cli_target = None
                self.si = n
                self._load(n)
            except ValueError:
                self.get_logger().warn('[BOX] scenario index must be an integer')

        elif cmd == 'goto':
            if len(parts) < 3:
                self.get_logger().warn('[BOX] Usage: goto <x> <y>')
                return
            try:
                tx, ty = float(parts[1]), float(parts[2])
                self.frozen = False          # unfreeze so _tick drives to target
                self.cli_target = (tx, ty)
                self.get_logger().info(
                    f'[BOX] GOTO ({tx:.3f}, {ty:.3f}) — '
                    f'moving from ({self.cx:.3f}, {self.cy:.3f})')
            except ValueError:
                self.get_logger().warn('[BOX] goto: x and y must be numbers')

        else:
            self.get_logger().warn(
                f'[BOX] Unknown command: "{raw}"  — send "help" for usage')

    # ── main tick ───────────────────────────────────────────────────────
    def _tick(self):

        # ── CLI goto: drive toward manual target ──
        if self.cli_target is not None:
            tx, ty = self.cli_target
            dx, dy = tx - self.cx, ty - self.cy
            d = math.sqrt(dx*dx + dy*dy)
            if d > SW:
                self.cx += dx/d * SW
                self.cy += dy/d * SW
                self.st_pub.publish(String(data='MOVING'))
            else:
                self.cx, self.cy = tx, ty
                self.cli_target = None
                self.frozen = True           # hold at destination once arrived
                self.get_logger().info(
                    f'[BOX] Arrived at ({tx:.3f}, {ty:.3f}) — frozen (send "resume" to continue)')
                self.st_pub.publish(String(data='BLOCKING'))
            self.pos_pub.publish(Point(x=self.cx, y=self.cy, z=BOX_Z))
            return

        # ── frozen: publish position but don't move ──
        if self.frozen:
            self.st_pub.publish(String(data='BLOCKING'))
            self.pos_pub.publish(Point(x=self.cx, y=self.cy, z=BOX_Z))
            return

        # ── original auto-sequence logic (unchanged) ──
        sc = self.sc
        sx, sy, ex, ey = sc[0], sc[1], sc[2], sc[3]

        if not self.allow:
            st = 'BLOCKING' if self.state == 'AT' else 'MOVING'
            self.st_pub.publish(String(data=st))
            self.pos_pub.publish(Point(x=self.cx, y=self.cy, z=BOX_Z))
            return

        if self.state == 'SWEEP':
            dx, dy = ex - self.cx, ey - self.cy
            d = math.sqrt(dx*dx + dy*dy)
            if d > SW:
                self.cx += dx/d * SW
                self.cy += dy/d * SW
            else:
                self.cx, self.cy = ex, ey
                self.state = 'AT'
                self.get_logger().warn(f'[BOX] BLOCKING at ({ex:.2f},{ey:.2f})')
            self.st_pub.publish(String(data='MOVING'))

        elif self.state == 'AT':
            self.st_pub.publish(String(data='BLOCKING'))

        elif self.state == 'BACK':
            dx, dy = sx - self.cx, sy - self.cy
            d = math.sqrt(dx*dx + dy*dy)
            if d > RT:
                self.cx += dx/d * RT
                self.cy += dy/d * RT
            else:
                self.cx, self.cy = sx, sy
                nxt = (self.si + 1) % len(SCENARIOS)
                self.si = nxt
                self._load(nxt)
            self.st_pub.publish(String(data='MOVING'))

        self.pos_pub.publish(Point(x=self.cx, y=self.cy, z=BOX_Z))


def main(args=None):
    rclpy.init(args=args)
    n = BoxMover()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()