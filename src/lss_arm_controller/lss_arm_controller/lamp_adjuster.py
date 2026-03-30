#!/usr/bin/env python3
"""
LumenEye — Optimized Lamp Adjuster with Beam Alignment
=======================================================
Optimizer balances:
  - Ray clearance from box         (maximize)
  - Joint delta from current pose  (minimize)
  - Future box escape margin       (maximize)
  - Beam alignment to tool tip     (maximize) ← NEW
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

TOOL_TIP    = np.array([0.35, 0.0, 0.012])
BOX_HALF    = np.array([0.06, 0.06, 0.02])
JOINT_NAMES = ['lss_arm_joint_1','lss_arm_joint_2',
               'lss_arm_joint_3','lss_arm_joint_4']
CONFIRM_N   = 3
CHECK_DT    = 0.3

# Cost weights
W_CLEARANCE  = 2.0
W_JOINT_MOVE = 0.8
W_FUTURE     = 1.2
W_ALIGNMENT  = 3.0   # strongly reward beam pointing at tip

# Arm shapes: [j2, j3] only — j4 is solved analytically for alignment
ARM_SHAPES = [
    [1.00, -0.85],
    [1.20, -1.00],
    [0.85, -0.70],
    [1.10, -0.90],
]

# j4 sweep range for alignment search
J4_SWEEP = [math.radians(a) for a in
            [-1.8, -1.4, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 1.4, 1.8]]

COARSE_J1 = [math.radians(a) for a in
             [0, 30, 60, 90, 120, 150, -30, -60, -90, -120, -150, 180]]

# ── FK ────────────────────────────────────────────────────────────────
def _Rx(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]],dtype=float)
def _Ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]],dtype=float)
def _Rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]],dtype=float)
def _rpy(r,p,y): return _Rz(y)@_Ry(p)@_Rx(r)
def _T(R,t):
    M=np.eye(4); M[:3,:3]=R
    M[:3,3]=np.asarray(t,dtype=float); return M

def fk(q1,q2,q3,q4):
    T=np.eye(4)
    T=T@_T(_rpy(1.5708,0,0),[0,0,0.0427])@_T(_Ry(q1),[0,0,0])
    T=T@_T(_rpy(0,0,-1.3621),[0,0.0588,0])@_T(_Rz(q2),[0,0,0])
    T=T@_T(_rpy(0,0,1.02985),[-0.12954806,0.05339756,0])@_T(_Rz(q3),[0,0,0])
    T=T@_T(_rpy(0,0,0.3323),[0.1543,0.0532,0])@_T(_Rz(q4),[0,0,0])
    T=T@_T(np.eye(3),[0.11,0,0])
    pos=T[:3,3].copy()
    beam=-T[:3,1].copy()
    n=np.linalg.norm(beam); 
    if n>1e-9: beam/=n
    return pos,beam

def beam_alignment(lamp_pos, beam_dir):
    """
    Cosine similarity between beam direction and lamp→tip unit vector.
    Returns value in [-1, 1]. 1.0 = perfectly aligned.
    """
    to_tip = TOOL_TIP - lamp_pos
    d = np.linalg.norm(to_tip)
    if d < 1e-6:
        return 0.0
    to_tip /= d
    return float(np.dot(beam_dir, to_tip))

def best_j4_for_alignment(q1, q2, q3):
    """
    Sweep j4 over J4_SWEEP values and return the j4 that maximizes
    beam alignment (dot product of beam with lamp→tip direction).
    """
    best_j4 = 0.0
    best_align = -999.0
    for j4 in J4_SWEEP:
        pos, beam = fk(q1, q2, q3, j4)
        align = beam_alignment(pos, beam)
        if align > best_align:
            best_align = align
            best_j4 = j4
    return best_j4, best_align

def ray_blocked(lamp,tip,bc,bh):
    d=tip-lamp; t0,t1=0.0,1.0
    for i in range(3):
        if abs(d[i])<1e-9:
            if lamp[i]<bc[i]-bh[i] or lamp[i]>bc[i]+bh[i]:
                return False
            continue
        a=(bc[i]-bh[i]-lamp[i])/d[i]
        b=(bc[i]+bh[i]-lamp[i])/d[i]
        if a>b: a,b=b,a
        t0=max(t0,a); t1=min(t1,b)
        if t0>t1: return False
    return True

def ray_clearance(lamp,tip,bc,bh):
    best=float('inf')
    for fr in np.linspace(0.05,0.95,20):
        pt=lamp+fr*(tip-lamp)
        q=np.abs(pt-bc)-bh
        dist=float(np.linalg.norm(np.maximum(q,0.))+min(float(np.max(q)),0.))
        best=min(best,dist)
    return float(best)

# ── Optimizer ─────────────────────────────────────────────────────────

def _cost(cfg, cur_q, box_pos, box_vel):
    """
    cfg = [j1, j2, j3, j4]
    Returns (cost, lamp_pos, clearance, alignment) or None.
    """
    lamp, beam = fk(*cfg)

    if lamp[2] < 0.10:
        return None
    if np.linalg.norm(lamp - TOOL_TIP) > 0.65:
        return None
    if ray_blocked(lamp, TOOL_TIP, box_pos, BOX_HALF):
        return None

    clr        = ray_clearance(lamp, TOOL_TIP, box_pos, BOX_HALF)
    align      = beam_alignment(lamp, beam)
    joint_delta= sum(abs(cfg[i] - cur_q[i]) for i in range(4))
    future_box = box_pos + box_vel * 1.5
    future_clr = ray_clearance(lamp, TOOL_TIP, future_box, BOX_HALF)

    cost = (- W_CLEARANCE  * clr
            + W_JOINT_MOVE * joint_delta
            - W_FUTURE     * future_clr
            - W_ALIGNMENT  * align)      # ← alignment term

    return cost, lamp, clr, align


def hierarchical_search(cur_q, box_pos, box_vel, logger=None):
    """
    Phase 1 — coarse: 12 j1 × 4 [j2,j3] shapes, j4 solved for alignment
    Phase 2 — fine:   ±15° around top 3 j1 winners
    """

    # ── Phase 1: coarse ────────────────────────────────────────────────
    coarse_results = []
    for j1 in COARSE_J1:
        best_for_j1 = None
        for shape in ARM_SHAPES:
            q2, q3 = shape
            # Find best j4 for beam alignment at this j1,j2,j3
            j4, align_pred = best_j4_for_alignment(j1, q2, q3)
            cfg = [j1, q2, q3, j4]
            r = _cost(cfg, cur_q, box_pos, box_vel)
            if r is None:
                continue
            cost, lamp, clr, align = r
            if best_for_j1 is None or cost < best_for_j1[0]:
                best_for_j1 = (cost, cfg, lamp, clr, align)
        if best_for_j1 is not None:
            coarse_results.append(best_for_j1)

    if not coarse_results:
        return None

    coarse_results.sort(key=lambda x: x[0])
    top3 = coarse_results[:3]

    if logger:
        logger.info(
            f'  [OPT] Coarse: {len(coarse_results)} valid, '
            f'top3 j1={[round(math.degrees(t[1][0])) for t in top3]}° '
            f'align={[round(t[4],2) for t in top3]}')

    # ── Phase 2: fine refinement ────────────────────────────────────────
    best_final = None
    for _, cfg_coarse, _, _, _ in top3:
        j1_center = cfg_coarse[0]
        fine_angles = [j1_center + math.radians(d)
                       for d in [-15, -8, 0, 8, 15]]
        for j1_fine in fine_angles:
            j1_fine = max(-math.pi, min(math.pi, j1_fine))
            for shape in ARM_SHAPES:
                q2, q3 = shape
                # Re-solve j4 for alignment at refined j1
                j4, _ = best_j4_for_alignment(j1_fine, q2, q3)
                cfg = [j1_fine, q2, q3, j4]
                r = _cost(cfg, cur_q, box_pos, box_vel)
                if r is None:
                    continue
                cost, lamp, clr, align = r
                if best_final is None or cost < best_final[0]:
                    best_final = (cost, cfg, lamp, clr, align)

    if best_final is None:
        return None

    _, cfg, lamp, clr, align = best_final
    return cfg, lamp, clr, align


# ── Node ──────────────────────────────────────────────────────────────

class LampAdjuster(Node):
    def __init__(self):
        super().__init__('lamp_adjuster')
        self.cur_q   = {n: 0.0 for n in JOINT_NAMES}
        self.got_j   = False
        self.box_pos = None
        self.box_vel = np.zeros(3)
        self._prev_box_pos  = None
        self._prev_box_time = None
        self.allowed    = False
        self.confirm    = 0
        self.adj_count  = 0
        self.last_check = 0.0
        self.moving     = False
        self.move_end_t = 0.0
        self._init_done = False

        self.create_subscription(JointState, '/joint_states',      self._js, 10)
        self.create_subscription(Point,      '/lumeneye/box_pose', self._bp, 10)
        self.allow_pub = self.create_publisher(Bool, '/lumeneye/plate_allow',    10)
        self.adj_pub   = self.create_publisher(Bool, '/lumeneye/lamp_adjusting', 10)
        self.traj_pub  = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_timer(0.1, self._loop)
        self.create_timer(5.0, self._init)

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║  LumenEye — Beam-Aligned Optimizer           ║')
        self.get_logger().info('║  j4 solved analytically for lamp alignment   ║')
        self.get_logger().info('║  Cost: clearance+joint_delta+future+align    ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')

    def _js(self, m):
        for i, n in enumerate(m.name):
            if n in self.cur_q and i < len(m.position):
                self.cur_q[n] = m.position[i]
        self.got_j = True

    def _bp(self, m):
        now = self._now()
        new_pos = np.array([m.x, m.y, m.z])
        if self._prev_box_pos is not None and self._prev_box_time is not None:
            dt = now - self._prev_box_time
            if dt > 0.01:
                raw_vel = (new_pos - self._prev_box_pos) / dt
                self.box_vel = 0.7 * self.box_vel + 0.3 * raw_vel
        self._prev_box_pos  = new_pos.copy()
        self._prev_box_time = now
        self.box_pos = new_pos

    def _init(self):
        if self._init_done:
            return
        self._init_done = True
        # Find aligned init pose
        j4_init, align = best_j4_for_alignment(0.0, 1.0, -0.85)
        init = [0.0, 1.0, -0.85, j4_init]
        self._send(init, sec=4)
        p, beam = fk(*init)
        self.get_logger().info(
            f'Init: j1=0° j4={math.degrees(j4_init):.1f}° '
            f'lamp=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) '
            f'alignment={align:.3f}')

    def _loop(self):
        if not self._init_done or not self.got_j or self.box_pos is None:
            return
        now = self._now()
        if now - self.last_check < CHECK_DT:
            return
        self.last_check = now

        if self.moving:
            if now < self.move_end_t:
                self.allow_pub.publish(Bool(data=False))
                return
            else:
                self.moving = False
                self.get_logger().info('  [ARM] Movement done.')

        cur_q = [self.cur_q.get(n, 0.) for n in JOINT_NAMES]
        lamp, beam = fk(*cur_q)
        blk   = ray_blocked(lamp, TOOL_TIP, self.box_pos, BOX_HALF)
        align = beam_alignment(lamp, beam)
        j1d   = math.degrees(cur_q[0])
        j4d   = math.degrees(cur_q[3])
        lx,ly,lz = lamp
        bx,by,bz = self.box_pos

        if not blk:
            self.confirm += 1
            self.allow_pub.publish(Bool(data=True))
            if not self.allowed and self.confirm >= CONFIRM_N:
                self.allowed = True
                self.adj_pub.publish(Bool(data=False))
                clr = ray_clearance(lamp, TOOL_TIP, self.box_pos, BOX_HALF)
                self.get_logger().info('')
                self.get_logger().info('█'*54)
                self.get_logger().info('  ✓  BEAM CLEAR — plate_allow = TRUE')
                self.get_logger().info(
                    f'  ✓  j1={j1d:+.0f}° j4={j4d:+.1f}°'
                    f' lamp=({lx:.3f},{ly:.3f},{lz:.3f})')
                self.get_logger().info(
                    f'  ✓  clearance={clr:.3f}m  alignment={align:.3f}')
                self.get_logger().info('█'*54)
            elif self.allowed:
                self.get_logger().info(
                    f'[OK] j1={j1d:+.0f}° j4={j4d:+.1f}°'
                    f' align={align:.2f} CLEAR')
        else:
            self.confirm = 0
            self.allow_pub.publish(Bool(data=False))
            if self.allowed:
                self.allowed = False
                self.adj_count += 1
                self.adj_pub.publish(Bool(data=True))
                self.get_logger().warn('▓'*54)
                self.get_logger().warn(
                    f'  SHADOW #{self.adj_count}! j1={j1d:+.0f}° align={align:.2f}')
                self.get_logger().warn('▓'*54)
                self._adjust()
            elif not self.moving:
                self.get_logger().warn(
                    f'[BLOCKED] j1={j1d:+.0f}° align={align:.2f}'
                    f' box=({bx:.2f},{by:.2f},{bz:.2f})')
                self._adjust()

    def _adjust(self):
        if self.box_pos is None:
            return
        cur_q = [self.cur_q.get(n, 0.) for n in JOINT_NAMES]
        result = hierarchical_search(
            cur_q, self.box_pos, self.box_vel, logger=self.get_logger())

        if result is None:
            self.get_logger().error('  [OPT] No valid config found!')
            return

        joints, pred, clr, align = result
        j1d = math.degrees(joints[0])
        j4d = math.degrees(joints[3])
        lx,ly,lz = pred
        delta_j1 = abs(joints[0] - cur_q[0])

        self.get_logger().info(
            f'  [OPT] j1={j1d:+.0f}° j4={j4d:+.1f}°'
            f' lamp=({lx:.3f},{ly:.3f},{lz:.3f})'
            f' clr={clr:.3f}m align={align:.3f}')

        sec = max(2, int(delta_j1 / 0.8) + 1)
        self._send(joints, sec=sec)
        self.moving = True
        self.move_end_t = self._now() + sec + 1.0

    def _send(self, j, sec=3):
        t = JointTrajectory()
        t.joint_names = JOINT_NAMES
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in j]
        p.time_from_start = Duration(sec=sec, nanosec=0)
        t.points = [p]
        self.traj_pub.publish(t)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    n = LampAdjuster()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()