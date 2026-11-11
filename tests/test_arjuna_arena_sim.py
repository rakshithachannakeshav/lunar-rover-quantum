#!/usr/bin/env python3
"""
tests/test_arjuna_arena_sim.py
------------------------------
The REAL quantum_nav_node.py (stub rospy) driving a simulated rover in a simulated 2.21 m lunar
arena: 3 craters with raised 2 cm rims, 4 rocks, a low border wall and a table edge. The
terrain detector runs on depth images rendered from that arena (tests/arena_sim.py).

"Clean" = reached the goal with no rock contact, no wheel on a crater rim, never inside a
crater and never off the table, judged from the TRUE rover position.
No ROS, no camera, no robot. Simulation only; see robot/arjuna_quantum_nav/README.md for what
still has to be checked on the real rover.
"""

import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(__file__)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import arena_sim as sim  # noqa: E402
import arena_world as aw  # noqa: E402
import compare_runs  # noqa: E402
import make_arena_map as mam  # noqa: E402
import nav_core  # noqa: E402

LAUNCH = os.path.abspath(os.path.join(HERE, '..', 'robot', 'arjuna_quantum_nav', 'launch'))
ALGOS = nav_core.ALGORITHMS


def describe(r):
    return "%s | %s | %s" % (r.summary(), r.status, [m for lvl, m in r.log if lvl != 'info'][-4:])


# ── the whole arena, every planner ───────────────────────────────────────────
@pytest.mark.parametrize('algo', ALGOS)
def test_each_planner_crosses_the_arena_without_touching_anything(algo):
    arena = sim.standard_arena()
    r = sim.run_mission(arena, algo, (0.25, 0.25, 0.0), [(1.95, 1.95)])
    assert r.clean, describe(r)
    assert r.cam_frames > 20                      # the camera really was in the loop
    assert r.min_rock_gap > 0.0 and r.min_crater_gap > 0.0
    assert r.bumps == 0                           # the camera saw every crater before a wheel did


@pytest.mark.parametrize('algo', ALGOS)
def test_each_planner_gets_back_the_other_way(algo):
    r = sim.run_mission(sim.standard_arena(), algo, (1.95, 1.95, 3.0), [(0.25, 0.25)], seed=2)
    assert r.clean, describe(r)


def test_a_route_through_three_points_is_driven_in_order():
    arena = sim.standard_arena()
    r = sim.run_mission(arena, 'astar', (0.25, 0.25, 0.0), [(1.95, 0.40), (1.95, 1.95), (0.40, 1.90)],
                        max_time=300)
    assert r.clean, describe(r)
    # it really went to the middle points: the trace comes within 15 cm of each
    for gx, gy in [(1.95, 0.40), (1.95, 1.95)]:
        assert min(math.hypot(x - gx, y - gy) for _, x, y, _ in r.trace) < 0.15


def test_the_terrain_map_matches_the_arena_at_the_end_of_a_run():
    arena = sim.standard_arena()
    r = sim.run_mission(arena, 'astar', (0.25, 0.25, 0.0), [(1.95, 1.95)])
    T = r.node.terrain
    found = T.discs()
    assert len(found) >= 2                        # the craters on the way were mapped
    for x, y, rad in found:                       # ... where they really are
        near = min(arena.craters, key=lambda c: math.hypot(c.x - x, c.y - y))
        assert math.hypot(near.x - x, near.y - y) < 0.08
        assert rad >= near.apothem - 0.02         # and big enough to cover the rim
    rocks = T.points(['rock'])
    assert rocks
    # no phantom rocks: every mapped rock cell is on or next to a real rock (the camera blocks a
    # little hidden floor behind each one), and the border cells lie along the table edge
    assert all(min(sim.dist_to_rock(rk, x, y) for rk in arena.rocks) < 0.25 for x, y in rocks)
    for x, y in T.points(['border']):
        assert min(x, y, arena.size - x, arena.size - y) < 0.12
    assert not T.points(['nofloor'])              # no table edge invented in the middle of the table


# ── the camera is what keeps the rover off the rims ──────────────────────────
def one_crater_in_the_way():
    return aw.Arena(2.21, craters=[aw.Crater(1.10, 1.10, 0.36, rot_deg=10)])


def test_with_the_camera_a_crater_on_the_straight_line_is_avoided():
    r = sim.run_mission(one_crater_in_the_way(), 'astar', (0.30, 0.30, 0.78), [(1.90, 1.90)])
    assert r.clean, describe(r)
    assert r.bumps == 0 and r.replans >= 1       # it replanned round the crater when it saw it
    assert r.min_crater_gap > 0.03


def test_without_the_camera_the_imu_bump_stops_backs_out_and_replans():
    arena = one_crater_in_the_way()
    r = sim.run_mission(arena, 'astar', (0.30, 0.30, 0.78), [(1.90, 1.90)], use_camera=False,
                        max_time=300)
    assert r.reached, describe(r)
    assert r.bumps >= 1                           # felt the rim ...
    assert r.inside_crater == 0                   # ... but never drove into the crater
    assert r.rock_touch == 0 and r.off_table == 0
    assert any('Bump' in m for lvl, m in r.log)
    # the crater edge is now marked on the planning map, in the world frame
    assert r.node.terrain.discs()


def test_use_craters_false_ignores_camera_and_bump():
    arena = one_crater_in_the_way()
    r = sim.run_mission(arena, 'astar', (0.30, 0.30, 0.78), [(1.90, 1.90)], use_camera=True,
                        params={'use_craters': False}, max_time=150)
    assert r.bumps == 0 and not r.node.terrain.discs()


def test_old_stop_and_wait_behaviour_still_works_with_avoid_obstacles_false():
    arena = aw.Arena(2.21, rocks=[aw.Rock(1.10, 1.10, 0.30, 0.30, 0.25)])
    r = sim.run_mission(arena, 'astar', (0.30, 0.30, 0.78), [(1.90, 1.90)], use_camera=False,
                        params={'avoid_obstacles': False})
    assert r.clean, describe(r)


def test_avoider_on_reverses_from_a_box_15cm_ahead_but_not_when_the_rear_is_blocked():
    node, clock, msgs = fresh_node(avoid_obstacles=True)
    send_pose(node, msgs, 1.0, 1.0)
    for rear, expect_back in ((3.0, True), (0.2, False)):
        # lidar is mounted backwards: its 0 deg is behind the rover (offset 180)
        sc = msgs['LaserScan']()
        sc.angle_min, sc.angle_increment, sc.range_min, sc.range_max = -math.pi, math.pi / 180, 0.05, 6.0
        sc.ranges = [float('inf')] * 360
        sc.ranges[180] = rear                      # behind the rover
        sc.ranges[0] = 0.15                        # box straight ahead
        sim.rospy.SUBSCRIBERS['scan'](sc)
        node.avoider = nav_core.ObstacleAvoider(max_linear=0.12, max_angular=0.6, **node.avoid_params)
        s = nav_core.scan_sectors(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_max,
                                  sc.range_min, node.laser_yaw_offset)
        cmd, _ = node.avoider.update(s, 1.0, 1.0)
        assert (cmd[0] < 0) == expect_back, (rear, cmd)


# ── known layout: map from make_arena_map.py, use_map:=true ──────────────────
def occupancy_for(arena):
    w, h, rows, (ox, oy) = mam.build_map(sim.layout_from_arena(arena), 0.01, 0.2)
    data = [0] * (w * h)
    for j, row in enumerate(rows):
        for i, v in enumerate(row):
            data[(h - 1 - j) * w + i] = 100 if v == mam.OCC else 0
    return w, h, 0.01, ox, oy, data


@pytest.mark.parametrize('algo', ALGOS)
def test_planners_on_the_known_layout_map(algo):
    arena = sim.standard_arena()
    r = sim.run_mission(arena, algo, (0.25, 0.25, 0.0), [(1.95, 1.95)], use_camera=False,
                        params={'use_map': True}, occupancy=occupancy_for(arena))
    assert r.clean, describe(r)
    assert r.bumps == 0                           # the map already holds every crater


def test_the_planner_comparison_is_logged_to_the_csv(tmp_path):
    """One goal, each planner in turn on the same map, logged for the comparison."""
    arena = sim.standard_arena()
    csv_path = str(tmp_path / 'quantum_nav_runs.csv')
    for algo in ALGOS:
        r = sim.run_mission(arena, algo, (0.25, 0.25, 0.0), [(1.95, 1.95)], use_camera=False,
                            params={'use_map': True}, occupancy=occupancy_for(arena), log_csv=csv_path)
        assert r.reached, describe(r)
    rows = open(csv_path).read().strip().splitlines()
    assert len(rows) == 1 + len(ALGOS)
    lines = compare_runs.summarise([dict(zip(rows[0].split(','), row.split(','))) for row in rows[1:]])
    assert sorted(L['algorithm'] for L in lines) == sorted(ALGOS)
    assert all(L['outcome'] == 'reached' and L['driven_distance_m'] > 2.0 for L in lines)
    assert all(L['measured_energy_J'] > 0 for L in lines)


# ── goal handling, STOP, launch-time goal ────────────────────────────────────
def fresh_node(arena=None, **params):
    arena = arena or sim.standard_arena()
    node, clock = sim.make_node(arena, 'astar', params)
    return node, clock, sim.rospy.msgs


def send_pose(node, msgs, x, y, yaw=0.0):
    m = msgs['PoseWithCovarianceStamped']()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    qx, qy, qz, qw = sim.quat(0, 0, yaw)
    o = m.pose.pose.orientation
    o.x, o.y, o.z, o.w = qx, qy, qz, qw
    sim.rospy.SUBSCRIBERS['robot_pose_ekf/odom_combined'](m)


def goal_msg(msgs, x, y):
    g = msgs['PoseStamped']()
    g.pose.position.x, g.pose.position.y = x, y
    return g


def test_goal_on_a_detected_crater_rock_or_outside_the_arena_is_refused():
    node, clock, msgs = fresh_node()
    send_pose(node, msgs, 0.25, 0.25)
    node.terrain.add_mark(1.0, 1.0, 0.2)
    node.terrain.add((0, 0, 0), [{"type": "rock", "cells": [[0.5, 0.5], [0.54, 0.5]]}])
    node.terrain.add((0, 0, 0), [{"type": "rock", "cells": [[0.5, 0.5], [0.54, 0.5]]}])
    goal_sub = sim.rospy.SUBSCRIBERS['move_base_simple/goal']
    for (x, y), what in [((1.05, 1.0), 'crater'), ((0.52, 0.5), 'rock'), ((3.0, 1.0), 'outside')]:
        goal_sub(goal_msg(msgs, x, y))
        assert node.goal is None and not node.new_goal, (x, y)
        assert 'goal rejected' in node.last_outcome and what in node.last_outcome
    goal_sub(goal_msg(msgs, 1.8, 0.4))            # open floor: accepted
    assert node.goal == (1.8, 0.4)


def test_a_route_with_one_bad_point_is_refused_as_a_whole():
    node, clock, msgs = fresh_node()
    node.terrain.add_mark(1.0, 1.0, 0.2)
    route = msgs['Path']()
    for x, y in [(0.4, 0.4), (1.0, 1.0), (1.8, 1.8)]:
        route.poses.append(goal_msg(msgs, x, y))
    sim.rospy.SUBSCRIBERS['route'](route)
    assert node.goal is None and 'point 2' in node.last_outcome and 'crater' in node.last_outcome


def test_stop_cancels_the_back_out_and_the_goal():
    node, clock, msgs = fresh_node()
    send_pose(node, msgs, 0.5, 0.5)
    node.backing = (0.5, 0.5, clock.time())
    node.goal = (1.8, 1.8)
    sim.rospy.SUBSCRIBERS['cancel'](msgs['Empty']())
    clock.t += 0.1
    send_pose(node, msgs, 0.5, 0.5)
    node.step(0.1, clock.time())
    assert node.backing is None and node.goal is None
    last = sim.rospy.PUBLISHED['cmd_vel'][-1]
    assert last.linear.x == 0.0 and last.angular.z == 0.0


def test_a_goal_given_at_launch_survives_waiting_for_the_first_pose():
    node, clock, msgs = fresh_node(goal_x=1.8, goal_y=0.4)
    for _ in range(5):                             # no pose yet: it must wait, not drop the goal
        clock.t += 0.1
        node.step(0.1, clock.time() - 1)
    assert node.status.startswith('waiting for pose') and node.new_goal
    send_pose(node, msgs, 0.25, 0.25)
    clock.t += 0.1
    node.step(0.1, clock.time() - 1)
    assert node.status.startswith('driving') or node.status.startswith('replanned'), node.status


def test_a_launch_goal_on_a_known_object_is_refused_at_startup():
    arena = sim.standard_arena()
    r = arena.craters[1]
    node, clock, msgs = fresh_node(arena, goal_x=r.x, goal_y=r.y)
    # the terrain map is empty at start, so nothing is known yet: the goal is accepted ...
    assert node.goal == (r.x, r.y)
    # ... but outside the arena it is refused straight away
    node2, _, _ = fresh_node(arena, goal_x=5.0, goal_y=5.0)
    assert node2.goal is None and 'outside' in node2.last_outcome


def test_the_navigator_stops_at_the_table_edge():
    node, clock, msgs = fresh_node()
    send_pose(node, msgs, 0.25, 0.25)
    node.goal, node.new_goal = (1.8, 1.8), True
    clock.t += 0.1
    node.step(0.1, clock.time() - 1)
    assert node.follower is not None
    send_pose(node, msgs, 2.19, 1.0)               # the odometry says the rover is at the edge
    clock.t += 0.1
    node.step(0.1, clock.time() - 1)
    assert node.status == 'ABORTED: at the table edge'
    assert sim.rospy.PUBLISHED['cmd_vel'][-1].linear.x == 0.0


def test_markers_and_terrain_json_are_published_for_rviz_and_the_web_page():
    node, clock, msgs = fresh_node()
    node.terrain.add_mark(1.0, 1.0, 0.2)
    node.terrain.add((0, 0, 0), [{"type": "rock", "cells": [[0.5, 0.5]]}] * 1)
    node.terrain.add((0, 0, 0), [{"type": "rock", "cells": [[0.5, 0.5]]}])
    node._publish_terrain(force=True)
    snap = json.loads(sim.rospy.PUBLISHED['quantum_nav/terrain'][-1].data
                      if 'quantum_nav/terrain' in sim.rospy.PUBLISHED
                      else sim.rospy.PUBLISHED['terrain'][-1].data)
    assert snap['craters'][0]['r'] == 0.2 and snap['rock'] == [[0.5, 0.5]]
    key = [k for k in sim.rospy.PUBLISHED if k.endswith('terrain_markers')][0]
    kinds = {m.ns for m in sim.rospy.PUBLISHED[key][-1].markers}
    assert 'craters' in kinds and 'rock' in kinds


# ── launch files: what the sim assumes is what arena.launch sets ─────────────
def launch_args(name):
    root = ET.parse(os.path.join(LAUNCH, name)).getroot()
    return dict((a.get('name'), a.get('default')) for a in root.findall('arg')), root


def test_arena_launch_sets_what_the_simulation_assumes():
    args, root = launch_args('arena.launch')
    inc = [i for i in root.findall('include') if 'quantum_nav.launch' in i.get('file')][0]
    passed = dict((a.get('name'), a.get('value')) for a in inc.findall('arg'))
    assert float(passed['laser_yaw_offset_deg']) == 180.0
    assert passed['avoid_obstacles'] == '$(arg avoid_obstacles)' and args['avoid_obstacles'] == 'false'
    assert float(passed['soft_margin']) == sim.ARENA_PARAMS['soft_margin']
    assert float(passed['lidar_stop_distance']) == sim.ARENA_PARAMS['lidar_stop_distance']
    assert float(passed['path_spacing']) == sim.ARENA_PARAMS['path_spacing']
    assert args['use_craters'] == 'true' and args['use_camera'] == 'true'
    assert args['use_map'] == 'false' and float(args['arena_size']) == 2.21
    # the detector is only started when use_camera is true
    det = [i for i in root.findall('include') if 'crater_detector' in i.get('file')][0]
    assert det.get('if') == '$(arg use_camera)'
    nav_args, _ = launch_args('quantum_nav.launch')
    assert float(nav_args['laser_yaw_offset_deg']) == 180.0


def test_quantum_nav_launch_has_every_arg_arena_launch_passes():
    nav_args, _ = launch_args('quantum_nav.launch')
    _, root = launch_args('arena.launch')
    inc = [i for i in root.findall('include') if 'quantum_nav.launch' in i.get('file')][0]
    for a in inc.findall('arg'):
        assert a.get('name') in nav_args, a.get('name')
    # and every $(arg x) it uses is declared in arena.launch itself
    text = open(os.path.join(LAUNCH, 'arena.launch')).read()
    body = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    mine = launch_args('arena.launch')[0]
    for used in set(re.findall(r'\$\(arg (\w+)\)', body)):
        assert used in mine, used
