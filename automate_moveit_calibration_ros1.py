#!/usr/bin/env python3
"""ROS 1 / MoveIt: plan cautious Panda calibration poses; optionally execute and record Vicon/FK TF pairs.

Examples:
  python3 automate_moveit_calibration_ros1.py --num-poses 2
  python3 automate_moveit_calibration_ros1.py --num-poses 2 --execute --capture --vicon-world vicon_world --marker-frame panda_calib
  python3 automate_moveit_calibration_ros1.py --num-poses 15 --execute --auto --capture --vicon-world vicon_world --marker-frame panda_calib

Without --execute, PLANS ONLY. --auto requires --execute; otherwise operator approves each motion.
MoveIt does not model a newly attached marker fixture unless added to the planning scene.
"""
import argparse
import json
import math
import random
import sys
from pathlib import Path

import moveit_commander
import rospy
import tf


def unpack_plan(result):
    """MoveIt 1 Python API variations: RobotTrajectory or (success, trajectory, time, error)."""
    if isinstance(result, tuple):
        return bool(result[0]), result[1]
    return bool(result.joint_trajectory.points), result


def capture_pair(listener, args):
    captures = []
    for _ in range(args.samples_per_pose):
        try:
            # lookupTransform(target, source) returns T_target_source.
            b_e = listener.lookupTransform(args.base, args.flange, rospy.Time(0))
            w_m = listener.lookupTransform(args.vicon_world, args.marker_frame, rospy.Time(0))
            captures.append({
                'B_E': {'xyz': list(b_e[0]), 'xyzw': list(b_e[1])},
                'W_M': {'xyz': list(w_m[0]), 'xyzw': list(w_m[1])},
            })
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            rospy.logwarn('TF lookup failed: %s', exc)
        rospy.sleep(args.dt)
    return captures


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--num-poses', type=int, default=2)
    parser.add_argument('--group', default='panda_arm')
    parser.add_argument('--max-joint-offset', type=float, default=0.22,
                        help='Maximum random displacement (radians) per joint relative to INITIAL pose')
    parser.add_argument('--velocity-scale', type=float, default=0.10)
    parser.add_argument('--acceleration-scale', type=float, default=0.10)
    parser.add_argument('--planning-time', type=float, default=8.0)
    parser.add_argument('--planning-attempts', type=int, default=5)
    parser.add_argument('--candidate-attempts', type=int, default=30)
    parser.add_argument('--settle-seconds', type=float, default=2.0)
    parser.add_argument('--seed', type=int, default=5)
    parser.add_argument('--execute', action='store_true', help='Allow real motion; otherwise plan-only')
    parser.add_argument('--auto', action='store_true', help='Execute without confirmation per pose (supervision required)')
    parser.add_argument('--capture', action='store_true', help='Capture TF pairs at each successful pose')
    parser.add_argument('--base', default='panda_link0')
    parser.add_argument('--flange', default='panda_link8')
    parser.add_argument('--vicon-world', default='vicon_world')
    parser.add_argument('--marker-frame', default='panda_calib')
    parser.add_argument('--out', default='calibration_pairs.json')
    parser.add_argument('--samples-per-pose', type=int, default=12)
    parser.add_argument('--dt', type=float, default=0.05)
    args = parser.parse_args()

    if args.num_poses < 1 or args.max_joint_offset <= 0 or args.max_joint_offset > 0.5:
        parser.error('Require num-poses >= 1 and 0 < max-joint-offset <= 0.5 rad')
    if not (0 < args.velocity_scale <= 0.25 and 0 < args.acceleration_scale <= 0.25):
        parser.error('This calibration tool caps velocity/acceleration scale at 0.25')
    if args.auto and not args.execute:
        parser.error('--auto requires --execute')
    if args.capture and not args.execute:
        parser.error('--capture requires --execute')

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_calibration_motion', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(args.group)
    group.set_max_velocity_scaling_factor(args.velocity_scale)
    group.set_max_acceleration_scaling_factor(args.acceleration_scale)
    group.set_planning_time(args.planning_time)
    group.set_num_planning_attempts(args.planning_attempts)
    joint_names = group.get_active_joints()
    initial = list(group.get_current_joint_values())
    if len(initial) != 7 or len(joint_names) != 7:
        raise RuntimeError('Expected 7 active Panda arm joints; found {}: {}'.format(len(initial), joint_names))

    bounds = [robot.get_joint(name).bounds() for name in joint_names]
    print('Group:', args.group)
    print('Joints:', joint_names)
    print('Initial configuration:', [round(x, 3) for x in initial])
    print('Joint bounds:', bounds)
    print('MODE:', 'EXECUTION ENABLED' if args.execute else 'PLAN ONLY — NO ROBOT MOTION')
    print('WARNING: MoveIt collision checking DOES NOT include your marker attachment unless modeled.')
    print('Keep the workspace clear and an operator at the E-stop throughout.')
    if args.execute:
        acknowledgement = input('Confirm real robot is ready, clear, and supervised (type YES): ').strip()
        if acknowledgement != 'YES':
            print('Cancelled before movement.')
            return

    listener = tf.TransformListener() if args.capture else None
    random_generator = random.Random(args.seed)
    recorded = []
    successes = 0
    for pose_idx in range(args.num_poses):
        if rospy.is_shutdown():
            break
        trajectory = None
        target = None
        for attempt in range(args.candidate_attempts):
            # Restrict goals to a small neighborhood of the observed initial pose.
            candidate = [home + random_generator.uniform(-args.max_joint_offset, args.max_joint_offset)
                         for home in initial]
            if any(not (lower + 0.02 <= angle <= upper - 0.02)
                   for angle, (lower, upper) in zip(candidate, bounds)):
                continue
            group.set_start_state_to_current_state()
            try:
                group.set_joint_value_target(candidate)
                succeeded, planned = unpack_plan(group.plan())
            except Exception as exc:
                rospy.logwarn('Planning candidate rejected: %s', exc)
                continue
            if succeeded and planned.joint_trajectory.points:
                target, trajectory = candidate, planned
                break
        if trajectory is None:
            rospy.logerr('No valid collision-checked plan for pose %d; stopping.', pose_idx+1)
            break
        print('\nPose {}/{}: target = {}'.format(pose_idx+1, args.num_poses,
                                                [round(x, 3) for x in target]))
        print('Trajectory points:', len(trajectory.joint_trajectory.points))
        if not args.execute:
            print('PLAN ONLY: no movement or capture performed')
            successes += 1
            continue
        if not args.auto:
            choice = input('Execute this planned trajectory? [y/N/q]: ').strip().lower()
            if choice == 'q':
                break
            if choice != 'y':
                print('Skipped.')
                continue
        # Moves only the trajectory produced by MoveIt (joint limits and modeled collisions checked).
        ok = group.execute(trajectory, wait=True)
        group.stop()
        if not ok:
            rospy.logerr('MoveIt execution failed. Stopping sequence.')
            break
        rospy.sleep(args.settle_seconds)
        successes += 1
        if args.capture:
            captures = capture_pair(listener, args)
            if len(captures) < max(3, args.samples_per_pose // 2):
                rospy.logerr('Not enough valid Vicon/FK TF samples at pose %d; stopping.', pose_idx+1)
                break
            recorded.append({'pose_id': len(recorded)+1, 'captures': captures})
            payload = {'frames': {'base': args.base, 'flange': args.flange,
                                  'vicon_world': args.vicon_world, 'marker': args.marker_frame},
                       'poses': recorded}
            Path(args.out).write_text(json.dumps(payload, indent=2))
            print('Saved capture {} to {}'.format(len(recorded), args.out))
        else:
            print('Pose settled. Ready for external/manual capture.')
    print('\nSuccessful plans/executions:', successes, 'captures:', len(recorded))
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
