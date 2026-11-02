#!/usr/bin/env python3
"""Collect stationary ROS1 TF pose pairs for Panda/Vicon robot-world calibration.

Usage:
  python3 collect_poses_ros1.py --vicon-world vicon_world --marker-frame panda_calib \
    --base panda_link0 --flange panda_link8 --out calibration_pairs.json

Requirements: rospy, tf (ROS1). The Vicon rigid body must be in the same TF tree
or accessible through the specified TF frames. Keep markers rigidly attached to flange.
"""
import argparse
import json
import os
import time

import rospy
import tf


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-world', required=True, help='TF Vicon world frame')
    p.add_argument('--marker-frame', required=True, help='TF tracked rigid-body frame')
    p.add_argument('--base', default='panda_link0')
    p.add_argument('--flange', default='panda_link8')
    p.add_argument('--out', default='calibration_pairs.json')
    p.add_argument('--samples-per-pose', type=int, default=12)
    p.add_argument('--dt', type=float, default=0.05)
    args = p.parse_args()
    rospy.init_node('panda_vicon_pose_collector', anonymous=True)
    listener = tf.TransformListener()
    samples = []
    print('Required TF: {} -> {}, {} -> {}'.format(args.base,args.flange,args.vicon_world,args.marker_frame))
    print('Move Panda using an approved controller, stop, ensure Vicon sees all markers, then press Enter.')
    print('Type q to finish. All measurements are captured only while stationary.')
    while not rospy.is_shutdown():
        choice = input('\nPose {}: Enter=record, q=quit: '.format(len(samples)+1)).strip().lower()
        if choice in ('q','quit','exit'):
            break
        captures=[]
        for i in range(args.samples_per_pose):
            try:
                # In ROS tf lookupTransform(target, source) returns transform T_target_source.
                b_e = listener.lookupTransform(args.base, args.flange, rospy.Time(0))
                w_m = listener.lookupTransform(args.vicon_world, args.marker_frame, rospy.Time(0))
                captures.append({'B_E': {'xyz': list(b_e[0]), 'xyzw': list(b_e[1])},
                                 'W_M': {'xyz': list(w_m[0]), 'xyzw': list(w_m[1])}})
            except (tf.Exception,tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as e:
                print('TF sample failed:',e)
            rospy.sleep(args.dt)
        if len(captures) < max(3, args.samples_per_pose // 2):
            print('Insufficient valid TF samples; pose not saved.'); continue
        # Store raw snapshots so the solver can assess within-pose measurement variability.
        samples.append({'pose_id':len(samples)+1,'captures':captures})
        with open(args.out,'w') as f:
            json.dump({'frames':{'base':args.base,'flange':args.flange,
                                 'vicon_world':args.vicon_world,'marker':args.marker_frame},
                       'poses':samples},f,indent=2)
        print('Saved pose {} ({} samples) to {}'.format(len(samples),len(captures),args.out))
    print('Total poses:',len(samples))


if __name__=='__main__':
    main()
