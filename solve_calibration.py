#!/usr/bin/env python3
"""Solve T_WB and T_EM from ROS1 pose pairs: T_WM = T_WB @ T_BE @ T_EM.

Python 3 with numpy and scipy. No ROS required.
Usage: python3 solve_calibration.py calibration_pairs.json --out calibration_result.json
"""
import argparse
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def mat(entry):
    T=np.eye(4)
    T[:3,:3]=Rotation.from_quat(entry['xyzw']).as_matrix()
    T[:3,3]=np.asarray(entry['xyz'],dtype=float)
    return T


def med_transform(captures,field):
    Ts=[mat(c[field]) for c in captures]
    q=np.array([Rotation.from_matrix(t[:3,:3]).as_rotvec() for t in Ts])
    # All readings at one stationary pose, so local rotvec median is appropriate
    out=np.eye(4)
    out[:3,:3]=Rotation.from_rotvec(np.median(q,axis=0)).as_matrix()
    out[:3,3]=np.median(np.array([t[:3,3] for t in Ts]),axis=0)
    return out


def T_from_x(x):
    T=np.eye(4)
    T[:3,:3]=Rotation.from_rotvec(x[:3]).as_matrix()
    T[:3,3]=x[3:6]
    return T


def x_from_T(T):
    return np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]]


def inv(T):
    U=np.eye(4); U[:3,:3]=T[:3,:3].T; U[:3,3]=-U[:3,:3]@T[:3,3]
    return U


def err_raw(x,pairs):
    W_B=T_from_x(x[:6]); E_M=T_from_x(x[6:]); out=[]
    for B_E,W_M in pairs:
        predicted=W_B@B_E@E_M
        delta=inv(W_M)@predicted
        out.extend(delta[:3,3]); out.extend(Rotation.from_matrix(delta[:3,:3]).as_rotvec())
    return np.array(out)


def solve(pairs):
    # Initial seed: assume marker and flange roughly coincident, so T_EM ~ identity.
    x0=np.r_[x_from_T(pairs[0][1]@inv(pairs[0][0])),np.zeros(6)]
    # Optimize all 12 unknowns simultaneously. Balance meters and radians.
    fun=lambda x,ps: err_raw(x,ps)
    fit=least_squares(fun,x0,args=(pairs,),max_nfev=20000,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    return fit


def run():
    p=argparse.ArgumentParser()
    p.add_argument('input')
    p.add_argument('--out',default='calibration_result.json')
    p.add_argument('--holdout',type=int,default=4,help='Last N poses for independent validation')
    args=p.parse_args()
    with open(args.input) as f: data=json.load(f)
    pairs=[(med_transform(pose['captures'],'B_E'),med_transform(pose['captures'],'W_M')) for pose in data['poses']]
    if len(pairs)<10: raise SystemExit('Collect at least 10 distinct poses; 15-25 with varied orientations recommended.')
    if args.holdout<1 or len(pairs)-args.holdout<6: raise SystemExit('Need at least six fitting poses and one holdout pose.')
    train=pairs[:-args.holdout]; test=pairs[-args.holdout:]
    fit=solve(train)
    W_B=T_from_x(fit.x[:6]); E_M=T_from_x(fit.x[6:])
    results={}
    for name, subset in [('train',train),('holdout',test)]:
        residual=err_raw(fit.x,subset).reshape(-1,6)
        trans=np.linalg.norm(residual[:,:3],axis=1)*1000
        angle=np.linalg.norm(residual[:,3:],axis=1)*180/np.pi
        results[name]={'translation_rmse_mm':float(np.sqrt(np.mean(trans**2))),
                       'translation_max_mm':float(np.max(trans)),
                       'rotation_rmse_deg':float(np.sqrt(np.mean(angle**2))),
                       'rotation_max_deg':float(np.max(angle))}
        print('{}: translation RMSE {:.2f} mm, max {:.2f} mm | rotation RMSE {:.3f} deg, max {:.3f} deg'.format(
            name,results[name]['translation_rmse_mm'],results[name]['translation_max_mm'],
            results[name]['rotation_rmse_deg'],results[name]['rotation_max_deg']))
    if not fit.success: print('WARNING optimizer did not converge:',fit.message)
    print('T_WB (Panda base -> Vicon world):\n',W_B)
    print('T_BW (Vicon world -> Panda base):\n',inv(W_B))
    print('T_EM (flange -> marker cluster):\n',E_M)
    output={'frames':data.get('frames',{}),'num_fit_poses':len(train),'num_holdout_poses':len(test),
            'optimizer_success':bool(fit.success),'T_WB':W_B.tolist(), 'T_BW':inv(W_B).tolist(),
            'T_EM':E_M.tolist(),'errors':results}
    with open(args.out,'w') as f: json.dump(output,f,indent=2)
    print('Saved',args.out)

if __name__=='__main__':run()
