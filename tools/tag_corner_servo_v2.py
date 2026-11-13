"""Single-AprilTag image feedback and honest pose ambiguity diagnostics.

No ROS, hardware I/O, scene features, odometry or fixed-heading state. Units of
the proposed body correction are metres, metres and radians. Proposals are NOT
motion commands and this module cannot move a robot.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np

TAG_POINTS = np.array([[-.08,-.08,0.],[.08,-.08,0.],
                       [.08,.08,0.],[-.08,.08,0.]])

def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))

def rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])

@dataclass
class Geometry:
    K: np.ndarray
    D: np.ndarray
    Rbc: np.ndarray
    txy: np.ndarray

    def normalize(self,pixels):
        q=(np.asarray(pixels,float)-.5-self.K[:2,2])/np.array([self.K[0,0],self.K[1,1]])
        x0,y0=q[...,0],q[...,1];x,y=x0.copy(),y0.copy()
        for _ in range(10):
            rr=x*x+y*y;inv=1/(1+((self.D[4]*rr+self.D[1])*rr+self.D[0])*rr)
            a,b=x/inv,y/inv
            x=(x0-2*self.D[2]*a*b-self.D[3]*(rr+2*a*a))*inv
            y=(y0-2*self.D[3]*a*b-self.D[2]*(rr+2*b*b))*inv
        return np.stack([x,y],axis=-1)

    def project(self,points):
        points=np.asarray(points,float)
        if not np.isfinite(points).all() or np.any(points[...,2]<=0):
            raise ValueError('nonfinite or behind-camera tag geometry')
        x,y=(points[...,:2]/points[...,2,None]).T;rr=x*x+y*y
        f=1+((self.D[4]*rr+self.D[1])*rr+self.D[0])*rr;a,b=x*f,y*f
        return np.c_[a+2*self.D[2]*a*b+self.D[3]*(rr+2*a*a),
                      b+2*self.D[3]*a*b+self.D[2]*(rr+2*b*b)]*np.array([self.K[0,0],self.K[1,1]])+self.K[:2,2]+.5

    def camera_after_body_move(self,points,move):
        """The tag is fixed; the camera moves with the base, including its lever arm."""
        x,y,a=np.asarray(move,float);t=np.r_[self.txy,0.]
        return ((np.asarray(points)@self.Rbc.T+t-np.array([x,y,0.]))@rz(-a).T-t)@self.Rbc

    def hypotheses(self,corners):
        c=np.asarray(corners,float)
        if c.shape!=(4,2) or not np.isfinite(c).all():
            raise ValueError('exactly four finite native LB,RB,RT,LT tag corners required')
        r=cv2.solvePnPGeneric(TAG_POINTS[[3,2,1,0]],self.normalize(c[[3,2,1,0]]),
                              np.eye(3),np.zeros(5),flags=cv2.SOLVEPNP_IPPE_SQUARE)
        out=[]
        for rv,tv in zip(r[1],r[2]):
            R=cv2.Rodrigues(rv)[0];p=TAG_POINTS@R.T+tv.ravel()
            if not np.isfinite(p).all() or min(p[:,2])<=0:
                continue
            residual=self.project(p)-c
            out.append({'R':R,'t':tv.ravel(),'points':p,
                        'corner_rms_px':float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))})
        # Keep BOTH. Small reprojection-error differences are not a heading truth.
        return sorted(out,key=lambda h:h['corner_rms_px'])

    def interaction(self,points):
        """Pixel displacement per actual body displacement, x/y/yaw simultaneously."""
        columns=[]
        for axis in range(3):
            d=np.zeros(3);d[axis]=1e-5
            plus=self.project(self.camera_after_body_move(points,d))
            minus=self.project(self.camera_after_body_move(points,-d))
            columns.append(((plus-minus)/(2e-5)).ravel())
        return np.array(columns).T

    def relative_hypotheses(self,goal,current):
        out=[]
        for i,g in enumerate(goal):
            for j,c in enumerate(current):
                Rrel=self.Rbc@c['R']@g['R'].T@self.Rbc.T
                yaw=math.atan2(Rrel[1,0],Rrel[0,0])
                # Do not silently discard candidates using an unverified gravity
                # or planar-motion tolerance. Report their mismatch explicitly.
                off_axis=cv2.Rodrigues(rz(-yaw)@Rrel)[0].ravel()
                q=self.Rbc@c['t'];r=self.Rbc@g['t']
                xy=q[:2]+self.txy-rz(yaw)[:2,:2]@(r[:2]+self.txy)
                out.append({'goal_branch':i,'current_branch':j,'xy_m':xy.tolist(),
                            'yaw_rad':yaw,'off_axis_rotation_rad':float(np.linalg.norm(off_axis)),
                            'goal_reprojection_rms_px':g['corner_rms_px'],
                            'current_reprojection_rms_px':c['corner_rms_px']})
        return out

def image_diagnostic(geometry,goal_corners,current_corners,observed_noise_px):
    """Every call recomputes from the current four corners; no held heading.

    Noise sensitivity is a local calculation, not an absolute error certificate.
    The measured noise budget is an INPUT with provenance, not a hidden constant.
    """
    if not math.isfinite(observed_noise_px) or observed_noise_px<=0:
        raise ValueError('positive measured pixel noise required')
    goal_corners=np.asarray(goal_corners,float);current_corners=np.asarray(current_corners,float)
    goal=geometry.hypotheses(goal_corners);current=geometry.hypotheses(current_corners)
    if not goal or not current:
        raise ValueError('no forward-facing tag hypotheses')
    error=(current_corners-goal_corners).ravel();proposals=[]
    length=float(np.mean([np.linalg.norm(h['t']) for h in current]))
    scale=np.diag([1.,1.,1./length])
    for h in current:
        J=geometry.interaction(h['points']);A=J@scale
        U,s,Vt=np.linalg.svd(A,full_matrices=False)
        # Un-damped correction is diagnostic only; it must never be sent as a
        # velocity. Retaining the weak component makes the uncertainty visible.
        correction=-scale@np.linalg.pinv(A)@error
        covariance=(np.linalg.pinv(J)*observed_noise_px)@(np.linalg.pinv(J)*observed_noise_px).T
        xy_radius=2.448*math.sqrt(max(np.linalg.eigvalsh(covariance[:2,:2])))
        proposals.append({'correction_xyyaw':correction.tolist(),
                          'singular_values_px_per_scaled_m':s.tolist(),
                          'condition_number_scaled':float(s[0]/s[-1]),
                          'local_xy_95_noise_radius_m':xy_radius,
                          'local_yaw_noise_std_rad':float(math.sqrt(covariance[2,2])),
                          'weak_direction_xy_range_times_yaw':Vt[-1].tolist(),
                          'J_pixels_per_m_m_rad':J.tolist()})
    hypotheses=geometry.relative_hypotheses(goal,current)
    xs=np.array([h['xy_m'] for h in hypotheses]);ys=np.unwrap([h['yaw_rad'] for h in hypotheses])
    diameter=float(np.max(np.linalg.norm(xs[:,None]-xs[None,:],axis=2)))
    return {'corner_rms_to_goal_px':float(np.sqrt(np.mean(np.sum(error.reshape(4,2)**2,axis=1)))),
            'corner_error_px':error.reshape(4,2).tolist(),
            'heading_source':'fresh_four_corner_hypotheses_each_observation',
            'fixed_heading_assumption':False,'pointcloud_used':False,'scene_features_used':False,
            'pixel_noise_budget_from_observation_px':observed_noise_px,
            'relative_pose_hypotheses':hypotheses,'pose_hypothesis_xy_diameter_m':diameter,
            'pose_hypothesis_yaw_span_rad':float(np.ptp(ys)),
            'image_feedback_proposals_not_commands':proposals,
            'all_branches_within_user_5cm_target':diameter<=.05,
            'local_noise_sensitivity_within_user_5cm_target':max(p['local_xy_95_noise_radius_m'] for p in proposals)<=.05,
            'absolute_accuracy_certified':False}

def validate_motion_permit(report):
    """A user-requested validation result; not a replacement for pose verification."""
    reasons=[]
    if not report.get('implementation_regressions_passed'):reasons.append('implementation not verified')
    if not report.get('data_replay_feedback_passed'):reasons.append('real-image response replay not verified')
    if not report.get('all_branches_within_user_5cm_target'):reasons.append('unresolved pose alternatives exceed 5 cm')
    if not report.get('local_noise_sensitivity_within_user_5cm_target'):reasons.append('measured image uncertainty exceeds 5 cm')
    return {'motion_permitted':not reasons,'reasons':reasons}
