"""Camera-measured response bounds for the bounded return trial."""
import numpy as np

def camera_response_guard(command,duration,delta):
 u=np.asarray(command,float);d=np.asarray(delta,float)
 if not np.isfinite(d).all():raise RuntimeError('invalid visual response')
 if abs(u[2])>0:
  if d[2]*np.sign(u[2])<.001:raise RuntimeError('no/wrong visual yaw response')
  if abs(d[2])>max(.025,abs(u[2])*duration*1.7+.008):raise RuntimeError('visual yaw displacement budget')
  if np.linalg.norm(d[:2])>.020:raise RuntimeError('unexpected visual translation during yaw')
 else:
  axis=u[:2]/np.linalg.norm(u[:2]);along=float(d[:2]@axis);cross=float(np.linalg.norm(d[:2]-along*axis))
  if along<.0005:raise RuntimeError('no/wrong visual translation response')
  if np.linalg.norm(d[:2])>max(.025,np.linalg.norm(u[:2])*duration*1.7+.008):raise RuntimeError('visual translation displacement budget')
  if cross>.015 or abs(d[2])>.025:raise RuntimeError('unexpected visual cross-axis/yaw motion')

def next_camera_step(error,commission=False,phase='translate'):
 e=np.asarray(error,float);u=np.zeros(3)
 if phase=='yaw':
  if abs(e[2])>.006:
   u[2]=np.copysign(.015,e[2]);return u,min(2.,max(.6,(abs(e[2])-.003)/.015))
  return u,0.
 if np.linalg.norm(e[:2])<=.010:return u,0.
 axis=int(np.argmax(abs(e[:2])))
 speed=.02
 u[axis]=np.copysign(speed,e[axis]);return u,min(2.,max(.6,(abs(e[axis])-.004)/speed))

def ramped_command(command,elapsed):
 return np.asarray(command,float)*min(1.,max(.02,elapsed/.5))

def steering_error(angles,axis):
 a=np.asarray(angles,float)
 if a.shape!=(4,) or not np.isfinite(a).all():raise RuntimeError('invalid steering feedback')
 target=np.pi/2 if axis==1 else 0.
 return float(np.max(abs((a-target+np.pi/2)%np.pi-np.pi/2)))

def preparation_response_guard(delta):
 d=np.asarray(delta,float)
 if not np.isfinite(d).all() or np.linalg.norm(d[:2])>.020 or abs(d[2])>.020:raise RuntimeError('wheel preparation visual displacement limit')

def calibrate_axis_sign(axis,command,delta):
 d=np.asarray(delta,float);u=np.asarray(command,float)
 cross=abs(d[1-axis])
 if not np.isfinite(d).all() or not .004<=abs(d[axis])<=.025 or cross>.015 or abs(d[axis])<cross or abs(d[2])>.015:raise RuntimeError('insufficient/uncontrolled visual drive calibration')
 return 1 if d[axis]*u[axis]>0 else -1
