"""50 mm ID 4: parked 5x visual anchor, then a continuous wheel-odometry/IMU return."""
import argparse,collections,hashlib,json,math,os,signal,sys,time
from pathlib import Path
import numpy as np,yaml,rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String,Bool
from move.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from driver.msg import MotorsData
from move.srv import SetParkingMode
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue
from rosidl_runtime_py.convert import message_to_ordereddict as conv
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
from camera_return_core_fast import calibrate_axis_sign
from tag_pixel_v4_single_height_model import continuous_command
from tag_pixel_v4_single_pose_imu import PoseImuTrack, calibration as imu_calibration, control_to_origin, native_pose, startup_command


def pose_difference(a, b):
    return float(np.linalg.norm(np.asarray(a[:2]) - b[:2])), abs(math.remainder(a[2] - b[2], 2 * math.pi))


def main():
 if not __debug__:
  raise RuntimeError('Python assertions must be enabled for the field controller')
 ap=argparse.ArgumentParser();mode=ap.add_mutually_exclusive_group(required=True);mode.add_argument('--check',action='store_true');mode.add_argument('--execute',action='store_true');ap.add_argument('--profile',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);args=ap.parse_args()
 camera_serial=os.environ['APRILTAG_CAMERA_SERIAL']
 cfg=json.loads(args.profile.read_text());assert cfg['additional_clearance_m']==0 and cfg['mode']=='tag_pixel_v3_experimental_return'

 topic=cfg['command_topic']
 assert topic in ['/move/AutoMoveCmd','/move/ManualMoveCmd']
 use_auto=topic=='/move/AutoMoveCmd'
 mission_duration_s = cfg['mission_duration_s']
 max_steps = cfg['max_steps']
 if type(mission_duration_s) is not int or not 150 <= mission_duration_s <= 660:
  raise ValueError('mission_duration_s must be an integer between 150 and 660')
 if type(max_steps) is not int or not 60 <= max_steps <= 180:
  raise ValueError('max_steps must be an integer between 60 and 180')

 def pinned(name):
  path=ROOT/cfg[name]['path'];assert hashlib.sha256(path.read_bytes()).hexdigest()==cfg[name]['sha256'],name+' changed';return path
 ref=json.loads(pinned('reference').read_text());cal=yaml.safe_load(pinned('calibration').read_text());validation=json.loads(pinned('validation').read_text())
 assert cfg['user_authorized_tag40_id2_trial'] and validation['implementation_checks_passed'] and validation['static_origin_verified'] and validation['dynamic_direction_replay_passed'] and validation['base_heading_used'] is False,'closed-loop verification failed'
 for rel,digest in {**validation['source_sha256'],**validation['control_source_sha256']}.items():assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==digest,'validated source changed: '+rel
 directions=json.loads(pinned('directions').read_text());assert directions['reference_sha256']==cfg['reference']['sha256']
 for rel,digest in directions['source_logs_sha256'].items():assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==digest,'direction evidence changed'
 measured_axis_signs={}
 for key,entry in directions['axes'].items():measured_axis_signs[int(key)]=calibrate_axis_sign(int(key),entry['command'],entry['delta'])
 assert cal['planar_sensor_consistency_pass']
 assert ref['camera_usb_serial']==camera_serial and ref['marker']=={'family':'tag36h11','id':0,'black_border_side_m':.16}
 proof=json.loads(pinned('tag40_trial_record').read_text())
 assert proof['marker']=={'family':'tag36h11','id':4,'black_border_side_m':.05}
 assert proof['original_reference_id']==ref['reference_id'] and proof['scale']==5
 assert cfg['max_linear_mps']==.02 and cfg['max_yaw_radps']==.025
 out=args.output_dir;out.mkdir(exist_ok=True);result_path=out/('result_execute.json' if args.execute else 'result_check.json');assert not result_path.exists(),'refuse repeated execution/result overwrite'
 log=(out/('steps_execute.jsonl' if args.execute else 'steps_check.jsonl')).open('x')
 rclpy.init(args=['--ros-args','--log-level','fatal']);n=rclpy.create_node('apriltag_planar_return_trial');latest={};rx={};states=collections.deque();corner_records=collections.deque(maxlen=150);motor_trace=collections.deque(maxlen=2500);observations=collections.deque();foreign=[];interrupted=[]
 pub=None;hb=None;armed=False;wd_ready=False;last_beat=0.;owned_stamps=set();old_auto=None;may_move=False;mission_start=None;travel=0.;parking_transition=False;steering_preparation=False;last_visual=None;visual_records=collections.deque();seq=0
 report={'execute':args.execute,'session':cfg['session'],'status':'preflight','motion_started':False,'nonzero_commands':0,'additional_clearance_m':0.,'pointcloud_subscribed':False,'local_avoidance_enabled':False,'full_optical_tf_enabled':False,'absolute_accuracy_verified':False,'base_heading_used':False,'visual_heading_source':'id4_50mm_lanczos_sharpened_start_anchor','fixed_heading_assumption':False,'physical_tape_measure_acceptance_pending':True,'command_timing':'continuous wheel/IMU feedback, updated every 0.15 s','travel_m_is_independently_measured':False,'goal_policy':{'local_model_budget_m':cfg['goal_model_budget_m'],'corner_rms_limit_px':cfg['goal_pixel_rms_limit'],'physical_accuracy_requires_external_measurement':True},'steps':[],'cleanup':{}}
 report['mission_duration_s'] = mission_duration_s
 report['max_steps'] = max_steps
 report['motion_kind']='single_start_tag_wheel_imu_continuous'
 report['marker']=proof['marker']
 report['physical_accuracy_assumed']=False
 report['visual_branch_selection']='wall-normal match at reference and lowest current 5x corner residual'
 report['vision_during_motion']=False
 report['anchor_while_parked']=True
 report['wheel_pose_quaternion_used']=False
 report['translation_source']='/controller/odom native accumulated x/y, rotated from chassis heading to IMU heading'
 report['native_yaw_encoding']='orientation.w is radians per controller_odom_bridge.py'
 report['startup_ramp_s']=2.0
 report['yaw_source']='/driver/imu/Data projected gyro with stationary bias correction'
 tracker=None;vision_frozen=False;feedback_fault=[];imu_history=collections.deque(maxlen=500)
 startup_trace=collections.deque(maxlen=180);motion_start_wall_ns=None
 report['command_topic']=topic

 def emit(kind,**kw):
  row={'event':kind,'wall_ns':time.time_ns(),**kw};log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
  print(json.dumps({key: value for key, value in row.items() if key != 'steering_trace'}), flush=True)
 def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
 def vel():
  m=latest['base'].speed;return np.array([m.linear.x,m.linear.y,m.angular.z])
 def keep(k,m):latest[k]=m;rx[k]=time.monotonic()
 def base(m):
  keep('base',m);states.append((time.monotonic(),time.time_ns(),np.array([m.speed.linear.x,m.speed.linear.y,m.speed.angular.z])))
  while states and time.monotonic()-states[0][0]>3:states.popleft()
 def motor(m):
  keep('motors',m);motor_trace.append((time.monotonic(),[float(x.position) for x in m.steer_data],[float(x.speed) for x in m.steer_data]))
 def xyz(vector):return np.array([vector.x,vector.y,vector.z],float)
 def pose_of(m):
  p=m.position;q=m.orientation
  return native_pose((p.x,p.y),(q.x,q.y,q.z,q.w))
 def feedback_row(m):
  p=m.pose.pose;q=p.orientation
  return {'stamp_ns':stamp(m),'native_xy_yaw':[p.position.x,p.position.y,q.w],
          'raw_twist':xyz(m.twist.twist.linear).tolist(),
          'base_feedback':vel().tolist(),
          'steering_positions':[float(x.position) for x in latest['motors'].steer_data]}
 def inertial(m):
  keep('imu',m)
  sample=(stamp(m),xyz(m.angular_velocity),xyz(m.linear_acceleration))
  imu_history.append(sample)
  if tracker is not None and not feedback_fault:
   try:
    if m.header.frame_id!='imu':raise ValueError('IMU frame changed')
    tracker.imu(sample[0],sample[1])
   except ValueError as ex:feedback_fault.append(str(ex))
 def wheel(m):
  keep('wheel',m)
  if tracker is not None and not feedback_fault:
   try:
    if m.header.frame_id!='odom' or m.child_frame_id!='base_link':raise ValueError('wheel odometry frame changed')
    if motion_start_wall_ns is not None and time.time_ns()-motion_start_wall_ns<4_000_000_000:
     if not startup_trace or stamp(m)-startup_trace[-1]['stamp_ns']>=25_000_000:
      startup_trace.append(feedback_row(m))
    tracker.wheel(stamp(m),pose_of(m.pose.pose))
   except ValueError as ex:
    feedback_fault.append(str(ex))
    report['odometry_at_fault']=conv(m)
 def refresh_visual():
  nonlocal last_visual
  tagpath=out/'vision/tag_live.json'
  if tagpath.exists():
   tag=json.loads(tagpath.read_text());age=(time.time_ns()-tag.get('image_stamp_ns',0))/1e9
   if tag.get('corners') is not None and (not corner_records or tag['image_stamp_ns']!=corner_records[-1][0]):corner_records.append((tag['image_stamp_ns'],tag['corners']))
   if tag.get('state')=='valid' and tag.get('serial')==camera_serial and tag.get('tag_id')==4 and 0<=age<.8:rx['valid_tag']=time.monotonic()-age
  path=out/'vision/visual_live.json'
  if not path.exists():return
  row=json.loads(path.read_text())
  if row.get('state')!='valid':return
  assert row['source_sha256']==validation['source_sha256'],'vision worker source mismatch'
  assert row['base_heading_used'] is False and row['tag_id']==4,'wrong ID 4 visual source'
  if last_visual is None or row['image_stamp_ns']!=last_visual['image_stamp_ns']:
   last_visual=row;visual_records.append(row)
   while len(visual_records)>30:visual_records.popleft()

 def command(m,key):
  u=np.array([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z])
  if np.max(np.abs(u))>1e-6 and not (key==('auto' if use_auto else 'manual') and stamp(m) in owned_stamps):foreign.append(key)
 def wd(m):
  try:
   a=json.loads(m.data)
   if a['session']==cfg['session']:latest['watchdog']=a;rx['watchdog']=time.monotonic()
  except (ValueError,KeyError):pass
 subs=[n.create_subscription(State,'/move/State',base,qos_profile_sensor_data),n.create_subscription(MotorsData,'/driver/motor/Data',motor,qos_profile_sensor_data),n.create_subscription(String,'/apriltag_return_trial/watchdog_status',wd,10)]
 subs.append(n.create_subscription(Odometry,'/controller/odom',wheel,qos_profile_sensor_data))
 subs.append(n.create_subscription(Imu,'/driver/imu/Data',inertial,qos_profile_sensor_data))
 for incoming_topic,key in [('/move/ManualMoveCmd','manual'),('/move/AutoMoveCmd','auto')]:subs.append(n.create_subscription(TwistStamped,incoming_topic,lambda m,k=key:command(m,k),qos_profile_sensor_data))
 def standby(m,key):keep(key,bool(m.data))
 for suffix in ('active','shadow_active'):
  subs.append(n.create_subscription(Bool,'/k05/wait_w1/'+suffix,lambda m,k='k05_'+suffix:standby(m,k),qos_profile_sensor_data))
 signal.signal(signal.SIGTERM,lambda *_:interrupted.append(True));signal.signal(signal.SIGINT,lambda *_:interrupted.append(True));signal.signal(signal.SIGHUP,lambda *_:interrupted.append(True))
 def beat(force=False):
  nonlocal last_beat
  if hb is not None and (force or time.monotonic()-last_beat>.05):hb.publish(String(data=json.dumps({'session':cfg['session'],'armed':armed,'wall_ns':time.time_ns()})));last_beat=time.monotonic()
 def check():
  nonlocal travel
  if not vision_frozen:
   refresh_visual()
   assert 'valid_tag' in rx and 0<=time.monotonic()-rx['valid_tag']<.8,'stale/missing valid_tag'
   assert 0<=time.time_ns()-(out/'runtime/latest.npz').stat().st_mtime_ns<350_000_000,'camera capture stalled'
  now=time.monotonic()
  for key,limit in [('base',.25),('motors',.25),('wheel',.15),('imu',.15)]:
   assert key in rx and 0<=now-rx[key]<limit,'stale/missing '+key
  for key in ('wheel','imu'):
   assert -10_000_000<=time.time_ns()-stamp(latest[key])<150_000_000,'stale feedback header '+key
  assert not feedback_fault,'wheel/IMU tracking fault: '+str(feedback_fault[-1:])
  assert not interrupted,'operator/signal interrupted';assert not foreign,'competing nonzero '+str(foreign[-1:])
  b=latest['base'];assert b.is_hard_auto and b.is_soft_auto==use_auto and not b.is_handle_on and not b.is_joystick_on,'physical/operator/mode override'
  assert abs(time.time_ns()-stamp(b))<250_000_000 and abs(time.time_ns()-stamp(latest['motors']))<250_000_000,'stale header'
  if parking_transition or steering_preparation:assert np.isfinite(vel()).all(),'nonfinite steering-transition feedback'
  else:assert np.isfinite(vel()).all(),'nonfinite feedback'
  motors=latest['motors'];assert len(motors.wheel_data)==4 and len(motors.steer_data)==4 and all(m.is_connected and m.is_enable and m.error_code==0 for m in list(motors.wheel_data)+list(motors.steer_data)),'motor health'
  waist=json.loads((out/'runtime/waist_live.json').read_text());assert 0<=time.time_ns()-waist['wall_ns']<2_500_000_000 and np.max(abs(np.array(waist['counts'])-np.array(cfg['waist_counts'])))<=100,'waist pose stale/changed'
  if wd_ready:assert now-rx.get('watchdog',-999)<.3 and latest['watchdog']['state']=='armed','watchdog unavailable'
  if mission_start is not None and now - mission_start >= mission_duration_s:
   raise TimeoutError('mission deadline')
 def spin(seconds,strict=True):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   rclpy.spin_once(n,timeout_sec=.01);beat()
   if strict:check()
 def call(typ,name,q):
  c=n.create_client(typ,name)
  try:
   assert c.wait_for_service(timeout_sec=1),name+' unavailable';f=c.call_async(q);end=time.monotonic()+2
   while not f.done() and time.monotonic()<end:spin(.01,False)
   assert f.done() and f.result() is not None,name+' timeout';a=f.result();assert not getattr(a,'error_code',0),str(a);return a
  finally:n.destroy_client(c)
 def get_auto():
  v=call(GetParameters,'/steward/get_parameters',GetParameters.Request(names=['robot.enable_auto_parking_mode'])).values
  assert len(v)==1 and v[0].type==1,'auto parking parameter unavailable';return v[0].bool_value
 def set_auto(v):
  q=SetParameters.Request(parameters=[Parameter(name='robot.enable_auto_parking_mode',value=ParameterValue(type=1,bool_value=v))]);a=call(SetParameters,'/steward/set_parameters',q)
  assert len(a.results)==1 and a.results[0].successful and get_auto()==v,'auto parking write/readback failed'
 def park(v):return call(SetParkingMode,'/move/SetParkingMode',SetParkingMode.Request(enable=v))
 def send(u):
  nonlocal motion_start_wall_ns
  assert pub is not None
  assert np.isfinite(u).all() and np.linalg.norm(u[:2])<=.010001 and abs(u[2])<=.008001,'command exceeds return limit'
  if parking_transition:assert np.max(abs(u))<1e-7,'nonzero command forbidden during parking transition'
  if steering_preparation:assert np.linalg.norm(u[:2])<=.006001 and abs(u[2])<1e-9,'preparation command bound'
  m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=float(u[0]);m.twist.linear.y=float(u[1]);m.twist.angular.z=float(u[2]);owned_stamps.add(stamp(m));pub.publish(m)
  if np.max(abs(u))>1e-7:
   if not report['motion_started']:
    motion_start_wall_ns=time.time_ns();emit('MOTION_STARTED',command=u.tolist())
   report['nonzero_commands']+=1;report['motion_started']=True
 def zeros(seconds=.5):
  end=time.monotonic()+seconds
  while time.monotonic()<end:send(np.zeros(3));spin(.02,False)
 from tag_pixel_v4_single_tag50_pitch_anchor import Tag50PitchStart
 model=Tag50PitchStart(proof['geometry'],proof['reference_pose'],proof['start_yaw_rad'])
 model.reference_points *= .05 / .04
 report['position_source']=proof['position_source']
 report['apparent_camera_pitch_deg']=proof['apparent_pitch_offset_deg']
 report['odometry_position_comparison_source']=proof['odometry_position_comparison_source']
 report['base_heading_used']=True
 report['visual_heading_source']='50 mm origin and current parked native odometry'
 report['visual_branch_selection']='translation fitted from ID 4 using archived origin odometry for yaw'
 expected_corners=np.asarray(proof['start_corners_raw_px'])

 def fit_small_tag(corners):
  corner_shift=float(np.sqrt(np.mean(np.sum((np.asarray(corners)-expected_corners)**2,axis=1))))
  if corner_shift>.25:
   raise ValueError('live ID 4 corners no longer match the surveyed start')
  return model.fit(corners)

 def observe():
  native=verified_native_pose()
  expected=proof['start_native_odom']
  assert np.linalg.norm(native[0]-expected[:2])<=.012,'start moved after its heading was checked'
  assert abs(math.remainder(native[1]-expected[2],2*math.pi))<=.009,'start heading changed'
  begin=time.time_ns()
  deadline=time.monotonic()+16
  while time.monotonic()<deadline:
   spin(.03)
   now=time.monotonic()
   stationary=[row for row in states if now-row[0]<1.1]
   if not stationary or now-stationary[0][0]<1.0 or any(np.max(abs(row[2]))>.003 for row in stationary):
    continue
   rows=[row for row in visual_records if row['window_start_stamp_ns']>begin
         and 0<=time.time_ns()-row['image_stamp_ns']<4_000_000_000]
   if len(rows)<8 or rows[-1]['image_stamp_ns']-rows[0]['image_stamp_ns']<500_000_000:
    continue
   corners=[row['corners_by_scale']['5'] for row in rows]
   pose,rms=fit_small_tag(np.median(corners,axis=0))
   first,_=fit_small_tag(corners[0])
   last,_=fit_small_tag(corners[-1])
   dx,dyaw=pose_difference(first,last)
   if dx>.02 or math.degrees(dyaw)>.5:
    continue
   return {'pose':pose.tolist(),'rms_px':rms,'image_stamp_ns':rows[-1]['image_stamp_ns'],
           'pose_window_count':len(rows),'short_repeat_xy_m':dx,'short_repeat_yaw_deg':math.degrees(dyaw)}
  raise RuntimeError('no fresh settled ID 4 measurement')

 def ownership():
  assert [x.node_name for x in n.get_publishers_info_by_topic('/controller/odom')]==['controller_node'],'wheel odometry publisher changed'
  assert [x.node_name for x in n.get_publishers_info_by_topic('/driver/imu/Data')]==['driver_node'],'IMU publisher changed'
  assert (latest['wheel'].header.frame_id,latest['wheel'].child_frame_id,latest['imu'].header.frame_id)==('odom','base_link','imu'),'feedback frames changed'
  owners=n.get_publishers_info_by_topic('/k05/wait_w1/goal_xyyaw')
  if owners:
   assert all(x.node_name=='live_rectangular_roundtrip_execution' for x in owners),'unexpected navigation goal owner'
   assert all(time.monotonic()-rx.get(key,-1e9)<.5 and latest.get(key) is False
              for key in ('k05_active','k05_shadow_active')),'K05 standby is active or stale'
  allowed={n.get_name(),'apriltag_trial_independent_stop_watchdog'}
  if use_auto:
   assert all(x.node_name in allowed for x in n.get_publishers_info_by_topic(topic)),'automatic command topic has another publisher'
  else:
   assert not n.get_publishers_info_by_topic('/move/AutoMoveCmd'),'another automatic command publisher exists'

 def verified_native_pose():
  wheel=pose_of(latest['wheel'].pose.pose)
  state=pose_of(latest['base'].odometry)
  assert np.linalg.norm(wheel[0]-state[0])<.01,'native wheel pose disagrees with chassis state while stationary'
  assert abs(math.remainder(wheel[1]-state[1],2*math.pi))<.02,'native wheel yaw disagrees with chassis state while stationary'
  return wheel
 def plan(audit):
  pose=np.array(audit['pose'])
  rms=audit['rms_px']
  dx,dyaw=pose_difference(pose,proof['primary_pose_xy_yaw_height'])
  assert dx<=.03 and math.degrees(dyaw)<=1., 'fresh tag pose differs from the independently sampled visual start'
  assert rms<=.35,'ID 4 corner fit exceeded 0.35 px'
  u,duration,displacement=continuous_command(pose,cfg['nominal_motion_gain'])
  assert np.linalg.norm(displacement)<=.75 and abs(pose[2])<=math.radians(30) and duration<=90,'planned return exceeds the checked ID 4 trial envelope'
  assert np.linalg.norm(u[:2])<=.010001 and abs(u[2])<=.010001,'velocity cap'
  return pose,u,duration,{'initial_pose_xy_yaw_height':pose.tolist(),'fit_rms_px':rms,
                         'return_body_xy_m':displacement.tolist(),'command':u.tolist(),'duration_s':duration,'visual_audit':audit}

 def freeze_vision():
  nonlocal vision_frozen
  vision_frozen=True
  request=out/'runtime/stop_vision.request'
  with request.open('x') as f:f.write(cfg['session'])
  end=time.monotonic()+5
  cap=out/'runtime/capture_summary.json';worker=out/'vision/worker_summary.json'
  while time.monotonic()<end:
   send(np.zeros(3));spin(.03)
   if cap.exists() and worker.exists():
    try:
     camera=json.loads(cap.read_text());frames=json.loads(worker.read_text())
    except ValueError:
     continue
    assert camera['released'],'camera release failed'
    report['camera_stopped_before_motion']=True
    report['final_tag_counts']=frames
    emit('VISION_FROZEN',capture_frames=camera['frames'],tag_counts=frames)
    return
  raise RuntimeError('Camera and tag worker did not shut down before motion')

 def drive():
  begin=time.monotonic();last_log=begin;last_owner=begin
  command_time=-1e9;progress_time=begin;best=math.inf
  command=np.zeros(3);initial_command=control_to_origin(tracker.pose)[0]
  emit('STARTUP_RAMP',full_command=initial_command.tolist(),duration_s=2.0)
  while True:
   spin(.015)
   assert not latest['base'].is_parking_mode,'unexpected parking during odometry return'
   now=time.monotonic()
   assert tracker.distance<=1.35,'integrated travel exceeded ID 4 trial envelope'
   if now-last_owner>=.5:
    ownership();last_owner=now
   if now-command_time>=.15:
    command,distance,heading=control_to_origin(tracker.pose)
    command_time=now
    if now-last_log>=3 or not np.any(command):
     emit('ODOMETRY_PROGRESS',elapsed_s=now-begin,remaining_xy_m=distance,
          remaining_yaw_deg=math.degrees(heading),pose=tracker.pose.tolist(),
          wheel_travel_m=tracker.distance,command=command.tolist())
     last_log=now
    if not np.any(command):
     return 'odometry_goal_reached'
    cost=distance+.2*abs(heading)
    if cost<best-.006:
     best=cost;progress_time=now
    assert now-progress_time<9,'wheel/IMU target error did not decrease'
   send(startup_command(initial_command,now-begin+.015) if now-begin<2.0 else command)

 try:
  spin(2.5,False);check();ownership()
  assert latest['base'].is_parking_mode and np.max(abs(vel()))<.003,'initial parked standstill required'
  old_auto=get_auto()
  report['before_base']=conv(latest['base']);report['original_auto_parking']=old_auto
  native=verified_native_pose();report['preflight_native_pose']=[*native[0],native[1]]
  controller=call(GetParameters,'/controller_node/get_parameters',GetParameters.Request(names=['emergency_stop_distance']))
  v=controller.values;assert len(v)==1 and v[0].double_value>=.099,'unexpected emergency parameter'
  report['manufacturer_emergency_stop_distance_m']=v[0].double_value
  report['subscriptions']=[s.topic_name for s in subs]
  if not args.execute:
   audit=observe()
   initial_pose,initial_u,initial_duration,initial_plan=plan(audit)
   report['initial_plan']=initial_plan
   up,bias=imu_calibration(list(imu_history)[-300:])
   emit('PREFLIGHT_PASS',execute=False,plan=initial_plan,imu_up=up.tolist(),gyro_bias_radps=bias,
        wheel_frame=latest['wheel'].child_frame_id,native_pose=report['preflight_native_pose'],
        pointcloud_subscribed=False,local_avoidance_enabled=False)
   report['status']='check_passed';return
  emit('PREFLIGHT_PASS',execute=True,wheel_frame=latest['wheel'].child_frame_id,
       pointcloud_subscribed=False,local_avoidance_enabled=False)
  with (out/'execution.claim').open('x') as f:f.write(json.dumps({'session':cfg['session'],'wall_ns':time.time_ns()}))
  hb=n.create_publisher(String,'/apriltag_return_trial/heartbeat',10);armed=True;beat(True)
  end=time.monotonic()+1
  while time.monotonic()<end and (time.monotonic()-rx.get('watchdog',-999)>.2 or latest.get('watchdog',{}).get('state')!='armed'):spin(.03,False)
  assert latest.get('watchdog',{}).get('state')=='armed','watchdog did not arm';wd_ready=True
  pub=n.create_publisher(TwistStamped,topic,10);spin(.2);assert pub.get_subscription_count()>0,'no chassis command subscriber'
  assert latest['base'].is_parking_mode,'initial anchor must be taken while parked'
  audit=observe()
  pose,u,duration,initial_plan=plan(audit);ownership()
  report['initial_plan']=initial_plan
  if np.linalg.norm(pose[:2])<=.015 and abs(pose[2])<=.0175:
   raise RuntimeError('Starting tag measurement is already inside the odometry target')
  samples=[row for row in imu_history if stamp(latest['imu'])-row[0]<=1_400_000_000]
  up,bias=imu_calibration(samples)
  wheel_msg=latest['wheel'];imu_msg=latest['imu']
  assert np.linalg.norm(xyz(wheel_msg.twist.twist.linear)[:2])<.003,'wheel odometry not stationary at vision anchor'
  native=verified_native_pose()
  tracker=PoseImuTrack(pose,up,bias,stamp(imu_msg),xyz(imu_msg.angular_velocity),
                       stamp(wheel_msg),native)
  emit('ODOMETRY_ANCHOR',tag_plan=initial_plan,gyro_bias_radps=bias,imu_up=up.tolist(),
       native_pose=[*native[0],native[1]],anchor_while_parked=True)
  freeze_vision()
  may_move=True;parking_transition=True;release_started=time.monotonic();set_auto(False)
  for attempt in range(3):
   if not latest['base'].is_parking_mode:break
   park(False);end=time.monotonic()+1.0
   while latest['base'].is_parking_mode and time.monotonic()<end:spin(.03)
   emit('PARKING_RELEASE_WAIT',attempt=attempt+1,parked=latest['base'].is_parking_mode)
  assert not latest['base'].is_parking_mode,'parking release failed'
  quiet=None;release_deadline=time.monotonic()+4
  while time.monotonic()<release_deadline:
   send(np.zeros(3));spin(.02)
   if np.max(abs(vel()))<.003:
    if quiet is None:quiet=time.monotonic()
    if time.monotonic()-quiet>.5 and time.monotonic()-release_started>1.:break
   else:quiet=None
  else:raise RuntimeError('parking-release steering did not settle')
  parking_transition=False;mission_start=time.monotonic()
  report['stop_reason']=drive()
  report['status']='one_motion_ended'

 except Exception as exc:
  if 'base' in latest:report['abort_base']=conv(latest['base'])
  report['status']='aborted';report['error']=type(exc).__name__+': '+str(exc);emit('ABORT',error=report['error'])
 finally:
  mission_start = None  # Active movement has ended. Keep fresh-data checks during parking.
  if pub is not None:
   try:zeros(.8);report['cleanup']['zeros_sent']=True
   except Exception as ex:report['cleanup']['zero_error']=str(ex)
  if may_move:
   try:
    assert 'base' in rx and time.monotonic()-rx['base']<.25 and np.max(abs(vel()))<.005,'stop unconfirmed'
    report['cleanup']['stopped_before_parking']=True
    if latest['base'].is_hard_auto and not latest['base'].is_joystick_on and not latest['base'].is_handle_on:
     park(True);end=time.monotonic()+2
     while time.monotonic()<end and not latest['base'].is_parking_mode:spin(.03,False)
     assert latest['base'].is_parking_mode,'parking unconfirmed';report['cleanup']['parking_confirmed']=True
     end=time.monotonic()+3;still_since=None
     while time.monotonic()<end:
      spin(.03,False)
      if time.monotonic()-rx.get('base',-999)<.25 and np.max(abs(vel()))<.003:
       if still_since is None:still_since=time.monotonic()
       if time.monotonic()-still_since>.35:break
      else:still_since=None
     assert still_since is not None and time.monotonic()-still_since>.35,'post-parking stop unconfirmed'
     report['cleanup']['stopped_verified']=True
    if old_auto is not None:set_auto(old_auto);report['cleanup']['auto_parking_restored']=True
   except Exception as ex:report['cleanup']['stop_or_restore_error']=str(ex)
  if tracker is not None:
   report['final_wheel_imu_pose']=tracker.pose.tolist()
   report['final_wheel_imu_distance_m']=float(np.linalg.norm(tracker.pose[:2]))
   report['wheel_integrated_travel_m']=tracker.distance
  report['startup_feedback']=list(startup_trace)
  if feedback_fault:report['wheel_imu_feedback_fault']=feedback_fault
  if report['status']=='one_motion_ended':
   if all(report['cleanup'].get(key) for key in ('stopped_verified','parking_confirmed','auto_parking_restored')):
    report['status']='one_motion_parked_measurement_pending'
    emit('RETURN_COMPLETE',stop_reason=report['stop_reason'],wheel_imu_xy_m=report['final_wheel_imu_distance_m'])
   else:
    report['status']='stopped_not_accepted';report['postcheck_error']='parking not verified'
  if hb is not None:armed=False;beat(True);spin(.15,False)
  if 'base' in latest:report['final_base']=conv(latest['base'])
  report['travel_m']=travel;report['foreign_nonzero']=foreign
  result_path.write_text(json.dumps(report,indent=2,allow_nan=False));emit('FINAL',status=report['status'],nonzero_commands=report['nonzero_commands'],cleanup=report['cleanup']);log.close();n.destroy_node();rclpy.shutdown()
  if report['status'] not in ['check_passed','one_motion_parked_measurement_pending']:raise SystemExit(2)

if __name__=='__main__':main()
