"""Independent process: publishes only zero velocities and optional safe parking."""
import argparse,json,time
from pathlib import Path
import numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from move.msg import State
from move.srv import SetParkingMode

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--session',required=True);ap.add_argument('--output',type=Path,required=True)
 ap.add_argument('--duration', type=int, required=True)
 ap.add_argument('--command-topic', choices=['/move/ManualMoveCmd','/move/AutoMoveCmd'], required=True)
 args = ap.parse_args()
 if not 225 <= args.duration <= 735:
  ap.error('--duration must be between 225 and 735 seconds')
 assert not args.output.exists();rclpy.init(args=['--ros-args','--log-level','fatal']);n=rclpy.create_node('apriltag_trial_independent_stop_watchdog')
 command=n.create_publisher(TwistStamped,args.command_topic,10);status=n.create_publisher(String,'/apriltag_return_trial/watchdog_status',10)
 armed=False;ever=False;last=None;finish=False;fault=None;base=None;base_rx=None;start=time.monotonic();result={'session':args.session,'zero_commands':0,'park_requested':False}
 def state(m):
  nonlocal base,base_rx
  base=m;base_rx=time.monotonic()
 def heartbeat(m):
  nonlocal armed,ever,last,finish
  try:
   d=json.loads(m.data)
   if d['session']!=args.session or not 0<=time.time_ns()-d['wall_ns']<250_000_000:return
   if d['armed']:armed=True;ever=True;last=time.monotonic()
   elif ever:armed=False;finish=True
  except (KeyError,ValueError):return
 subs=[n.create_subscription(String,'/apriltag_return_trial/heartbeat',heartbeat,10),n.create_subscription(State,'/move/State',state,qos_profile_sensor_data)]
 def publish_status(v):status.publish(String(data=json.dumps({'session':args.session,'state':v,'wall_ns':time.time_ns()})))
 def zero():
  m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link';command.publish(m);result['zero_commands']+=1
 try:
  while time.monotonic() - start < args.duration:
   rclpy.spin_once(n,timeout_sec=.02)
   if armed:
    publish_status('armed')
    if time.monotonic()-last>.4:fault='heartbeat_timeout';break
   if finish:break
  else:fault='watchdog_deadline' if ever else 'unarmed_timeout'
  if ever:
   publish_status('stopping');end=time.monotonic()+.8
   while time.monotonic()<end:zero();rclpy.spin_once(n,timeout_sec=.02)
   result['stopped_verified']=bool(base is not None and time.monotonic()-base_rx<.25 and max(abs(base.speed.linear.x),abs(base.speed.linear.y),abs(base.speed.angular.z))<.005)
   if fault and result['stopped_verified'] and base.is_hard_auto and not base.is_joystick_on and not base.is_handle_on:
    c=n.create_client(SetParkingMode,'/move/SetParkingMode')
    if c.wait_for_service(timeout_sec=.5):
     f=c.call_async(SetParkingMode.Request(enable=True));rclpy.spin_until_future_complete(n,f,timeout_sec=1.);result['park_requested']=True;result['park_reply_ok']=bool(f.done() and f.result() is not None and f.result().error_code==0)
    n.destroy_client(c)
  result.update(fault=fault,ever_armed=ever,normal_disarm=finish);publish_status('finished')
 finally:
  args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
