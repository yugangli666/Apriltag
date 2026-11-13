"""Host-only bounded read-only SDO uploads. Never downloads or changes state."""
import argparse,json,subprocess,time
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--duration',type=float,default=240);a=ap.parse_args();start=time.monotonic()
while time.monotonic()-start<a.duration:
 try:
  counts=[]
  for axis in range(4):
   x=subprocess.run(['sudo','-n','ethercat','upload','-p',str(axis),'-t','int32','0x6064','0'],capture_output=True,text=True,timeout=.7,check=True);counts.append(int(x.stdout.split()[-1]))
  value={'wall_ns':time.time_ns(),'counts':counts,'read_only_sdo_upload':True}
  temp=a.output.with_suffix('.tmp');temp.write_text(json.dumps(value));temp.replace(a.output)
 except Exception as e:print(type(e).__name__+': '+str(e),flush=True)
 time.sleep(.35)
