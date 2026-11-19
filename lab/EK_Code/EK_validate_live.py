import rclpy,time,json,math,os
from pathlib import Path
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan,Image,CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
rclpy.init(); n=rclpy.create_node('EK_lab_validation')
out=Path(__file__).resolve().parents[1]; data={}; counts={}; edges=set(); stamps={}; odoms=[]
def cb(key,m):
 data[key]=m; counts[key]=counts.get(key,0)+1
 stamps.setdefault(key,[]).append(time.monotonic())
 if key=='tf':
  for t in m.transforms: edges.add((t.header.frame_id,t.child_frame_id))
 if key=='odom': odoms.append((time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z))
subs=[n.create_subscription(t,p,lambda m,k=k:cb(k,m),qos_profile_sensor_data) for k,t,p in [('clock',Clock,'/clock'),('scan',LaserScan,'/scan'),('image',Image,'/camera/image_raw'),('camera_info',CameraInfo,'/camera/camera_info'),('odom',Odometry,'/odom'),('tf',TFMessage,'/tf')]]
pub=n.create_publisher(Twist,'/cmd_vel',10)
start=time.monotonic()
while time.monotonic()-start<32:
 elapsed=time.monotonic()-start
 m=Twist()
 if 8<elapsed<14:m.linear.x=.12
 elif 17<elapsed<23:m.angular.z=.5
 pub.publish(m);rclpy.spin_once(n,timeout_sec=.05)
pub.publish(Twist())
result={'counts':counts,'wall_rates_hz':{k:(len(v)-1)/(v[-1]-v[0]) if len(v)>1 else 0 for k,v in stamps.items()},'tf_edges':sorted(edges),'missing':sorted(set(['clock','scan','image','camera_info','odom','tf'])-set(data)),'odometry_samples':odoms[::10]}
if 'scan' in data:
 s=data['scan'];result['scan']={'frame':s.header.frame_id,'ranges':len(s.ranges),'finite_returns':sum(math.isfinite(x) for x in s.ranges),'min':s.range_min,'max':s.range_max,'angle_min':s.angle_min,'angle_max':s.angle_max}
if 'image' in data:
 s=data['image'];result['image']={'frame':s.header.frame_id,'width':s.width,'height':s.height,'encoding':s.encoding,'bytes':len(s.data)}
 from PIL import Image as PILImage
 PILImage.frombytes('RGB',(s.width,s.height),bytes(s.data),'raw',{'rgb8':'RGB','bgr8':'BGR'}.get(s.encoding,'RGB')).save(out/'EK_Evidence/EK_camera_raw.png')
if 'odom' in data:
 s=data['odom'];result['odom']={'parent':s.header.frame_id,'child':s.child_frame_id}
result['forbidden_world_odom']=('world','odom') in edges
(out/'EK_Logs'/(os.environ.get('EK_VALIDATION_NAME','EK_live_validation')+'.json')).write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
