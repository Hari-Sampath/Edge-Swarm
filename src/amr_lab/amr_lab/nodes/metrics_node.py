"""Measure simulation outcomes and write one measured CSV row."""
import csv, json, os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool, String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from ..core.world import World

class MetricsNode(Node):
    """Collect ground-truth pose changes and reported run events."""
    def __init__(self):
        super().__init__('metrics')
        defaults={'seed':1,'packet_loss':0.0,'robot_count':4,'max_ticks':3000,'deadlock_ticks':20}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; self.tick=0; self.total=0; self.done=0
        self.distance=0; self.waits=0; self.collisions=0; self.attempts=0; self.drops=0; self.stalled=0; self.deadlocks=0; self.last_move=0; self.stalled_run=0; self.moved_this_tick=False; self.poses={}; self.finished=False; self.wait_events=set()
        share=Path(get_package_share_directory('amr_lab'))/'config'/'warehouse.yaml'
        rows=[line.strip().split('"')[1] for line in share.read_text().splitlines() if line.strip().startswith('- "')]
        self.world=World(tuple(rows)); self.goals={}; self.paths={}
        self.marker_pub=self.create_publisher(MarkerArray,'/visualization_marker_array',10); self.create_timer(.1,self.publish_markers)
        self.csv=Path(os.environ.get('AMR_TRIALS_CSV',str(Path.cwd()/'results'/'trials.csv'))); self.csv.parent.mkdir(parents=True,exist_ok=True)
        for topic,cb,typ in [('/tick',self.on_tick,Int64),('/metrics/collision',self.on_collision,Int64),('/metrics/delivery_attempts',self.on_attempt,Int64),('/metrics/delivery_drops',self.on_drop,Int64),('/metrics/wait',self.on_wait,String),('/tasks/total',self.on_total,Int64),('/tasks/completed',self.on_done,Int64),('/simulation/finished',self.on_finish,Bool)]:self.create_subscription(typ,topic,cb,10)
        for i in range(int(self.p['robot_count'])):
            name=f'robot_{i}'
            self.create_subscription(String,f'/{name}/pose',lambda m,n=name:self.on_pose(n,m),10)
            self.create_subscription(String,f'/bus/{name}',lambda m,n=name:self.on_bus(n,m),10)
            self.create_subscription(String,f'/tasks/assignment/{name}',lambda m,n=name:self.on_task(n,m),10)
        self.create_subscription(String,'/tasks/status',self.on_status,10)
    def on_pose(self,name,msg):
        try:
            cell=tuple(json.loads(msg.data)['cell']); old=self.poses.get(name)
            if old is not None and old!=cell:self.distance+=1; self.last_move=self.tick; self.moved_this_tick=True
            self.poses[name]=cell
        except (ValueError,KeyError,TypeError):pass
    def on_task(self,name,msg):
        try:self.goals[name]=tuple(json.loads(msg.data)['goal'])
        except (ValueError,KeyError,TypeError):pass
    def on_status(self,msg):
        try:
            event=json.loads(msg.data)
            if event.get('status')=='completed':self.goals.pop(event.get('robot_id'),None)
        except (ValueError,TypeError):pass
    def on_bus(self,name,msg):
        try:self.paths[name]=json.loads(msg.data)['intent']
        except (ValueError,KeyError,TypeError):pass
    def publish_markers(self):
        array=MarkerArray(); free=Marker(); free.header.frame_id='map'; free.id=0; free.type=Marker.CUBE_LIST; free.action=Marker.ADD
        free.scale.x=free.scale.y=1.; free.scale.z=.08; free.color.r=.25; free.color.g=.35; free.color.b=.42; free.color.a=1.
        for x,y in sorted(self.world.free):point=Point(); point.x=float(x); point.y=float(y); free.points.append(point)
        array.markers.append(free); walls=Marker(); walls.header.frame_id='map'; walls.id=1; walls.type=Marker.CUBE_LIST; walls.action=Marker.ADD
        walls.scale.x=walls.scale.y=1.; walls.scale.z=.2; walls.color.r=.15; walls.color.g=.18; walls.color.b=.22; walls.color.a=1.
        for y,row in enumerate(self.world.rows):
            for x,char in enumerate(row):
                if char=='#':point=Point(); point.x=float(x); point.y=float(y); walls.points.append(point)
        array.markers.append(walls); chokes=Marker(); chokes.header.frame_id='map'; chokes.id=2; chokes.type=Marker.CUBE_LIST; chokes.action=Marker.ADD
        chokes.scale.x=chokes.scale.y=.45; chokes.scale.z=.25; chokes.color.r=1.; chokes.color.g=.5; chokes.color.a=1.
        for x,y in self.world.chokes.values():point=Point(); point.x=float(x); point.y=float(y); chokes.points.append(point)
        array.markers.append(chokes)
        for i,(name,(x,y)) in enumerate(sorted(self.poses.items())):
            robot=Marker(); robot.header.frame_id='map'; robot.id=10+i; robot.type=Marker.SPHERE; robot.action=Marker.ADD
            robot.pose.position.x=float(x); robot.pose.position.y=float(y); robot.pose.position.z=.35; robot.scale.x=robot.scale.y=robot.scale.z=.5
            robot.color.r=(i*.31)%1.; robot.color.g=.8; robot.color.b=.2; robot.color.a=1.; array.markers.append(robot)
        goals=Marker(); goals.header.frame_id='map'; goals.id=20; goals.type=Marker.CUBE_LIST; goals.action=Marker.ADD
        goals.scale.x=goals.scale.y=.5; goals.scale.z=.25; goals.color.r=1.; goals.color.g=.85; goals.color.a=1.
        for x,y in self.goals.values():point=Point(); point.x=float(x); point.y=float(y); goals.points.append(point)
        array.markers.append(goals); paths=Marker(); paths.header.frame_id='map'; paths.id=21; paths.type=Marker.LINE_LIST; paths.action=Marker.ADD
        paths.scale.x=.08; paths.color.r=.1; paths.color.g=.9; paths.color.b=1.; paths.color.a=.9
        for name,reservations in self.paths.items():
            if name not in self.poses:continue
            last=self.poses[name]
            for cell,_ in reservations:
                a=Point(); a.x=float(last[0]); a.y=float(last[1]); b=Point(); b.x=float(cell[0]); b.y=float(cell[1]); paths.points.extend((a,b)); last=cell
        array.markers.append(paths); self.marker_pub.publish(array)
    def on_tick(self,msg):
        if msg.data<=self.tick:return
        self.tick=msg.data
        if self.moved_this_tick:self.stalled_run=0
        else:
            self.stalled+=1; self.stalled_run+=1
            if self.stalled_run==int(self.p['deadlock_ticks']):self.deadlocks+=1
        self.moved_this_tick=False
    def on_collision(self,msg):self.collisions=msg.data
    def on_attempt(self,msg):self.attempts+=msg.data
    def on_drop(self,msg):self.drops+=msg.data
    def on_wait(self,msg):
        try:
            event=json.loads(msg.data); key=(event['robot_id'],int(event['tick']))
            if key not in self.wait_events:self.wait_events.add(key); self.waits=len(self.wait_events)
        except (ValueError,KeyError,TypeError):pass
    def on_total(self,msg):self.total=msg.data
    def on_done(self,msg):self.done=msg.data
    def on_finish(self,msg):
        if msg.data and not self.finished:self.finished=True; self.finish_timer=self.create_timer(.1,self.write_once)
    def write_once(self):
        self.finish_timer.cancel(); self.write()
    def write(self):
        row={'seed':self.p['seed'],'packet_loss':self.p['packet_loss'],'tasks_total':self.total,'tasks_completed':self.done,'makespan':self.tick,'total_distance':self.distance,'collisions':self.collisions,'wait_steps':self.waits,'deadlocks':self.deadlocks,'stalled_ticks':self.stalled,'timed_out':self.done<self.total,'messages_sent':self.attempts,'messages_dropped':self.drops}
        with self.csv.open('a',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=row); writer.writeheader() if f.tell()==0 else None; writer.writerow(row)
        self.get_logger().info(f'Run metrics: {row}'); rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args); node=MetricsNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():rclpy.shutdown()
