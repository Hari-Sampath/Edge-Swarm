"""Measure simulation outcomes and write one measured CSV row."""
import csv, json, os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool, String

class MetricsNode(Node):
    """Collect ground-truth pose changes and reported run events."""
    def __init__(self):
        super().__init__('metrics')
        defaults={'seed':1,'packet_loss':0.0,'robot_count':4,'max_ticks':3000,'deadlock_ticks':20}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; self.tick=0; self.total=0; self.done=0
        self.distance=0; self.waits=0; self.collisions=0; self.attempts=0; self.drops=0; self.stalled=0; self.deadlocks=0; self.last_move=0; self.poses={}; self.finished=False
        self.csv=Path(os.environ.get('AMR_TRIALS_CSV',str(Path.cwd()/'results'/'trials.csv'))); self.csv.parent.mkdir(parents=True,exist_ok=True)
        for topic,cb,typ in [('/tick',self.on_tick,Int64),('/metrics/collision',self.on_collision,Int64),('/metrics/delivery_attempts',self.on_attempt,Int64),('/metrics/delivery_drops',self.on_drop,Int64),('/metrics/wait',self.on_wait,Int64),('/tasks/total',self.on_total,Int64),('/tasks/completed',self.on_done,Int64),('/simulation/finished',self.on_finish,Bool)]:self.create_subscription(typ,topic,cb,10)
        for i in range(int(self.p['robot_count'])):self.create_subscription(String,f'/robot_{i}/pose',lambda m,n=f'robot_{i}':self.on_pose(n,m),10)
    def on_pose(self,name,msg):
        try:
            cell=tuple(json.loads(msg.data)['cell']); old=self.poses.get(name)
            if old is not None and old!=cell:self.distance+=1; self.last_move=self.tick
            self.poses[name]=cell
        except (ValueError,KeyError,TypeError):pass
    def on_tick(self,msg):
        if msg.data<=self.tick:return
        self.tick=msg.data
        if self.tick>self.last_move:self.stalled+=1
        if self.tick-self.last_move==int(self.p['deadlock_ticks']):self.deadlocks+=1
    def on_collision(self,msg):self.collisions=msg.data
    def on_attempt(self,msg):self.attempts+=msg.data
    def on_drop(self,msg):self.drops+=msg.data
    def on_wait(self,msg):self.waits+=msg.data
    def on_total(self,msg):self.total=msg.data
    def on_done(self,msg):self.done=msg.data
    def on_finish(self,msg):
        if msg.data and not self.finished:self.finished=True; self.write()
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
