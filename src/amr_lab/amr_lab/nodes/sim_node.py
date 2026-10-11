"""Ground-truth grid stepper for the Robot Operating System 2 (ROS 2) fleet."""
import json, os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, String, Bool
from ament_index_python.packages import get_package_share_directory
from ..core.world import World

class Simulator(Node):
    """Publish poses and ticks, apply requested moves, and measure collisions."""
    def __init__(self):
        super().__init__('amr_simulator')
        self.declare_parameter('robot_count',4); self.declare_parameter('max_ticks',3000)
        self.declare_parameter('tick_rate',1000.0); self.declare_parameter('headless',True)
        self.declare_parameter('seed',1)
        p={k:self.get_parameter(k).value for k in ('robot_count','max_ticks','tick_rate','headless','seed')}
        cfg=Path(get_package_share_directory('amr_lab'))/'config'/'warehouse.yaml'
        rows=[line.strip().split('"')[1] for line in cfg.read_text().splitlines() if line.strip().startswith('- "')]
        self.world=World(tuple(rows)); cells=sorted(self.world.free); n=max(3,int(p['robot_count']))
        self.positions={f'robot_{i}':cells[(i*len(cells))//n] for i in range(n)}
        self.requests={}; self.tick=1; self.pulse=0; self.collisions=0; self.drained=False; self.finished=False
        self.pub_tick=self.create_publisher(Int64,'/tick',10)
        self.pub_collision=self.create_publisher(Int64,'/metrics/collision',10)
        self.pub_finish=self.create_publisher(Bool,'/simulation/finished',10)
        self.pose_pubs={r:self.create_publisher(String,f'/{r}/pose',10) for r in self.positions}
        self.create_subscription(Bool,'/tasks/drained',self.set_drained,10)
        for r in self.positions:
            self.create_subscription(String,f'/{r}/move',lambda m,name=r:self.request(name,m),10)
        period=1/max(1.0,float(p['tick_rate']))
        self.timer=self.create_timer(max(period,.02) if p['headless'] else period,self.advance)
        self.max_ticks=int(p['max_ticks']); self.publish_poses()
    def set_drained(self,msg): self.drained=msg.data
    def request(self,name,msg):
        try:
            item=json.loads(msg.data)
            if item['tick']==self.tick: self.requests[name]=tuple(item['cell'])
        except (ValueError,KeyError,TypeError): pass
    def publish_poses(self):
        for name,cell in self.positions.items():
            self.pose_pubs[name].publish(String(data=json.dumps({'robot_id':name,'tick':self.tick,'cell':cell})))
    def advance(self):
        if self.finished:return
        if len(self.requests)!=len(self.positions):
            self.pulse+=1
            self.publish_poses(); self.pub_tick.publish(Int64(data=self.tick))
            return
        old=dict(self.positions); targets={r:self.requests.pop(r,old[r]) for r in old}
        valid={r:(c if c in self.world.free and self.world.distance(old[r],c)<=1 else old[r]) for r,c in targets.items()}
        self.collisions+=sum(valid[a]==valid[b] for i,a in enumerate(valid) for b in list(valid)[i+1:])
        self.collisions+=sum(old[a]!=old[b] and valid[a]==old[b] and valid[b]==old[a] for i,a in enumerate(valid) for b in list(valid)[i+1:])
        self.positions.update(valid); self.tick+=1; self.pulse=9; self.publish_poses()
        self.pub_collision.publish(Int64(data=self.collisions)); self.pub_tick.publish(Int64(data=self.tick))
        if self.drained or self.tick>=self.max_ticks:
            self.finished=True; self.pub_finish.publish(Bool(data=True)); self.timer.cancel()

def main(args=None):
    rclpy.init(args=args); node=Simulator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():rclpy.shutdown()
