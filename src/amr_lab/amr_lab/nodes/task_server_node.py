"""Central task creation and Hungarian assignment service."""
import json, random
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool
from ament_index_python.packages import get_package_share_directory
from ..core.world import World
from ..core.registry import ALLOCATORS

class TaskServerNode(Node):
    """Create tasks and publish one-task assignment rounds to idle robots."""
    def __init__(self):
        super().__init__('task_server')
        defaults={'robot_count':4,'seed':1,'task_count':8,'task_arrival_rate':0.0,'allocator':'hungarian'}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; share=Path(get_package_share_directory('amr_lab'))
        rows=[line.strip().split('"')[1] for line in (share/'config'/'warehouse.yaml').read_text().splitlines() if line.strip().startswith('- "')]
        self.world=World(tuple(rows)); self.all_tasks=self.world.tasks(int(self.p['seed']),int(self.p['task_count']))
        self.pending={}; self.active={}; self.acknowledged=set(); self.done=set(); self.last_tick=0; self.poses={}; self.next_index=0
        self.rng=random.Random(int(self.p['seed'])+7919); self.assigner=ALLOCATORS[self.p['allocator']]()
        self.pubs={f'robot_{i}':self.create_publisher(String,f'/tasks/assignment/robot_{i}',10) for i in range(max(3,int(self.p['robot_count'])))}
        self.total_pub=self.create_publisher(Int64,'/tasks/total',10); self.done_pub=self.create_publisher(Int64,'/tasks/completed',10)
        self.drained_pub=self.create_publisher(Bool,'/tasks/drained',10)
        self.create_subscription(Int64,'/tick',self.on_tick,10)
        self.create_subscription(String,'/tasks/status',self.on_status,10)
        for robot in self.pubs:self.create_subscription(String,f'/{robot}/pose',lambda m,n=robot:self.on_pose(n,m),10)
        if float(self.p['task_arrival_rate'])<=0:self.release_all()
    def release_all(self):
        self.pending.update(self.all_tasks); self.next_index=len(self.all_tasks); self.publish_total()
    def publish_total(self):self.total_pub.publish(Int64(data=len(self.all_tasks)))
    def on_pose(self,name,msg):
        try:self.poses[name]=tuple(json.loads(msg.data)['cell'])
        except (ValueError,KeyError,TypeError):pass
    def on_tick(self,msg):
        if msg.data<=self.last_tick:return
        self.last_tick=msg.data; self.publish_total()
        rate=float(self.p['task_arrival_rate'])
        if rate>0 and self.next_index<len(self.all_tasks) and self.rng.random()<rate:
            tid=sorted(self.all_tasks)[self.next_index]; self.pending[tid]=self.all_tasks[tid]; self.next_index+=1; self.publish_total()
        
        for robot,tid in self.active.items():
            if robot not in self.acknowledged:
                task=self.all_tasks[tid]; self.pubs[robot].publish(String(data=json.dumps({'task_id':tid,**task})))
        idle={r:p for r,p in self.poses.items() if r not in self.active}
        if idle and self.pending:
            choices=self.assigner.assign({'world':self.world,'robots':idle,'tasks':self.pending})
            for robot,ids in choices.items():
                if not ids:continue
                tid=ids[0]; task=self.pending.pop(tid); self.active[robot]=tid
                self.pubs[robot].publish(String(data=json.dumps({'task_id':tid,**task})))
        complete=(self.next_index>=len(self.all_tasks) and not self.pending and not self.active)
        self.drained_pub.publish(Bool(data=complete))
    def on_status(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        robot=data.get('robot_id'); tid=data.get('task_id')
        if data.get('status')=='assigned' and self.active.get(robot)==tid:self.acknowledged.add(robot)
        if data.get('status')=='completed' and self.active.get(robot)==tid:
            self.active.pop(robot); self.acknowledged.discard(robot); self.done.add(tid); self.done_pub.publish(Int64(data=len(self.done)))

def main(args=None):
    rclpy.init(args=args); node=TaskServerNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():rclpy.shutdown()
