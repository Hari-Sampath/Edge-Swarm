"""Independent robot planner and simulated bus participant."""
import json, os, random
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64
from ament_index_python.packages import get_package_share_directory
from ..core.world import World
from ..core.registry import CONFLICTS
from ..core.bus_filter import LossFilter
from ..core.messages import BusMessage

class RobotNode(Node):
    """Plan only from this robot's pose, assignment, and delivered bus messages."""
    def __init__(self):
        super().__init__('robot')
        defaults={'robot_id':'robot_0','robot_count':4,'seed':1,'packet_loss':0.0,'peer_timeout':6,'intent_steps':8,'conflict_resolver':'reservation','deadlock_ticks':20}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; self.name=self.p['robot_id']
        cfg=Path(get_package_share_directory('amr_lab'))/'config'/'warehouse.yaml'
        rows=[line.strip().split('"')[1] for line in cfg.read_text().splitlines() if line.strip().startswith('- "')]
        self.world=World(tuple(rows)); self.resolver=CONFLICTS[self.p['conflict_resolver']](self.world,int(self.p['intent_steps']))
        idx=int(self.name.split('_')[-1]); self.loss=LossFilter(float(self.p['packet_loss']),int(self.p['seed'])+idx*1009)
        self.peers={}; self.seen=set(); self.pending={}; self.tick=0; self.last_bus=None; self.acted_tick=0; self.pose=None; self.task=None; self.phase='pickup'; self.waits=0; self.wait_streak=0
        bus_qos=10
        self.pub_bus=self.create_publisher(String,f'/bus/{self.name}',bus_qos)
        self.pub_move=self.create_publisher(String,f'/{self.name}/move',10)
        self.pub_status=self.create_publisher(String,'/tasks/status',10); self.pub_wait=self.create_publisher(String,'/metrics/wait',10)
        self.pub_attempt=self.create_publisher(Int64,'/metrics/delivery_attempts',10)
        self.pub_drop=self.create_publisher(Int64,'/metrics/delivery_drops',10)
        self.create_subscription(Int64,'/tick',self.on_tick,10)
        self.create_subscription(String,f'/{self.name}/pose',self.on_pose,10)
        self.create_subscription(String,f'/tasks/assignment/{self.name}',self.on_assignment,10)
        for i in range(int(self.p['robot_count'])):
            peer=f'robot_{i}'
            if peer!=self.name:self.create_subscription(String,f'/bus/{peer}',lambda m,n=peer:self.on_bus(n,m),bus_qos)
        self.log_path=Path(os.environ.get('AMR_RESULTS_DIR',str(Path.cwd()/'results'/'run'))); self.log_path.mkdir(parents=True,exist_ok=True)
    def on_pose(self,msg):
        try:
            self.pose=json.loads(msg.data)
        except ValueError:pass
    def on_assignment(self,msg):
        try:
            task=json.loads(msg.data)
            if not self.task or task['task_id']!=self.task['task_id']:self.phase='pickup'
            self.task=task
            self.pub_status.publish(String(data=json.dumps({'robot_id':self.name,'task_id':task['task_id'],'status':'assigned'})))
        except (ValueError,KeyError):pass
    def on_bus(self,peer,msg):
        try:
            data=BusMessage.decode(msg.data); delivered=self.loss.keep()
            self.pub_attempt.publish(Int64(data=1)); self.pub_drop.publish(Int64(data=int(not delivered)))
            if data.tick>=self.tick:
                self.seen.add(peer)
                if delivered:self.peers[peer]={'tick':data.tick,'position':data.position,'intent':data.intent}
                self.pending[(data.tick,peer)]=(data,delivered)
        except (ValueError,KeyError,TypeError):pass
        self.act_if_ready()
    def on_tick(self,msg):
        if msg.data==self.tick and self.last_bus is not None:
            self.pub_bus.publish(String(data=self.last_bus)); return
        if msg.data<self.tick:return
        self.tick=msg.data; self.seen=set()
        for peer in list(self.peers):
            if self.peers[peer]['tick']<self.tick:self.peers[peer]=dict(self.peers[peer])
        for (tick,peer),(data,delivered) in list(self.pending.items()):
            if tick==self.tick:
                self.seen.add(peer); self.pending.pop((tick,peer),None)
                if delivered:self.peers[peer]={'tick':data.tick,'position':data.position,'intent':data.intent}
        if self.pose is None:return
        intent=[]
        if self.task:
            goal=tuple(self.task['start'] if self.phase=='pickup' else self.task['goal'])
            path=self.resolver.plan({'robot_id':self.name,'tick':self.tick-1,'position':tuple(self.pose['cell']),'goal':goal,'timeout':int(self.p['peer_timeout']),'wait_streak':self.wait_streak,'recovery_ticks':int(self.p['deadlock_ticks']),'robot_count':int(self.p['robot_count'])},self.peers)
            intent=path
        bus=BusMessage(self.name,self.tick,tuple(self.pose['cell']),intent[:int(self.p['intent_steps'])])
        self.last_bus=bus.encode(); self.pub_bus.publish(String(data=self.last_bus)); self.act_if_ready()
    def act_if_ready(self):
        if self.acted_tick==self.tick:return
        peers={f'robot_{i}' for i in range(int(self.p['robot_count'])) if f'robot_{i}'!=self.name}
        if self.pose is None or len(self.seen)<len(peers) or self.tick<=0:return
        cell=tuple(self.pose['cell']); target=cell; why='no task assigned'
        if self.task:
            goal=tuple(self.task['start'] if self.phase=='pickup' else self.task['goal'])
            if cell==goal:
                if self.phase=='pickup':self.phase='delivery'; goal=tuple(self.task['goal'])
                else:
                    self.pub_status.publish(String(data=json.dumps({'robot_id':self.name,'task_id':self.task['task_id'],'status':'completed'})))
                    self.task=None; why='task delivered'
            if self.task:
                path=self.resolver.plan({'robot_id':self.name,'tick':self.tick-1,'position':cell,'goal':goal,'timeout':int(self.p['peer_timeout']),'wait_streak':self.wait_streak,'recovery_ticks':int(self.p['deadlock_ticks']),'robot_count':int(self.p['robot_count'])},self.peers)
                if path:target=tuple(path[0][0]); why=f'clear next reservation at {target}'
                elif self.wait_streak>=int(self.p['deadlock_ticks']):
                    count=int(self.p['robot_count']); token=f'robot_{(self.tick//int(self.p["deadlock_ticks"]))%count}'
                    why=f'yielding recovery turn to {token}' if token!=self.name else 'recovery turn has no safe route'
                else:why='no safe route is available'
        action='MOVE' if target!=cell else 'WAIT'
        if action=='WAIT':
            self.waits+=1
            self.wait_streak=self.wait_streak+1 if self.task else 0; self.pub_wait.publish(String(data=json.dumps({'robot_id':self.name,'tick':self.tick})))
        if action=='MOVE':self.wait_streak=0
        self.acted_tick=self.tick
        line=f'[t={self.tick}] {self.name} | {action} | {why}'; self.get_logger().info(line)
        with (self.log_path/'decisions.log').open('a') as log:log.write(line+'\n')
        self.pub_move.publish(String(data=json.dumps({'tick':self.tick,'cell':target})))

def main(args=None):
    rclpy.init(args=args); node=RobotNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():rclpy.shutdown()
