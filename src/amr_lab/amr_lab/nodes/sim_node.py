"""Robot Operating System 2 (ROS 2) tick-driven warehouse simulation."""
import csv, os
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from ..core.world import World
from ..core.registry import ALLOCATORS, CONFLICTS
from ..core.bus_filter import LossFilter
from ..core.messages import BusMessage

class Simulator(Node):
    """Run a reproducible multi-robot simulation on published ticks."""
    def __init__(self):
        super().__init__('amr_simulator')
        share=get_package_share_directory('amr_lab'); cfg=Path(share)/'config'
        params={'robot_count':4,'seed':1,'packet_loss':0.0,'task_arrival_rate':0.0,'task_count':8,'tick_rate':1000.0,'max_ticks':3000,'deadlock_ticks':20,'peer_timeout':6,'allocator':'hungarian','conflict_resolver':'reservation','intent_steps':8,'show_markers':True,'headless':True}
        for k,v in params.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in params}
        self.p['seed']=self.get_parameter('seed').value; self.p['packet_loss']=self.get_parameter('packet_loss').value
        rows=[line.strip().split('"',2)[1] for line in (cfg/'warehouse.yaml').read_text().splitlines() if line.strip().startswith('- "')]
        self.world=World(tuple(rows))
        n=max(3,int(self.p['robot_count'])); cells=sorted(self.world.free); self.positions={f'robot_{i}':cells[(i*len(cells))//n] for i in range(n)}
        parking=[c for c in self.world.parking_cells if c not in self.positions.values()]
        self.parking_spots={r:parking[i%len(parking)] for i,r in enumerate(self.positions)}; self.parking=set()
        self.goals={}; self.task_goals={}; self.done=set(); self.tasks=self.world.tasks(self.p['seed'],int(self.p['task_count']))
        self.allocator=ALLOCATORS[self.p['allocator']](); self.resolver=CONFLICTS[self.p['conflict_resolver']](self.world,int(self.p['intent_steps']))
        self.filters={r:LossFilter(float(self.p['packet_loss']),int(self.p['seed'])+i) for i,r in enumerate(self.positions)}
        self.peers={r:{} for r in self.positions}; self.tick=0; self.distance=0; self.waits=0; self.collisions=0; self.sent=0; self.dropped=0
        self.last_progress=0; self.deadlocks=0; self.assigned={}; self.logdir=Path(os.environ.get('AMR_RESULTS_DIR',str(Path.cwd()/'results'/'run'))); self.logdir.mkdir(parents=True,exist_ok=True)
        self.logfile=(self.logdir/'decisions.log').open('w'); self.csvfile=Path(os.environ.get('AMR_TRIALS_CSV',str(Path.cwd()/'results'/'trials.csv'))); self.csvfile.parent.mkdir(parents=True,exist_ok=True)
        self.tick_pub=self.create_publisher(Int64,'/tick',10); self.bus={r:self.create_publisher(String,f'/bus/{r}',10) for r in self.positions}
        self.markers=self.create_publisher(MarkerArray,'/visualization_marker_array',10)
        period=1/max(1.0,float(self.p['tick_rate']))
        self.timer=self.create_timer(min(period,0.0001) if self.p['headless'] else period,self.step)
    def say(self, robot, action, why):
        line=f'[t={self.tick}] {robot} | {action} | {why}'; self.get_logger().info(line); self.logfile.write(line+'\n'); self.logfile.flush()
    def assign(self):
        idle=[r for r in self.positions if r not in self.goals]; unassigned={k:v for k,v in self.tasks.items() if k not in self.assigned and k not in self.done}
        if not idle or not unassigned:return
        result=self.allocator.assign({'world':self.world,'robots':{r:self.positions[r] for r in idle},'tasks':unassigned})
        for r,ids in result.items():
            if ids: self.assigned[ids[0]]=r; self.task_goals[r]=self.tasks[ids[0]]; self.goals[r]=self.tasks[ids[0]]['start']
    def step(self):
        self.tick+=1; self.tick_pub.publish(Int64(data=self.tick)); self.assign(); old=dict(self.positions); proposals={}; planned_paths={}; moved=False
        for r,pos in old.items():
            if r not in self.goals: continue
            goal=self.goals[r]
            if pos==goal:
                if r in self.parking:
                    self.parking.remove(r); del self.goals[r]; del self.task_goals[r]; continue
                task=self.task_goals[r]
                if goal==task['start']:
                    self.goals[r]=task['goal']; self.say(r,'PICKUP',f'task {next(k for k,v in self.assigned.items() if v==r)} ready')
                else:
                    tid=next(k for k,v in self.assigned.items() if v==r); self.done.add(tid); self.say(r,'DELIVER',f'task {tid} completed'); self.assigned.pop(tid); self.goals[r]=self.parking_spots[r]; self.task_goals[r]=None; self.parking.add(r); continue
            path=self.resolver.plan({'robot_id':r,'tick':self.tick-1,'position':pos,'goal':self.goals[r],'timeout':int(self.p['peer_timeout'])},self.peers[r])
            planned_paths[r]=path; proposals[r]=path[0][0] if path else pos
        occupied=set(old.values()); accepted={}
        for r in sorted(proposals):
            target=proposals[r]
            blocked=any(other!=r and (old[other]==target or (proposals.get(other)==old[r] and target==old[other])) for other in old)
            if blocked or target in accepted.values():
                self.waits+=1; self.say(r,'WAIT',f'cell {target} is occupied or reserved at t={self.tick}') ; accepted[r]=old[r]
            elif target==old[r]:
                self.waits+=1; self.say(r,'WAIT',f'no safe route is available from {old[r]}') ; accepted[r]=old[r]
            else:
                accepted[r]=target
                self.distance+=1; moved=True; self.say(r,'MOVE',f'next cell {target} is clear')
        self.collisions+=sum(1 for a in accepted for b in accepted if a<b and accepted[a]==accepted[b])
        self.collisions+=sum(1 for a in accepted for b in accepted if a<b and accepted[a]==old[b] and accepted[b]==old[a] and old[a]!=old[b])
        self.positions.update(accepted)
        if moved:self.last_progress=self.tick
        if self.tick-self.last_progress>=int(self.p['deadlock_ticks']): self.deadlocks+=1; self.last_progress=self.tick
        messages={}
        for sender in self.positions:
            msg=BusMessage(sender,self.tick,self.positions[sender],planned_paths.get(sender,[])[:int(self.p['intent_steps'])])
            messages[sender]=msg; self.bus[sender].publish(String(data=msg.encode())); self.sent+=1
        for receiver in self.positions:
            for sender,msg in messages.items():
                if sender==receiver: continue
                if self.filters[receiver].keep(): self.peers[receiver][sender]={'tick':msg.tick,'position':msg.position,'intent':msg.intent}
                else:self.dropped+=1
        self.latest_paths=planned_paths
        if self.p['show_markers']: self.publish_markers()
        if len(self.done)==len(self.tasks) or self.tick>=int(self.p['max_ticks']): self.finish()
    def publish_markers(self):
        arr=MarkerArray(); m=Marker(); m.header.frame_id='map'; m.type=Marker.CUBE_LIST; m.action=Marker.ADD; m.scale.x=m.scale.y=1.; m.scale.z=.1; m.color.r=.25; m.color.g=.35; m.color.b=.42; m.color.a=1.
        for x,y in sorted(self.world.free): p=Point(); p.x=float(x); p.y=float(y); m.points.append(p)
        arr.markers.append(m)
        walls=Marker(); walls.header.frame_id='map'; walls.id=1; walls.type=Marker.CUBE_LIST; walls.action=Marker.ADD; walls.scale.x=walls.scale.y=1.; walls.scale.z=.18; walls.color.r=.18; walls.color.g=.2; walls.color.b=.24; walls.color.a=1.
        for y,row in enumerate(self.world.rows):
            for x,char in enumerate(row):
                if char=='#': p=Point(); p.x=float(x); p.y=float(y); walls.points.append(p)
        arr.markers.append(walls)
        chokes=Marker(); chokes.header.frame_id='map'; chokes.id=2; chokes.type=Marker.CUBE_LIST; chokes.action=Marker.ADD; chokes.scale.x=chokes.scale.y=.48; chokes.scale.z=.22; chokes.color.r=1.; chokes.color.g=.55; chokes.color.a=1.
        for x,y in self.world.chokes.values(): p=Point(); p.x=float(x); p.y=float(y); chokes.points.append(p)
        arr.markers.append(chokes)
        for i,(r,(x,y)) in enumerate(sorted(self.positions.items())):
            a=Marker(); a.header.frame_id='map'; a.id=i+3; a.type=Marker.SPHERE; a.action=Marker.ADD; a.pose.position.x=float(x); a.pose.position.y=float(y); a.pose.position.z=.35; a.scale.x=a.scale.y=a.scale.z=.45; a.color.r=(i*.31)%1; a.color.g=.8; a.color.b=.2; a.color.a=1.; arr.markers.append(a)
        line=Marker(); line.header.frame_id='map'; line.id=50; line.type=Marker.LINE_LIST; line.action=Marker.ADD; line.scale.x=.08; line.color.r=.1; line.color.g=.9; line.color.b=1.; line.color.a=.9
        for r,p in self.positions.items():
            last=Point(); last.x=float(p[0]); last.y=float(p[1])
            for cell,_ in getattr(self,'latest_paths',{}).get(r,[]):
                next_point=Point(); next_point.x=float(cell[0]); next_point.y=float(cell[1]); line.points.extend((last,next_point)); last=next_point
        arr.markers.append(line)
        goals=Marker(); goals.header.frame_id='map'; goals.id=51; goals.type=Marker.CUBE_LIST; goals.action=Marker.ADD; goals.scale.x=goals.scale.y=.5; goals.scale.z=.25; goals.color.r=1.; goals.color.g=.85; goals.color.a=1.
        for x,y in self.goals.values(): p=Point(); p.x=float(x); p.y=float(y); goals.points.append(p)
        arr.markers.append(goals)
        self.markers.publish(arr)
    def finish(self):
        row={'seed':self.p['seed'],'packet_loss':self.p['packet_loss'],'tasks_completed':len(self.done),'makespan':self.tick,'total_distance':self.distance,'collisions':self.collisions,'wait_steps':self.waits,'deadlocks':self.deadlocks,'messages_sent':self.sent,'messages_dropped':self.dropped}
        exists=self.csvfile.exists();
        with self.csvfile.open('a',newline='') as f: w=csv.DictWriter(f,fieldnames=row); w.writeheader() if not exists else None; w.writerow(row)
        self.get_logger().info(f'Run metrics: {row}'); self.logfile.close(); self.timer.cancel(); rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args); node=Simulator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,):pass
    finally:
        if rclpy.ok():rclpy.shutdown()
