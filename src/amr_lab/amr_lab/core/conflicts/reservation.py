"""Decentralized space-time reservation planner."""
from ..astar import astar

class ReservationResolver:
    """Plan around last heard positions, intents, and choke resources."""
    def __init__(self, world, intent_steps=8): self.world=world; self.intent_steps=intent_steps
    def plan(self, robot_state, peer_view):
        """Return a safe path, or yield the deterministic recovery turn."""
        tick=robot_state['tick']; occupied=set(); intents=set(); me=robot_state['robot_id']
        recovery=robot_state.get('recovery_ticks',0); count=robot_state.get('robot_count',len(peer_view)+1)
        active=[me] if robot_state.get('has_task',True) else []
        recovery_mode=robot_state.get('recovering',False)
        for peer,data in peer_view.items():
            if peer==me: continue
            age=tick-data['tick']
            if age>robot_state.get('timeout',6): return []
            if data.get('has_task',bool(data.get('intent'))):active.append(peer)
            recovery_mode=recovery_mode or data.get('recovering',False)
            occupied.add((tuple(data['position']),tick+1))
            intents.update((tuple(cell),int(t),peer) for cell,t in data['intent'])
        start=tuple(robot_state['position']); goal=tuple(robot_state['goal'])
        recovery_turn=False
        if recovery and recovery_mode and active:
            active=sorted(set(active)); token=active[tick%len(active)]
            if token!=me:return []
            recovery_turn=True
        resources={cell:rid for rid,cell in self.world.chokes.items()}
        open_path=astar(self.world,start,goal,occupied,start_time=tick)
        # Choke requests use ascending resource numbers; lower robot id wins equal requests.
        requests=[]
        for cell,t in open_path:
            for peer_cell,peer_time,peer in intents:
                if cell==peer_cell and t==peer_time:
                    requests.append((resources.get(cell,10**9),peer,cell,t))
        overrides=set()
        for resource_id,peer,cell,t in sorted(requests):
            if me < peer: overrides.add((cell,t,peer))
        blocked=occupied | {(cell,t) for cell,t,peer in intents if (cell,t,peer) not in overrides}
        path=astar(self.world,start,goal,blocked,start_time=tick)
        if recovery_turn:
            escapes=[]
            for parking in self.world.parking_cells:
                if parking==start:continue
                route=astar(self.world,start,parking,blocked,start_time=tick)
                if route and route[0][0]!=start:escapes.append((len(route),parking,route))
            if escapes:path=min(escapes,key=lambda item:(item[0],item[1]))[2]
        return path[:self.intent_steps]
