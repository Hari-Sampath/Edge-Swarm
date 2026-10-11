"""Decentralized space-time reservation planner."""
from ..astar import astar

class ReservationResolver:
    """Plan around last heard positions, intents, and choke resources."""
    def __init__(self, world, intent_steps=8): self.world=world; self.intent_steps=intent_steps
    def plan(self, robot_state, peer_view):
        """Return the next path as cell and simulation tick pairs."""
        tick=robot_state['tick']; occupied=set(); intents=set(); me=robot_state['robot_id']
        for peer,data in peer_view.items():
            if peer==me: continue
            age=tick-data['tick']
            if age>robot_state.get('timeout',6): return []
            occupied.add((tuple(data['position']),tick+1))
            intents.update((tuple(cell),int(t),peer) for cell,t in data['intent'])
        start=tuple(robot_state['position']); goal=tuple(robot_state['goal'])
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
        return path[:self.intent_steps]
