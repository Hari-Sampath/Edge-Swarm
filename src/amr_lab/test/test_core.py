"""Tests for path planning, assignment, reservation, and packet loss."""
import itertools
import numpy as np
from amr_lab.core.world import World
from amr_lab.core.astar import astar
from amr_lab.core.allocators.hungarian import HungarianAllocator
from amr_lab.core.conflicts.reservation import ReservationResolver
from amr_lab.core.bus_filter import LossFilter

def small_world(): return World(('.....','.....','.....'))

def test_astar_shortest_path():
    path=astar(small_world(),(0,0),(4,2)); assert len(path)==6 and path[-1][0]==(4,2)

def test_hungarian_matches_brute_force():
    rng=np.random.default_rng(4); allocator=HungarianAllocator(); world=small_world(); cells=sorted(world.free)
    for n in range(2,5):
        robots={f'robot_{i}':cells[int(rng.integers(len(cells)))] for i in range(n)}
        tasks={f'task_{i}':{'start':cells[int(rng.integers(len(cells)))],'goal':cells[int(rng.integers(len(cells)))]} for i in range(n)}
        robot_ids=sorted(robots); task_ids=sorted(tasks)
        costs={r:{t:len(astar(world,robots[r],tasks[t]['start']))+len(astar(world,tasks[t]['start'],tasks[t]['goal'])) for t in task_ids} for r in robot_ids}
        assigned=allocator.assign({'world':world,'robots':robots,'tasks':tasks})
        actual=sum(costs[r][t] for r,ids in assigned.items() for t in ids)
        expected=min(sum(costs[r][perm[i]] for i,r in enumerate(robot_ids)) for perm in itertools.permutations(task_ids))
        assert actual==expected

def test_reservations_avoid_peer_vertex_and_swap():
    world=small_world(); resolver=ReservationResolver(world); peer={'robot_0':{'tick':3,'position':(2,1),'intent':[((2,2),4)]}}
    path=resolver.plan({'robot_id':'robot_1','tick':3,'position':(1,1),'goal':(4,1)},peer)
    assert path and all(not (cell==(2,1) and t==4) and not (cell==(2,2) and t==4) for cell,t in path)
    assert not (path[0][0]==(2,1) and peer['robot_0']['intent'][0][0]==(1,1))
    swap={'robot_0':{'tick':3,'position':(2,1),'intent':[((1,1),4)]}}
    detour=resolver.plan({'robot_id':'robot_1','tick':3,'position':(1,1),'goal':(2,1)},swap)
    assert detour and detour[0]!=( (2,1),4 )

def test_seeded_loss_rate():
    filt=LossFilter(.2,17); kept=sum(filt.keep() for _ in range(10000)); assert abs(kept/10000-.8)<.02

def test_planners_do_not_import_simulator_internals():
    import ast
    from pathlib import Path
    package=Path(__file__).parents[1]/'amr_lab'
    checked=list((package/'core').rglob('*.py'))+[package/'nodes'/'robot_node.py']
    for path in checked:
        tree=ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Import):
                assert all('sim_node' not in alias.name for alias in node.names), path
            if isinstance(node,ast.ImportFrom):
                assert 'sim_node' not in (node.module or ''), path
