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

def test_loss_ratio_matches_configured_probability_per_attempt():
    filt=LossFilter(.2,90210)
    for _ in range(100000):filt.keep()
    assert filt.attempts==100000
    assert abs(filt.dropped/filt.attempts-.2)<.01

def test_recovery_turn_is_deterministic_and_ordered():
    world=small_world(); resolver=ReservationResolver(world)
    state={'robot_id':'robot_1','tick':21,'position':(1,1),'goal':(4,1),'wait_streak':10,'recovery_ticks':10,'robot_count':3,'recovering':True}
    peers={'robot_0':{'tick':20,'position':(3,1),'intent':[],'has_task':True},'robot_1':{'tick':20,'position':(2,2),'intent':[],'has_task':False}}
    assert resolver.plan(state,peers)==[]
    state['robot_id']='robot_0'; state['position']=(3,1); state['goal']=(0,1)
    assert resolver.plan(state,peers)

def test_idle_robot_gets_a_recovery_turn_from_global_rotation():
    world=small_world(); resolver=ReservationResolver(world)
    state={'robot_id':'robot_0','tick':3,'position':(0,0),'goal':(0,0),'recovery_ticks':3,'robot_count':3,'has_task':False}
    peers={'robot_1':{'tick':3,'position':(0,1),'intent':[],'recovering':True,'has_task':False}}
    path=resolver.plan(state,peers)
    assert path and path[0][0]!=state['position']

def test_idle_robot_stays_put_after_reaching_parking_cell():
    world=small_world(); resolver=ReservationResolver(world)
    state={'robot_id':'robot_0','tick':3,'position':(1,1),'goal':(1,1),'recovery_ticks':3,'robot_count':3,'has_task':False}
    peers={'robot_1':{'tick':3,'position':(0,1),'intent':[],'recovering':True,'has_task':False}}
    assert resolver.plan(state,peers)==[((1,1),4)]

def test_recovering_task_robot_takes_safe_step_toward_its_goal():
    world=small_world(); resolver=ReservationResolver(world)
    state={'robot_id':'robot_0','tick':3,'position':(1,1),'goal':(4,1),'recovery_ticks':3,'robot_count':3,'has_task':True,'recovering':True}
    peers={'robot_1':{'tick':3,'position':(4,2),'intent':[],'recovering':False}}
    path=resolver.plan(state,peers)
    assert path and path[0][0]==(2,1)


def test_seeded_dynamic_task_arrivals_repeat_and_release_once():
    from amr_lab.core.task_stream import SeededTaskStream
    tasks=small_world().tasks(19,12)
    first=SeededTaskStream(tasks,19,.35); second=SeededTaskStream(tasks,19,.35)
    arrivals_a=[first.arrivals() for _ in range(80)]
    arrivals_b=[second.arrivals() for _ in range(80)]
    assert arrivals_a==arrivals_b
    released=[task_id for batch in arrivals_a for task_id,_ in batch]
    assert released==sorted(tasks)
    immediate=SeededTaskStream(tasks,19,0.0)
    assert len(immediate.arrivals())==len(tasks)
    assert immediate.arrivals()==[]
