"""Round based optimal task assignment."""
from scipy.optimize import linear_sum_assignment
import numpy as np
from ..astar import astar

class HungarianAllocator:
    """Assign one task to each available robot at minimum path cost."""
    def assign(self, state):
        """Return robot identifiers mapped to ordered task identifiers."""
        robots=state['robots']; tasks=state['tasks']; ids=sorted(robots); tids=sorted(tasks)
        if not ids or not tids: return {}
        costs=np.array([[len(astar(state['world'],robots[r],tasks[t]['start']))+len(astar(state['world'],tasks[t]['start'],tasks[t]['goal'])) for t in tids] for r in ids], dtype=float)
        ri,ci=linear_sum_assignment(costs); return {ids[i]:[tids[j]] for i,j in zip(ri,ci) if np.isfinite(costs[i,j])}
