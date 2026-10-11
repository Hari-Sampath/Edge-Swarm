"""Grid warehouse data and deterministic task generation."""
from dataclasses import dataclass
from collections import deque
import random

@dataclass
class World:
    """Walkable grid and named narrow resources."""
    rows: tuple[str, ...]
    @property
    def free(self):
        return {(x, y) for y, row in enumerate(self.rows) for x, c in enumerate(row) if c != '#'}
    @property
    def chokes(self):
        return {i: (x, y) for i, (x, y) in enumerate(sorted(c for c in self.free if sum((c[0]+dx,c[1]+dy) in self.free for dx,dy in ((1,0),(-1,0),(0,1),(0,-1))) == 2))}
    @property
    def parking_cells(self):
        """Return open aisle junctions where an idle robot can wait."""
        return sorted(c for c in self.free if sum((c[0]+dx,c[1]+dy) in self.free for dx,dy in ((1,0),(-1,0),(0,1),(0,-1))) >= 3)
    def distance(self, start, goal):
        """Return shortest grid distance or infinity when unreachable."""
        q=deque([(start,0)]); seen={start}
        while q:
            p,d=q.popleft()
            if p==goal: return d
            for n in ((p[0]+1,p[1]),(p[0]-1,p[1]),(p[0],p[1]+1),(p[0],p[1]-1)):
                if n in self.free and n not in seen: seen.add(n); q.append((n,d+1))
        return float('inf')
    def tasks(self, seed, count=12):
        """Make fixed reproducible pickup and delivery goals."""
        rng=random.Random(seed); cells=sorted(self.free); return {f't{i}': {'start': cells[rng.randrange(len(cells))], 'goal': cells[rng.randrange(len(cells))]} for i in range(count)}
