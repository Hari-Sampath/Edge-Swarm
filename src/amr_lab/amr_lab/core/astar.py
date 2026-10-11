"""Grid and space-time A-star search."""
import heapq

def astar(world, start, goal, blocked=None, horizon=80, start_time=0):
    """Find a shortest path, optionally avoiding reserved cell-time pairs."""
    blocked=blocked or set(); q=[(0,0,start,start_time)]; prev={}; cost={(start,start_time):0}; end=None
    while q:
        _,g,p,t=heapq.heappop(q); key=(p,t)
        if p==goal: end=key; break
        if t-start_time>=horizon: continue
        for n in (p,(p[0]+1,p[1]),(p[0]-1,p[1]),(p[0],p[1]+1),(p[0],p[1]-1)):
            nt=t+1
            if n not in world.free or (n,nt) in blocked or ((n,p,nt) in blocked): continue
            nk=(n,nt); ng=g+1
            if ng<cost.get(nk,10**9):
                cost[nk]=ng; prev[nk]=key; h=abs(goal[0]-n[0])+abs(goal[1]-n[1]); heapq.heappush(q,(ng+h,ng,n,nt))
    if end is None: return []
    path=[]
    while end in prev: path.append(end); end=prev[end]
    return list(reversed(path))
