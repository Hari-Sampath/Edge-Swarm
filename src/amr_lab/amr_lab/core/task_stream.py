"""Deterministic seeded Bernoulli task arrival stream."""
import random

class SeededTaskStream:
    """Release fixed task records over ticks using a seeded process."""
    def __init__(self, tasks, seed, arrival_rate):
        if not 0.0 <= arrival_rate <= 1.0: raise ValueError('arrival_rate must be between 0 and 1')
        self.tasks=tasks; self.ids=sorted(tasks); self.rng=random.Random(seed+7919)
        self.rate=arrival_rate; self.index=0; self.initial_released=False
    def arrivals(self):
        """Return newly arrived task id and record pairs for this tick."""
        if self.rate==0.0:
            if self.initial_released:return []
            self.initial_released=True; amount=len(self.ids)
        else:amount=int(self.index<len(self.ids) and self.rng.random()<self.rate)
        result=[(tid,self.tasks[tid]) for tid in self.ids[self.index:self.index+amount]]
        self.index+=amount
        return result
