"""Seeded independent packet loss for incoming bus messages."""
import random

class LossFilter:
    """A reproducible filter for simulated packet delivery."""
    def __init__(self, probability, seed):
        if not 0.0 <= probability <= 1.0: raise ValueError('packet loss must be between 0 and 1')
        self.probability=probability; self.rng=random.Random(seed); self.dropped=0
    def keep(self):
        """Return whether this incoming packet is delivered."""
        drop=self.rng.random()<self.probability; self.dropped+=int(drop); return not drop
