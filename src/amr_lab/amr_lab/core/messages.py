"""Plain data records used on the simulated robot bus."""
from dataclasses import dataclass, asdict
import json

@dataclass
class BusMessage:
    """One robot's position and future cell reservations."""
    robot_id: str
    tick: int
    position: tuple
    intent: list
    def encode(self):
        """Serialize this record as JavaScript Object Notation."""
        return json.dumps(asdict(self))
    @classmethod
    def decode(cls, raw):
        """Rebuild the record from its bus string."""
        d=json.loads(raw); d['position']=tuple(d['position']); d['intent']=[(tuple(p),t) for p,t in d['intent']]; return cls(**d)
