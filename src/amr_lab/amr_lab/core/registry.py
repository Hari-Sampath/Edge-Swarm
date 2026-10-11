"""Algorithm plug-in registries selected by parameter name."""
from .allocators.hungarian import HungarianAllocator
from .conflicts.reservation import ReservationResolver
ALLOCATORS={'hungarian': HungarianAllocator}
CONFLICTS={'reservation': ReservationResolver}
