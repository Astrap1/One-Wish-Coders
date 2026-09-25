"""Internal lifecycle phases used by the safety supervisor.

These phases are deliberately separate from the public CRUISE, CAUTION, HOLD
and RETURN safety states. They are used only to decide whether a new route is
an outbound or a return route.
"""

from enum import Enum


class MissionPhase(str, Enum):
    """The lifecycle known to the safety supervisor."""

    PRELAUNCH = "PRELAUNCH"
    OUTBOUND = "OUTBOUND"
    DELIVERED = "DELIVERED"
    RETURNING = "RETURNING"
