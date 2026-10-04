"""The one exception the join raises.

Every malformed input — a conflicting `#define`, an unreadable board table, a
net naming a role the header does not define — is a HardwareError carrying the
file and the offending value, so a caller can fail with one clear line instead
of a traceback, and a test can assert which input was rejected.
"""


class HardwareError(Exception):
    """An input to the hardware join is malformed or does not resolve."""
