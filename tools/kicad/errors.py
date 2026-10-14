"""The one exception tools/kicad raises.

Every malformed or unresolvable input (a symbol missing from the vendored
library, a MANIFEST hash that does not match, a board.toml with an unknown key)
is a KicadError carrying the file and the offending value. A CLI's `main`
prints it as one `FAIL` line and returns 1, and a test can assert which input
was rejected.
"""


class KicadError(Exception):
    """An input to tools/kicad is malformed or does not resolve."""
