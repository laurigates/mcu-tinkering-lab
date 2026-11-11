"""Circuit blocks: a stage designed once, used from any hardware.toml (#976).

A block is `docs/reference/blocks/<slug>.toml`: registry parts with no refs,
the rails between them, and ports, each a list of block pins. A project uses
one with a `[blocks.<id>]` table:

    [blocks.buck]
    block = "tps563208-5v"
    net_prefix = "BUCK"                    # internal rail SW becomes BUCK_SW
    refs = { ic = "U10", l = "L1", ... }   # one per block part, no more
    net_names = { SW = "BUCK_SW" }         # optional: name an internal rail
    parts.fb_top = { value = "56k" }       # optional: override value/lcsc/...
    build = "carrier"                      # optional: on every entry

and connects its ports by writing `<id>.<PORT>` wherever a part pin may go
(`to = ["buck.VIN"]`). `expand()` turns the instance into plain hardware.toml
entries before anything else reads the file: part `<key>` becomes
`[parts.<id>_<key>]` with its ref, each internal rail is appended to
`[[rails]]`, and each port reference becomes the port's pins. The join,
tools/kicad and kicad.lcsc then see an ordinary sidecar, so the netlist, the
schematic and the check stamp of a project re-expressed through blocks stay
byte-identical, and nothing downstream knows blocks exist.

What stays per project: refs (they are the board's), the rails that join the
block to the rest of the circuit, and any part the block's datasheet does not
call for (a bulk capacitor sized to the pack, a test header).
"""

from __future__ import annotations

import copy
import re
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .errors import HardwareError

BLOCKS = Path("docs/reference/blocks")
REPO_ROOT = Path(__file__).resolve().parents[2]

_INSTANCE_KEYS = {"block", "net_prefix", "refs", "net_names", "parts", "build"}
_BLOCK_KEYS = {"block", "ports", "parts", "rails"}
_HEAD_KEYS = {"name", "source", "note"}
_RAIL_KEYS = {"name", "from", "to"}
# What an instance may change on a block part. The part slug, kind and ref
# are the block's design (ref comes from `refs`), not a per-project choice.
_OVERRIDES = {"name", "note", "value", "lcsc", "buy"}
_REF = re.compile(r"[A-Z]+[0-9]+")
_ID = re.compile(r"[a-z][a-z0-9_]*")
# Tables whose `from`/`to` may name a part pin, and so a port.
_ENDPOINT_TABLES = ("rails", "nets", "channel_nets", "bus_nets", "outputs")


@dataclass(frozen=True)
class Block:
    slug: str
    path: Path
    ports: dict[str, list[str]]  # port -> block pins ("<key>.<pin>")
    parts: dict[str, dict[str, Any]]
    rails: list[dict[str, Any]]


def load_block(slug: str, repo_root: Path = REPO_ROOT) -> Block:
    """Read and check `docs/reference/blocks/<slug>.toml`."""
    path = repo_root / BLOCKS / f"{slug}.toml"
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise HardwareError(f"block {slug!r}: {path} not found") from None
    except tomllib.TOMLDecodeError as e:
        raise HardwareError(f"{path}: {e}") from None
    _keys(str(path), data, _BLOCK_KEYS)
    head = data.get("block", {})
    _keys(f"{path} [block]", head, _HEAD_KEYS)
    if not head.get("name") or not head.get("source"):
        raise HardwareError(f"{path} [block]: name and source are required")
    parts = data.get("parts", {})
    if not isinstance(parts, dict) or not parts:
        raise HardwareError(f"{path}: a block needs at least one [parts.<key>]")
    for key, table in parts.items():
        _id(f"{path} [parts.{key}]", key)
        if not isinstance(table, dict):
            raise HardwareError(f"{path} [parts.{key}]: expected a table")
        for fixed in ("ref", "build"):
            if fixed in table:
                raise HardwareError(
                    f"{path} [parts.{key}]: {fixed!r} belongs to the instance, not the block"
                )
    ports = data.get("ports", {})
    if not isinstance(ports, dict) or not ports:
        raise HardwareError(f"{path}: a block needs at least one port under [ports]")
    for port, pins in ports.items():
        if not isinstance(pins, list) or not pins:
            raise HardwareError(f"{path} [ports] {port}: expected a list of pins")
        for pin in pins:
            _local_pin(f"{path} [ports] {port}", pin, parts)
    rails = data.get("rails", [])
    for i, rail in enumerate(rails):
        where = f"{path} [[rails]] #{i + 1}"
        _keys(where, rail, _RAIL_KEYS)
        if not rail.get("name") or not rail.get("from"):
            raise HardwareError(f"{where}: name and from are required")
        for pin in [rail["from"], *rail.get("to", [])]:
            _local_pin(where, pin, parts)
    return Block(slug, path, ports, parts, rails)


def expand(
    data: dict[str, Any], where: str, repo_root: Path = REPO_ROOT
) -> dict[str, Any]:
    """`data` (a parsed hardware.toml) with every `[blocks.*]` instance expanded.

    Returns a new dict; `data` is not changed. Without `blocks` it is a copy.
    """
    out = copy.deepcopy(data)
    instances = out.pop("blocks", {})
    if not instances:
        return out
    if not isinstance(instances, dict):
        raise HardwareError(f"{where} [blocks]: expected a table of instances")
    parts = out.setdefault("parts", {})
    rails = out.setdefault("rails", [])
    rail_names = {r.get("name") for r in rails}
    ports: dict[str, tuple[str, list[str]]] = {}  # "<id>.<PORT>" -> (where, pins)
    for inst_id, inst in instances.items():
        iw = f"{where} [blocks.{inst_id}]"
        _id(iw, inst_id)
        if inst_id in parts:
            raise HardwareError(f"{iw}: a part is already named {inst_id!r}")
        _keys(iw, inst, _INSTANCE_KEYS)
        block = load_block(_required(iw, inst, "block"), repo_root)
        prefix = _required(iw, inst, "net_prefix")
        build = inst.get("build")
        refs = inst.get("refs", {})
        missing = sorted(set(block.parts) - set(refs))
        if missing:
            raise HardwareError(f"{iw}: no ref for block part(s) {missing}")
        extra = sorted(set(refs) - set(block.parts))
        if extra:
            raise HardwareError(
                f"{iw} refs: {extra} not in block {block.slug!r}; "
                f"its parts are {sorted(block.parts)}"
            )
        overrides = inst.get("parts", {})
        unknown = sorted(set(overrides) - set(block.parts))
        if unknown:
            raise HardwareError(f"{iw} parts: {unknown} not in block {block.slug!r}")

        def key_of(local: str, inst_id: str = inst_id) -> str:
            return f"{inst_id}_{local}"

        def pin_of(local_pin: str) -> str:
            key, pin = local_pin.split(".", 1)
            return f"{key_of(key)}.{pin}"

        for key, table in block.parts.items():
            ref = refs[key]
            if not isinstance(ref, str) or not _REF.fullmatch(ref):
                raise HardwareError(
                    f"{iw} refs.{key}: {ref!r} is not a reference like R12"
                )
            override = overrides.get(key, {})
            _keys(f"{iw} parts.{key}", override, _OVERRIDES)
            name = key_of(key)
            if name in parts:
                raise HardwareError(f"{iw}: part {name!r} is already defined")
            entry = {**table, **override, "ref": ref}
            if build:
                entry["build"] = build
            parts[name] = entry

        local_rails = {r["name"] for r in block.rails}
        names = inst.get("net_names", {})
        stray = sorted(set(names) - local_rails)
        if stray:
            raise HardwareError(
                f"{iw} net_names: {stray} not a rail of block {block.slug!r}; "
                f"its rails are {sorted(local_rails)}"
            )
        for rail in block.rails:
            name = names.get(rail["name"], f"{prefix}_{rail['name']}")
            if name in rail_names:
                raise HardwareError(f"{iw}: rail {name!r} is already defined")
            rail_names.add(name)
            new = {
                "name": name,
                "from": pin_of(rail["from"]),
                "to": [pin_of(p) for p in rail.get("to", [])],
            }
            if build:
                new["build"] = build
            rails.append(new)

        for port, pins in block.ports.items():
            ports[f"{inst_id}.{port}"] = (iw, [pin_of(p) for p in pins])

    used: set[str] = set()
    for table_name in _ENDPOINT_TABLES:
        for i, table in enumerate(out.get(table_name, [])):
            tw = f"{where} [[{table_name}]] #{i + 1}"
            _substitute(tw, table_name, table, ports, instances, used)
    for ref, (iw, _) in ports.items():
        if ref not in used:
            port = ref.split(".", 1)[1]
            raise HardwareError(f"{iw}: port {port} is not connected to anything")
    return out


def load_sidecar(path: Path, repo_root: Path = REPO_ROOT) -> dict[str, Any]:
    """Parse a hardware.toml and expand its blocks: what every reader should use."""
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except tomllib.TOMLDecodeError as e:
        raise HardwareError(f"{path}: {e}") from e
    return expand(data, str(path), repo_root)


def _substitute(
    where: str,
    table_name: str,
    table: dict[str, Any],
    ports: dict[str, tuple[str, list[str]]],
    instances: dict[str, Any],
    used: set[str],
) -> None:
    def resolve(value: Any) -> list[str] | None:
        if not isinstance(value, str) or "." not in value:
            return None
        inst_id = value.split(".", 1)[0]
        if inst_id not in instances:
            return None
        if value not in ports:
            known = sorted(
                p.split(".", 1)[1] for p in ports if p.startswith(f"{inst_id}.")
            )
            raise HardwareError(
                f"{where}: {value!r} is not a port of block {inst_id}; its ports are {known}"
            )
        used.add(value)
        return ports[value][1]

    to = table.get("to")
    if isinstance(to, list):
        new_to: list[Any] = []
        for item in to:
            pins = resolve(item)
            new_to.extend(pins if pins is not None else [item])
        table["to"] = new_to
    elif to is not None:
        pins = resolve(to)
        if pins is not None:
            table["to"] = _single(where, to, pins)

    src = table.get("from")
    pins = resolve(src)
    if pins is not None:
        if table_name == "rails":
            table["from"] = pins[0]
            table["to"] = [*pins[1:], *table.get("to", [])]
        else:
            table["from"] = _single(where, src, pins)


def _single(where: str, ref: str, pins: list[str]) -> str:
    if len(pins) != 1:
        port = ref.split(".", 1)[1]
        raise HardwareError(
            f"{where}: port {port} has {len(pins)} pins and this field takes one; "
            f"connect it from a [[rails]] entry"
        )
    return pins[0]


def _local_pin(where: str, pin: Any, parts: dict[str, Any]) -> None:
    if not isinstance(pin, str) or "." not in pin:
        raise HardwareError(f"{where}: {pin!r} is not '<part>.<pin>'")
    key = pin.split(".", 1)[0]
    if key not in parts:
        raise HardwareError(
            f"{where}: {pin!r} names no block part; parts are {sorted(parts)}"
        )


def _keys(where: str, table: Any, allowed: set[str]) -> None:
    if not isinstance(table, dict):
        raise HardwareError(f"{where}: expected a table, got {table!r}")
    unknown = sorted(set(table) - allowed)
    if unknown:
        raise HardwareError(
            f"{where}: unknown key(s) {unknown}; allowed: {sorted(allowed)}"
        )


def _required(where: str, table: dict[str, Any], key: str) -> str:
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise HardwareError(f"{where}: {key} is required")
    return value


def _id(where: str, name: str) -> None:
    if not _ID.fullmatch(name):
        raise HardwareError(f"{where}: {name!r} must be lower_snake_case")
