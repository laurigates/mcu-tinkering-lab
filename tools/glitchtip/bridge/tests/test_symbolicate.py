import shutil

from robocar_glitchtip_bridge.symbolicate import (
    ElfStore,
    LineTable,
    load_line_table,
    lookup,
)


def test_lookup_hand_built_table():
    table = LineTable(
        rows=[(0x100, 0x108, "a.c", 10), (0x108, 0x110, "a.c", 11)],
        functions=[(0x100, 0x110, "f")],
    )
    hit = lookup(table, 0x10A)
    assert (hit.function, hit.filename, hit.lineno) == ("f", "a.c", 11)
    assert lookup(table, 0x110) is None  # ranges are end-exclusive
    assert lookup(table, 0x0FF) is None


def test_lookup_function_without_line_rows():
    table = LineTable(rows=[], functions=[(0x200, 0x240, "g")])
    hit = lookup(table, 0x210)
    assert (hit.function, hit.filename, hit.lineno) == ("g", None, None)


def test_symbolicates_compiled_elf(tiny_elf):
    table = load_line_table(tiny_elf)
    funcs = {name: (low, high) for low, high, name in table.functions}
    assert set(funcs) >= {"add", "entry"}

    # Every address inside add() names add() and a line of its body (1-4).
    low, high = funcs["add"]
    for addr in range(low, high):
        hit = lookup(table, addr)
        assert hit.function == "add"
        assert hit.filename.endswith("tiny.c")
        assert 1 <= hit.lineno <= 4
    # The `return a + b;` line has code of its own.
    assert any(lookup(table, a).lineno == 3 for a in range(low, high))

    low, high = funcs["entry"]
    assert any(lookup(table, a).lineno == 8 for a in range(low, high))


def test_elf_store_by_sha(tiny_elf, tmp_path):
    store = ElfStore(tmp_path)
    assert store.table_for("abc1234") is None
    shutil.copy(tiny_elf, tmp_path / "abc1234.elf")
    assert store.table_for("abc1234") is not None  # a miss is not cached
