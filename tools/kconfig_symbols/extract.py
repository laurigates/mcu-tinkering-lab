"""Write a committed Kconfig symbol list (run with PYTHONPATH=tools).

    python3 -m kconfig_symbols.extract idf <IDF_PATH> --version v5.4 \
        --source <image@digest> -o <file>
        Every symbol the ESP-IDF tree defines: `config`/`menuconfig` in the
        root Kconfig and under components/, plus the old names that
        sdkconfig.rename files still accept. Run inside the espressif/idf
        image (`just kconfig-symbols::idf` does).

    python3 -m kconfig_symbols.extract component <managed_components/ns__name> \
        --out-dir tools/kconfig_symbols/components
        Every symbol one managed component defines, with its version and its
        own dependencies from the idf_component.yml the component manager
        downloaded. Populate managed_components/ with a build first.

Stdlib only, and no imports from outside this package, because the idf mode
runs in the container with only this package mounted.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

from .symbols import (
    component_file_name,
    format_list,
    kconfig_symbols,
    manifest_dependencies,
    rename_symbols,
    tree_symbols,
)


def idf_list(idf_path: Path, version: str, source: str) -> str:
    # kconfgen reads the root Kconfig and every component's Kconfig files; the
    # tree walk skips test_apps/ and the like, which no project build sources.
    symbols = tree_symbols(idf_path / "components")
    root_kconfig = idf_path / "Kconfig"
    if root_kconfig.is_file():
        symbols |= kconfig_symbols(root_kconfig.read_text(encoding="utf-8"))
    root_rename = idf_path / "sdkconfig.rename"
    if root_rename.is_file():
        symbols |= rename_symbols(root_rename.read_text(encoding="utf-8"))
    header = {
        "what": "ESP-IDF Kconfig symbols (config/menuconfig, plus sdkconfig.rename old names)",
        "version": version,
        "source": source,
        "command": "just kconfig-symbols::idf",
    }
    return format_list(header, symbols)


def component_name(comp_dir: Path) -> str:
    """`managed_components/espressif__mdns` -> `espressif/mdns`;
    `external/bluepad32` (a checkout an EXTRA_COMPONENT_DIRS entry points
    into) -> `external/bluepad32`."""
    if comp_dir.parent.name == "external":
        return f"external/{comp_dir.name}"
    return comp_dir.name.replace("__", "/", 1)


def component_version(comp_dir: Path) -> str | None:
    manifest = comp_dir / "idf_component.yml"
    text = manifest.read_text(encoding="utf-8") if manifest.is_file() else ""
    m = re.search(r"^version:[ \t]*['\"]?([^'\"\s]+)", text, re.MULTILINE)
    return m.group(1) if m else None


def component_list(comp_dir: Path, version: str) -> str:
    name = component_name(comp_dir)
    manifest = comp_dir / "idf_component.yml"
    text = manifest.read_text(encoding="utf-8") if manifest.is_file() else ""
    header = {
        "what": f"Kconfig symbols of the component {name}",
        "component": name,
        "version": version,
        "depends": " ".join(manifest_dependencies(text)),
        "command": "just kconfig-symbols::component <project>/"
        + f"{comp_dir.parent.name}/{comp_dir.name}"
        + (f" {version}" if comp_dir.parent.name == "external" else ""),
    }
    return format_list(header, tree_symbols(comp_dir))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1].strip())
    sub = parser.add_subparsers(dest="mode", required=True)
    idf = sub.add_parser("idf")
    idf.add_argument("idf_path", type=Path)
    idf.add_argument("--version", required=True)
    idf.add_argument("--source", required=True)
    idf.add_argument("-o", "--output", type=Path, required=True)
    comp = sub.add_parser("component")
    comp.add_argument("comp_dir", type=Path)
    comp.add_argument(
        "--version",
        help="required for an external/ checkout, which has no idf_component.yml version",
    )
    comp.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="writes <ns>__<name>@<version>.txt (external/x: external__x@<tag>.txt) here",
    )
    args = parser.parse_args(argv)

    if args.mode == "idf":
        if not (args.idf_path / "components").is_dir():
            parser.error(f"{args.idf_path} has no components/ directory")
        text = idf_list(args.idf_path, args.version, args.source)
    else:
        if not args.comp_dir.is_dir():
            parser.error(f"{args.comp_dir} is not a directory")
        version = args.version or component_version(args.comp_dir)
        if not version:
            parser.error(
                f"{args.comp_dir} has no idf_component.yml version; pass --version"
            )
        text = component_list(args.comp_dir, version)
        name = component_file_name(component_name(args.comp_dir))
        args.output = args.out_dir / f"{name}@{version}.txt"
    count = sum(1 for line in text.splitlines() if line and not line.startswith("#"))
    if str(args.output) == "-":
        sys.stdout.write(text)
        print(f"{count} symbols", file=sys.stderr)
        return 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")
    print(f"{args.output}: {count} symbols")
    return 0


if __name__ == "__main__":
    sys.exit(main())
