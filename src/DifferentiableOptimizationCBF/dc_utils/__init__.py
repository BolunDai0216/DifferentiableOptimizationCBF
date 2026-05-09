from __future__ import annotations

from importlib.resources import as_file, files


def include_jl(name: str):
    """Resolve `name` inside this package directory and run `juliacall.Main.include` on it.

    Uses `importlib.resources` so it works regardless of how the package was
    installed (editable, wheel, zipapp, etc.).
    """
    from juliacall import Main

    with as_file(files(__name__) / name) as path:
        return Main.include(str(path))
