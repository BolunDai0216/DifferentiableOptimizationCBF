from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

DC_UTILS_DIR = Path(__file__).parent.parent / "dc_utils"


def load_julia_functions() -> tuple[Callable[..., Any], Callable[..., Any]]:
    from juliacall import Main as jl

    return (
        jl.include(str(DC_UTILS_DIR / "unicycle_env_setup.jl")),
        jl.include(str(DC_UTILS_DIR / "get_cbf_unicycle_env.jl")),
    )
