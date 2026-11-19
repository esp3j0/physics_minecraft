"""tests(CPU)共享夹具(E02h):把 tools/train 放进 sys.path,供 quad.* 导入。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools" / "train"))


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return ROOT


@pytest.fixture(scope="session")
def mjcf_path(repo_root) -> str:
    p = repo_root / "target" / "e02c" / "quad.xml"
    if not p.exists():
        pytest.skip(f"缺 MJCF:{p}(xtask export-mjcf 产物)")
    return str(p)


@pytest.fixture(scope="session")
def body_path(repo_root) -> str:
    return str(repo_root / "assets" / "bodies" / "quad.body.toml")


@pytest.fixture(scope="session")
def manifest_path(repo_root) -> str:
    return str(repo_root / "tools" / "train" / "quad" / "manifests" / "quad_train.toml")