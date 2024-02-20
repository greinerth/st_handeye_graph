from __future__ import annotations

import importlib.metadata

import sthandeyegraphpy as m


def test_version():
    assert importlib.metadata.version("st_handeye_graph_py") == m.__version__
