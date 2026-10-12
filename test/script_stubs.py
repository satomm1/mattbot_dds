"""Import scripts/ nodes and dds_utils.transform without ROS / cyclonedds (test helper, not a test module)."""

import contextlib
import importlib
import importlib.abc
import importlib.util
import os
import re
import sys
import types
from unittest import mock

import numpy as np

HERE = os.path.dirname(__file__)
SCRIPTS = os.path.join(HERE, "..", "scripts")
DDS_UTILS_SRC = os.path.join(HERE, "..", "src", "dds_utils")

STUB_ROOTS = {
    "cyclonedds",
    "database_utils",
    "geometry_msgs",
    "mattbot_bringup",
    "mattbot_dds",
    "mattbot_image_detection",
    "nav_msgs",
    "navigation_utils",
    "rospkg",
    "rospy",
    "rospy_message_converter",
    "sensor_msgs",
    "std_msgs",
    "tf",
}


class Float64MultiArray:
    def __init__(self, data=None):
        self.data = list(data or [])


class _AutoModule(types.ModuleType):
    """Module whose unknown attributes are MagicMocks (stable per name)."""

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        value = mock.MagicMock(name=f"{self.__name__}.{name}")
        setattr(self, name, value)
        return value


class _Listener:
    def __init__(self, *args, **kwargs):
        pass


_OVERRIDES = {
    "cyclonedds.core": {"Listener": _Listener},
    "std_msgs.msg": {"Float64MultiArray": Float64MultiArray},
}


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname.split(".")[0] in STUB_ROOTS:
            return importlib.util.spec_from_loader(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        return _AutoModule(spec.name)

    def exec_module(self, module):
        module.__path__ = []
        for k, v in _OVERRIDES.get(module.__name__, {}).items():
            setattr(module, k, v)


def _message_constants():
    with open(os.path.join(DDS_UTILS_SRC, "messages.py")) as f:
        return dict(re.findall(r'^(MSG_\w+) = "([^"]*)"', f.read(), re.M))


def _load_glue():
    """Real src/dds_utils/transform.py (+ map_transform.py) without running dds_utils/__init__."""
    pkg = types.ModuleType("_dds_utils_glue")
    pkg.__path__ = [DDS_UTILS_SRC]
    sys.modules["_dds_utils_glue"] = pkg
    return importlib.import_module("_dds_utils_glue.transform")


@contextlib.contextmanager
def stubbed_imports():
    """Within the block, ROS/DDS packages are stubs and `dds_utils` exposes the real transform glue."""
    with mock.patch.dict(sys.modules):
        for name in list(sys.modules):
            if name.split(".")[0] in STUB_ROOTS | {"dds_utils", "_dds_utils_glue"}:
                del sys.modules[name]
        finder = _StubFinder()
        sys.meta_path.insert(0, finder)
        try:
            glue = _load_glue()
            dds_utils = _AutoModule("dds_utils")
            for k, v in _message_constants().items():
                setattr(dds_utils, k, v)
            dds_utils.TransformMixin = glue.TransformMixin
            dds_utils.DdsLogger = lambda _name: mock.MagicMock()
            sys.modules["dds_utils"] = dds_utils
            yield glue
        finally:
            sys.meta_path.remove(finder)


def import_script(name):
    """Fresh import of scripts/<name>.py; call inside stubbed_imports()."""
    sys.modules.pop(name, None)
    with mock.patch.object(sys, "path", [SCRIPTS] + sys.path):
        return importlib.import_module(name)


class RecordingTf:
    """Stand-in MapTransform: records (point, forward) and shifts x by +/-100 so outputs are traceable."""

    model = "tps"
    fallback_reason = None
    R = np.identity(2)
    t = np.zeros(2)

    def __init__(self):
        self.calls = []

    def apply(self, point, forward=True):
        self.calls.append((tuple(float(v) for v in point), forward))
        out = np.array(point, dtype=float)
        out[0] += 100.0 if forward else -100.0
        return out

    @property
    def directions(self):
        return [fwd for _, fwd in self.calls]
