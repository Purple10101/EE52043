"""TOML settings files (in this folder), loaded into frozen settings objects.

`load("workspace")` returns workspace.toml as nested read-only tables; a missing key raises
ConfigError naming the file and the key. The `*.from_config()` constructors below read one file
each and check it, so a bad value fails at startup rather than mid-move.
"""

import json
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CONFIG_DIR = Path(__file__).resolve().parent


class ConfigError(Exception):
    pass


class Table(Mapping):
    """One TOML table. Nested tables come back as Tables, everything else as parsed."""

    def __init__(self, data, file, path=""):
        self._data = data
        self._file = file
        self._path = path

    def __getitem__(self, key):
        dotted = f"{self._path}.{key}" if self._path else key
        try:
            value = self._data[key]
        except KeyError:
            raise ConfigError(f"{self._file}: missing key '{dotted}'") from None
        if isinstance(value, dict):
            return Table(value, self._file, dotted)
        return value

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    def range(self, key):
        """A [min, max] pair as a tuple of floats, checked min < max."""
        lo, hi = (float(v) for v in self[key])
        if not lo < hi:
            raise ConfigError(f"{self._file}: '{self._path}.{key}' must be [min, max] with min < max")
        return lo, hi


def load(name, config_dir=CONFIG_DIR):
    """Reads <config_dir>/<name>.toml."""
    path = Path(config_dir) / f"{name}.toml"
    with open(path, "rb") as f:
        return Table(tomllib.load(f), path.name)


@dataclass(frozen=True)
class ArmSettings:
    """The arm's geometry and the joint limits the firmware enforces. Metres and degrees."""

    link1: float
    link2: float
    j0: tuple[float, float]         # base
    j1: tuple[float, float]         # elbow
    j2: tuple[float, float]         # wrist
    z: tuple[float, float]          # slide range sent to SCARA_INITIALISE

    @classmethod
    def from_config(cls, name="robot_real"):
        t = load(name)
        arm, limits = t["arm"], t["limits"]
        s = cls(float(arm["link1_m"]), float(arm["link2_m"]), limits.range("j0_deg"),
                limits.range("j1_deg"), limits.range("j2_deg"), limits.range("z_m"))
        if s.link1 <= 0 or s.link2 <= 0:
            raise ConfigError(f"{name}.toml: link lengths must be positive")
        return s


@dataclass(frozen=True)
class RigSettings:
    """Where the rig scene frame (mm, origin at the base axis, z up; rectify's plate mm) sits in the
    robot frame (metres, the firmware's): robot = R(rotation) · scene / 1000 + translation."""

    rotation_deg: float
    translation_m: tuple[float, float]

    @classmethod
    def from_config(cls):
        t = load("workspace")["scene_to_robot"]
        return cls(float(t["rotation_deg"]), tuple(float(v) for v in t["translation_m"]))


@dataclass(frozen=True)
class Package:
    name: str
    length: float                   # body, m
    width: float


@dataclass(frozen=True)
class WorkspaceSettings:
    packages: tuple[Package, ...]
    size_tolerance: float           # share of a package's length / width a body may differ by
    hover_z: float                  # slide value for travel above the plate (m)
    move_time: float                # seconds per move

    @classmethod
    def from_config(cls):
        t = load("workspace")
        packages = tuple(Package(name, float(p["length_mm"]) / 1000, float(p["width_mm"]) / 1000)
                         for name, p in t["packages"].items())
        if not packages:
            raise ConfigError("workspace.toml: at least one package is needed")
        motion = t["motion"]
        return cls(packages, float(t["size_tolerance"]), float(motion["hover_z_m"]), float(motion["move_time_s"]))


@dataclass(frozen=True)
class GripperSettings:
    name: str
    tool_offset: tuple[float, float]  # grip point from the wrist axis in the tool frame, m
    tip_below_shaft: float          # fingertips below the Z shaft's bottom face, m
    symmetric: bool                 # gripping at yaw + 180 deg is the same grip
    jaw_axis_at_zero_deg: float     # robot-frame direction the jaws close along at tool angle 0

    @classmethod
    def from_config(cls, name="parallelogram_v4"):
        t = load(name, CONFIG_DIR / "grippers")
        return cls(name, tuple(float(v) / 1000 for v in t["tool_offset_mm"]), float(t["tip_below_shaft_mm"]) / 1000,
                   bool(t["symmetric"]), float(t["jaw_axis_at_zero_deg"]))


@dataclass(frozen=True, eq=False)
class KeepOutMap:
    """Joint angles at which the arm would hit an obstacle, on a grid (scripts/keep_out_map.py)."""

    name: str
    j0: tuple[float, float]         # (first, step) of the base grid, deg
    j1: tuple[float, float]         # (first, step) of the elbow grid, deg
    blocked: np.ndarray             # bool, [base index, elbow index]
    margin_mm: float

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        blocked = np.array([[c == "1" for c in row] for row in d["blocked"]])
        j0, j1 = d["j0_deg"], d["j1_deg"]
        if blocked.shape != (round((j0[1] - j0[0]) / j0[2]) + 1, round((j1[1] - j1[0]) / j1[2]) + 1):
            raise ConfigError(f"{Path(path).name}: 'blocked' doesn't match the j0_deg and j1_deg grids")
        return cls(d["name"], (j0[0], j0[2]), (j1[0], j1[2]), blocked, float(d["margin_mm"]))

    @classmethod
    def from_config(cls):
        """Every map named in workspace.toml's keep_out list, from config/keep_out/."""
        return tuple(cls.load(CONFIG_DIR / "keep_out" / f"{name}.json") for name in load("workspace")["keep_out"])


@dataclass(frozen=True)
class VisionSettings:
    weights: Path                   # the segmentation model; relative paths are in the robot_offline repo
    conf: float
    min_confidence: float | None = None     # the band reader's limit for the sorting run; None: the reader's own

    @classmethod
    def from_config(cls):
        t = load("vision")
        limit = t.get("min_confidence")
        return cls(Path(t["weights"]), float(t["conf"]), None if limit is None else float(limit))
