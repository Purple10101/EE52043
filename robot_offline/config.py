"""Loads the TOML files in robot_offline/configs/.

`load("dataset")` returns configs/dataset.toml as nested read-only tables. A missing key raises
ConfigError naming the file and the key, so a typo fails where it is used with a clear message.
"""

import tomllib
from collections.abc import Mapping
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]           # the EE52043 repo: models/, data/, runs/ and weights/ are here
CONFIG_DIR = Path(__file__).resolve().parent / "configs"
DATA_DIR = ROOT / "data"
# The segmentation model every tool runs unless told otherwise. seg_grey_v2_ft_20261123 (seg_grey_v2 fine-tuned
# on real rig frames, 2026-11-23: models/seg_grey_v2_ft_20261123/README.md); models/seg_grey_v2 to switch back.
SEG_WEIGHTS = ROOT / "models" / "seg_grey_v2_ft_20261123" / "best.pt"


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


def load(name, config_dir=CONFIG_DIR):
    """Reads <config_dir>/<name>.toml."""
    path = Path(config_dir) / f"{name}.toml"
    with open(path, "rb") as f:
        return Table(tomllib.load(f), path.name)
