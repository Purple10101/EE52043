# Setting up on a new machine

How to get this project running from scratch on Windows or Linux. It takes two repos, and they must sit
in a fixed layout relative to each other. Once it's set up, see [README.md](README.md) for running the
arm and [TOOLS.md](TOOLS.md) for the bench tools.

## 1. What you need first

| Tool | Why | Windows | Linux (Ubuntu/Debian) |
| --- | --- | --- | --- |
| Git | cloning | [git-scm.com](https://git-scm.com/download/win) | `sudo apt install git` |
| Git LFS | the vision models (about 220 MB) | included with Git for Windows | `sudo apt install git-lfs` |
| Python 3.11 | everything | [python.org](https://www.python.org/downloads/) (tick "Add to PATH") | `sudo apt install python3.11 python3.11-venv python3-tk` |

Run `git lfs install` once after installing Git LFS. **If you skip it, the model files come down as
small text pointers and the vision fails to load them.**

Optional, only for the parts you'll work on:

- **Unity 6000.3.6f1** for the simulator in `course_files/scara/mini_scara_simulator/` (install it through
  Unity Hub).
- **STM32CubeIDE**, or CMake + Ninja + `arm-none-eabi-gcc`, to rebuild the arm firmware. See
  [firmware/README.md](firmware/README.md).
- **Arduino IDE** with the ESP32 boards package, to flash the gripper (`gripper_firmware/`).

## 2. Clone both repos into this layout

```
<your workspace>/
├── EE52043/                     <- this repo (robot control, scripts, firmware)
└── Robot Resources/
    └── robot_offline/           <- EE52043_offline (vision: models, rectification, camera settings)
```

The folder names matter: the install command below and some script examples use the relative path
`../Robot Resources/robot_offline`. Pick a **short** workspace path (`C:\dev\` or `~/dev/`, not deep
inside OneDrive), because some files in the repo have long paths (see Troubleshooting).

### Windows (PowerShell)

```powershell
git config --global core.longpaths true        # once per machine: avoids "Filename too long"
git lfs install                                # once per machine

mkdir C:\dev; cd C:\dev
git clone https://github.com/Purple10101/EE52043.git
git clone https://github.com/Purple10101/EE52043_offline.git "Robot Resources\robot_offline"
```

### Linux (bash)

```bash
git lfs install                                # once per machine

mkdir -p ~/dev && cd ~/dev
git clone https://github.com/Purple10101/EE52043.git
git clone https://github.com/Purple10101/EE52043_offline.git "Robot Resources/robot_offline"
```

Use the **HTTPS** URLs above. Both repos are public, so no account or key is needed to clone. The
`git@github.com:` URLs only work if you have an SSH key set up on GitHub.

### Pulling only the models you need (optional, smaller download)

By default the second clone downloads every model (about 220 MB). To get only the three the robot uses
(about 56 MB), skip the automatic download and then pull just those:

```bash
# Linux; on Windows PowerShell, run  $env:GIT_LFS_SKIP_SMUDGE=1  first instead of the prefix
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Purple10101/EE52043_offline.git "Robot Resources/robot_offline"
cd "Robot Resources/robot_offline"
git lfs pull --include="models/seg_grey_v2/*,models/band_reader_kit_v1/*,models/resistor_tag_v1/*"
```

To check the models arrived, run `git lfs ls-files` inside `robot_offline`. Every line should show
`*`, not `-`. `models/seg_grey_v2/best.pt` should be about 55 MB, not a few hundred bytes.

## 3. Python environment

Create the venv **inside `EE52043`** and install everything into it. Run all of this from the `EE52043`
folder.

### Windows

```powershell
cd C:\dev\EE52043
py -3.11 -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt

# torch first, so ultralytics doesn't pull in a different build. With an NVIDIA GPU:
pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cu126
# ...or without one (CPU only, slower vision):
# pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cpu

pip install -e "..\Robot Resources\robot_offline"
```

### Linux

```bash
cd ~/dev/EE52043
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cu126
# or CPU only: --index-url https://download.pytorch.org/whl/cpu

pip install -e "../Robot Resources/robot_offline"
```

`robot_offline` must be installed with `-e` (editable): the code finds its `configs/` and `models/`
through the source folder, so don't move or rename that folder afterwards.

## 4. Check it works

```bash
python -m pytest -q                 # all tests should pass (no hardware needed)
python -c "import robot_offline, ultralytics, cv2, serial; print('ok')"
```

With no hardware, you can try the vision on saved frames:

```bash
python -m robot_control.task.live_plan --frames <folder of saved frames>
```

The README's commands are written for Windows (`.venv\Scripts\python scripts\...`). On Linux, use
`.venv/bin/python scripts/...` instead, or just `python scripts/...` with the venv activated.

## 5. Hardware notes

- **Serial ports are found automatically**: the Nucleo by its USB ID, the gripper's ESP32 by Espressif's.
  If more than one matches, pass `--port COM8` (Windows) or `--port /dev/ttyACM0` (Linux).
- **Linux serial permissions**: run `sudo usermod -aG dialout $USER` once, then log out and back in.
  Otherwise opening the port fails with "Permission denied".
- **The camera is Windows-only for now.** `robot_offline` opens it through DirectShow (`cv2.CAP_DSHOW`),
  and its exposure values are DirectShow's. On Linux, everything except the live camera works: the tests,
  saved-frame tools, the simulator and the serial arm. Running the sorter with the real camera needs Windows.
- **The digital twin isn't in either repo.** It's kept on one machine only, so anything under
  `scripts/twin/` or that mentions the twin won't be there. Without it, the sorting window draws the bin
  slots on a plain background.
- Read the README's **"Before driving the real arm"** section before you power the servos.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `Filename too long` / `unable to create file` during clone (Windows) | Windows' 260-character path limit. Run `git config --global core.longpaths true`, delete the half-cloned folder, and clone again into a short path like `C:\dev`. |
| `Permission denied (publickey)` | You used the SSH URL. Use the `https://github.com/...` URLs above. |
| `403` / `Permission to Purple10101/EE52043.git denied` | Cloning works for everyone, but pushing needs collaborator access. Ask the repo owner for an invite, or fork it. |
| `This repository is over its data quota` / LFS `batch response` errors | GitHub's LFS bandwidth for the month has run out. Clone with `GIT_LFS_SKIP_SMUDGE=1` (the repo still clones), then ask a teammate for the three model folders and copy them into `robot_offline/models/`. |
| `.pt not found (in the robot_offline repo, run git lfs pull)`, or `invalid load key 'v'` | The models are LFS pointers, not the real files. Run `git lfs install`, then `git lfs pull` inside `robot_offline`. |
| `ModuleNotFoundError: No module named 'robot_offline'` | The venv isn't active, or step 3's `pip install -e` was skipped or pointed at the wrong folder. |
| `RPC failed` / `early EOF` on university Wi-Fi | The network dropped a large transfer. Try again, use another network, or clone with `--depth 1`. |
| `no camera accepts 4608 × 2592` | The camera isn't plugged in, something else has it open, or you're on Linux (see Hardware notes). |
