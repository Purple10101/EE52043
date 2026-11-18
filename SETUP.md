# Setting up on a new machine

How to get this project running from scratch on Windows or Linux. Everything is in this one repo,
including the vision code (`robot_offline/`) and the models it loads (`models/`). Once it's set up, see
[README.md](README.md) for running the arm and [TOOLS.md](TOOLS.md) for the bench tools.

## 1. What you need first

| Tool | Windows | Linux (Ubuntu/Debian) |
| --- | --- | --- |
| Git | [git-scm.com](https://git-scm.com/download/win) | `sudo apt install git` |
| Python 3.11 | [python.org](https://www.python.org/downloads/) (tick "Add to PATH") | `sudo apt install python3.11 python3.11-venv python3-tk` |

You don't need Git LFS.

Optional, only for the parts you'll work on:

- **Unity 6000.3.6f1** for the simulator in `course_files/scara/mini_scara_simulator/` (install it through
  Unity Hub).
- **STM32CubeIDE**, or CMake + Ninja + `arm-none-eabi-gcc`, to rebuild the arm firmware. See
  [firmware/README.md](firmware/README.md).
- **Arduino IDE** with the ESP32 boards package, to flash the gripper (`gripper_firmware/`).

## 2. Clone

Clone into a **short** path (`C:\dev\` or `~/dev/`, not deep inside OneDrive), because some files in the
repo have long paths (see Troubleshooting).

### Windows (PowerShell)

```powershell
git config --global core.longpaths true        # once per machine: avoids "Filename too long"
mkdir C:\dev; cd C:\dev
git clone https://github.com/Purple10101/EE52043.git
```

### Linux (bash)

```bash
mkdir -p ~/dev && cd ~/dev
git clone https://github.com/Purple10101/EE52043.git
```

Use the **HTTPS** URL. The repo is public, so no account or key is needed to clone. The `git@github.com:`
URL only works if you have an SSH key set up on GitHub.

## 3. Python environment

Create the venv **inside `EE52043`**. Install torch **before** `requirements.txt`; otherwise ultralytics
pulls in the CPU-only build of torch.

### Windows

```powershell
cd C:\dev\EE52043
py -3.11 -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip

# With an NVIDIA GPU:
pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cu126
# ...or without one (CPU only, slower vision):
# pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cpu

pip install -r requirements.txt
```

### Linux

```bash
cd ~/dev/EE52043
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip

pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cu126
# or CPU only: --index-url https://download.pytorch.org/whl/cpu

pip install -r requirements.txt
```

There's nothing else to install: `robot_offline` is a plain folder in the repo, imported from the repo
root. Always run commands from the `EE52043` folder.

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
- **The camera is Windows-only for now.** `robot_offline/pre/camera.py` opens it through DirectShow
  (`cv2.CAP_DSHOW`), and its exposure values are DirectShow's. On Linux, everything except the live camera
  works: the tests, saved-frame tools, the simulator and the serial arm. Running the sorter with the real
  camera needs Windows.
- **The digital twin isn't in the repo.** It's kept on one machine only, so anything under `scripts/twin/`
  or that mentions the twin won't be there. Without it, the sorting window draws the bin slots on a plain
  background.
- Read the README's **"Before driving the real arm"** section before you power the servos.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `Filename too long` / `unable to create file` during clone (Windows) | Windows' 260-character path limit. Run `git config --global core.longpaths true`, delete the half-cloned folder, and clone again into a short path like `C:\dev`. |
| `Permission denied (publickey)` | You used the SSH URL. Use the `https://github.com/...` URL above. |
| `403` / `Permission to Purple10101/EE52043.git denied` | Cloning works for everyone, but pushing needs collaborator access. Ask the repo owner for an invite, or fork it. |
| `ModuleNotFoundError: No module named 'robot_offline'` or `'robot_control'` | You're not running from the `EE52043` folder, or the venv isn't active. |
| An old `robot_offline` gets imported (a setup from before 2026-11-18) | The venv still has the old separate repo installed. Run `pip uninstall robot_offline`; the copy in this repo is used after that. |
| `RPC failed` / `early EOF` on university Wi-Fi | The network dropped a large transfer (the models are about 56 MB). Try again, use another network, or clone with `--depth 1`. |
| `no camera accepts 4608 × 2592` | The camera isn't plugged in, something else has it open, or you're on Linux (see Hardware notes). |
