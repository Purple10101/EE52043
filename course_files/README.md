# Course files

Reference material from the module repo `github.bath.ac.uk/zr202/Applied_Robotics_Files`,
reorganised. The Python hardware APIs themselves live in `../hardware_api/`.

- `scara/API/python_examples/` - example scripts from the course (`scara_control.py` for the
  simulator, `scara_robot_test_script.py` for the real robot over serial).
- `scara/API/cpp/`, `scara/API/matlab/` - the same API in C++ and MATLAB.
- `scara/CAD/` - robot CAD (Fusion 360 `.f3d` and per-part `.step`).
- `scara/mini_scara_simulator/` - Unity simulator project (Assets, Packages, ProjectSettings
  only). Open in Unity Hub; `Library/`, `.sln` and `.csproj` are regenerated on first open.
- `scara/servo_motion_controller/` - STM32F446RE firmware (STM32CubeIDE project). Build output
  (`Debug/`) is not kept; rebuild in CubeIDE.
- `machine_vision/` - ESP32-CAM Arduino sketch and OpenCV Python scripts.
