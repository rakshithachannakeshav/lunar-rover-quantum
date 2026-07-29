# Lunar Rover Web Simulation

Browser-based 3D preview with smooth controls — works on Windows without ROS.

## Open the UI

**Windows (PowerShell)** — use `;` not `&&`:

```powershell
cd D:\lunar-rover-quantum\web
python -m http.server 8080
```

**WSL / Linux** — use `python3`:

```bash
cd /mnt/d/lunar-rover-quantum/web
python3 -m http.server 8080
```

Or: `bash start.sh`

Then open **http://localhost:8080** (not the `file://` path).

Windows: double-click `start.bat`. WSL: `bash start.sh`.

## Controls

| Input | Action |
|-------|--------|
| W / ▲ | Forward |
| S / ▼ | Reverse |
| A / ◀ | Turn left |
| D / ▶ | Turn right |
| Space / ■ | Stop |
| **Patrol** tab | Autonomous drive along trail to orange goal |

## Gazebo (full physics)

**Terminal 1** — simulation:

```bash
source ~/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
ros2 launch rover_simulation demo.launch.py
```

**Default** — auto-drive to goal (no extra terminal):

```bash
ros2 launch rover_simulation demo.launch.py
```

**Terminal 2** — keyboard (install once: `sudo apt install python3-pynput`):

```bash
ros2 run rover_simulation rover_keyboard.py
```

**Test movement** (while sim runs):

```bash
bash /mnt/d/lunar-rover-quantum/sim_files/scripts/drive_test.sh
```

In Gazebo: press **F** to follow a model, or right-click `lunar_rover` → **Move To**.
