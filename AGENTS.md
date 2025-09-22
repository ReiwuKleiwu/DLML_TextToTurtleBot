# Repository Guidelines

## Project Structure & Module Organization
- `code/` holds application code: `main.py` wires nodes; `classes/` contains modules grouped by capability (e.g., `navigation/`, `perception/`, `controllers/`).
- `models/` stores downloaded model weights; keep large binaries out of Git.
- `slam_navigator.py`, `test_nav.py`, and `test_slam_nav.py` are executable samples for Nav2 and SLAM flows.
- `DLML_TTT_Venv/` is a local virtualenv; recreate rather than commit changes.

## Build, Test, and Development Commands
- `python -m venv DLML_TTT_Venv && source DLML_TTT_Venv/bin/activate` creates an isolated dev environment (ROS overlay still required).
- `pip install -r code/requirements.txt` installs Python dependencies.
- `python code/main.py` runs the text-to-TurtleBot orchestrator (ensure `ros2` workspace is sourced and simulator is running).
- `python test_nav.py` exercises Nav2 integration against a running TurtleBot4 stack.
- `python test_slam_nav.py` verifies the SLAM navigation helper; cancel with Ctrl+C if stuck.

## Coding Style & Naming Conventions
- Follow PEP 8 with 4-space indentation; keep lines ≤ 100 chars.
- Modules and files use `lower_snake_case`; classes remain `CamelCase`; ROS topics/services stay descriptive (e.g., `navigation_goal`).
- Prefer type hints for new public APIs and add concise docstrings for behaviors or controllers.
- Keep ROS node side-effects inside class methods; avoid global state outside `main.py`.

## Commit & Pull Request Guidelines
- Match the `FEAT :: Description` style seen in `git log`; prefix with `FEAT`, `FIX`, or `DOC` as appropriate.
- Each PR should describe scenario setup (sim vs. hardware), link related issues, and include screenshots or log excerpts for navigation runs.
- Request review from perception and navigation owners when touching their folders; mention new models or assets explicitly.

## ROS & Environment Notes
- Source your ROS 2 Humble (or project target) setup before running scripts: `source /opt/ros/humble/setup.bash`.
- Ensure Nav2 bring-up is active (`ros2 launch turtlebot4_navigation nav2.launch.py`) before executing navigation scripts; failure to do so leads to hangs.
