# TextToTurtleBot

```
.
├── core/               # Behavior tree, LLM logic
├── shared/             # Shared code between packages
├── web/                # Web dashboard + API
├── launch.sh           # Lab quickstart script
├── launch_simulator.sh # Simulator quickstart script
└── requirements.txt    # Python dependencies
```

## Prerequisites

* Python 3.10+
* On lab computers, if you see `(base)` as a prefix in the terminal, run `conda deactivate` to prevent dependency conflicts.

## Quickstart Guide (Lab)

1. Pick up the TurtleBot and place it on the docking station, it will boot automatically
2. Use a computer connected to the lab wifi (cps-labor)
3. Clone the repository (it might already exist at `~/texttoturtle`)
4. Check the config file at `core/llm_config.json` and set the LLM provider you would like to use as `default_provider`. Depending on your choice, create `core/.env` and set your API key, for example `OPENAI_API_KEY=sk-proj-...`
5. Run the launch script. The IP needs to match the namespace and is visible on the TurtleBot's built-in display.
```bash
chmod +x launch.sh # Needed only once
# ./launch.sh <namespace> <ip_address> <robot_ssh_password>
./launch.sh robot_1 192.168.0.226 turtlebot4
```
6. RViz and a camera view should now open automatically, the web interface can be accessed at http://localhost:8000

## Quickstart Guide (Simulator)

1. Take a look at the TurtleBot documentation at https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html to check the latest compatible versions of Ubuntu and ROS 2. It might make sense to use the same version running on the lab TurtleBots and PC (Ubuntu 24.04 / ROS 2 Jazzy at the time of writing).
2. Make sure you have an Ubuntu environment of the same version you picked above available. On Windows, using WSL can work sufficiently well, but make sure to use a device with a dedicated GPU if at all possible to keep the performance workable.
3. Follow the ROS 2 installation instructions for the version compatible with your operating system version: https://docs.ros.org/en/jazzy/Installation.html
4. Follow the TurtleBot 4 Simulator installation guide at https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
5. Clone the repository
6. Check the config file at `core/llm_config.json` and set the LLM provider you would like to use as `default_provider`. Depending on your choice, create `core/.env` and set your API key, for example `OPENAI_API_KEY=sk-proj-...`
7. Create a venv by running `python3 -m venv .venv` and activate it using `source .venv/bin/activate`
8. Install the dependencies by executing `pip3 install -r requirements.txt`
9. Run the launch script.
```bash
chmod +x launch_simulator.sh # Needed only once
./launch_simulator.sh
```
10. The Gazebo simulator, RViz and a camera view should now open automatically, the web interface can be accessed at http://localhost:8000

## Manual Guide (Lab)

1. Pick up the TurtleBot and place it on the docking station, it will boot automatically
2. Use a computer connected to the lab wifi (cps-labor)
3. Clone the repository (it might already exist at ~/texttoturtle)
4. Check the config file at `core/llm_config.json` and set the LLM provider you would like to use as `default_provider`. Depending on your choice, create `core/.env` and set your API key, for example `OPENAI_API_KEY=sk-proj-...`
5. First, SLAM and nav2 need to be launched. Preferably these should be ran on the Raspberry Pi of the TurtleBot to minimize issues related to the network latency and bandwidth, but they can also be ran on a lab computer at the cost of reliability. SSH into the TurtleBot using for example `ssh ubuntu@192.168.0.226` and the password `turtlebot4`. Then launch the following scripts in that order in separate SSH sessions or using tmux and replace the namespace with the one of the robot you are executing it on:
```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=robot_1
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot_1
```
6. Optional but useful: spawn an instance of RViz:
```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=robot_1
```
7. Launch the TextToTurtlebot application by starting both packages:
```bash
python3 -m core.main --namespace "/robot_1"
python3 -m web.backend.main --namespace "/robot_1"
```
8. The camera view should come up and the web server is accessible at http://localhost:8000

> [!NOTE]
> The launch script from the Quickstart Guide will automatically clean up any remaining processes from previous attempts. If you are following the manual steps, you will be left with residual processes running in the background after your first launch, which will eventually lead to multiple instances of the same ROS 2 service running and cause issues with nav2 and other services. Either restart the TurtleBot by holding down its power button or have a look at `launch.sh` for a list of processes to kill.

## Manual Guide (Simulator)

1. Take a look at the TurtleBot documentation at https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html to check the latest compatible versions of Ubuntu and ROS 2. It might make sense to use the same version running on the lab TurtleBots and PC (Ubuntu 24.04 / ROS 2 Jazzy at the time of writing).
2. Make sure you have an Ubuntu environment of the same version you picked above available. On Windows, using WSL can work sufficiently well, but make sure to use a device with a dedicated GPU if at all possible to keep the performance workable.
3. Follow the ROS 2 installation instructions for the version compatible with your operating system version: https://docs.ros.org/en/jazzy/Installation.html
4. Follow the TurtleBot 4 Simulator installation guide at https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
5. Clone the repository
6. Check the config file at `core/llm_config.json` and set the LLM provider you would like to use as `default_provider`. Depending on your choice, create `core/.env` and set your API key, for example `OPENAI_API_KEY=sk-proj-...`
7. Create a venv by running `python3 -m venv .venv` and activate it using `source .venv/bin/activate`
8. Install the dependencies by executing `pip3 install -r requirements.txt`
9. Launch the simulator using the following command and wait until the costmap (heatmap) becomes visible in RViz
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py slam:=true nav2:=true rviz:=true
```
10. Launch the TextToTurtlebot application by starting both packages:
```bash
python3 -m core.main --namespace "" --use_turtlebot_sim
python3 -m web.backend.main --namespace ""
```
11. The Gazebo simulator, RViz and a camera view should now open automatically, the web interface can be accessed at http://localhost:8000
