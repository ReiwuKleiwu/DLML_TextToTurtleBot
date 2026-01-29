# TextToTurtleBot

## How to run

### 1. Install dependencies

```bash
python3 -m venv .venv
```
```bash
source .venv/bin/activate
```
```bash
pip3 install -r requirements.txt
```

### 2. Run the ROS2 nodes

```bash
python3 -m core.main
```
```bash
python3 -m web.backend.main
```