# Kinova Controller

**Kinova Controller** is a Python-only library for controlling Kinova robots. It includes an optional software emergency stop (e-stop) that halts operations without powering off the robot.

## Installation

Run the following command in the project root to install the library:

```bash
pip install -e .
```

**Note**: We highly recommend using a virtual environment or a Conda environment for the installation to avoid dependency conflicts.

## Usage

### Running the Controller Directly on a Single Compute Machine

You can use `kinova.py` to control the robot directly from the compute machine:

```bash
python kinova.py
```

### Running with a NUC + Compute Machine Setup

#### On the NUC:
Run the robot server:

```bash
python arm_server.py
```

#### On the Compute Machine:
Run the robot client:

```bash
python arm_client.py
```

#### Optional:
You can also run the emergency stop script (to be documented).
