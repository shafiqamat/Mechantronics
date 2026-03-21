# Mechantronics

This repository contains the firmware and documentation for a Mechatronics
final project: a MicroPython-controlled Romi robot that follows lines, logs
test data, and uses encoder plus IMU feedback for state estimation and
autonomous navigation.

## Project Structure

- `src/`: MicroPython source for drivers, cooperative tasks, and top-level robot setup.
- `docs/`: Sphinx source files for the project report and API documentation.
- `docs/_static/`: Images, videos, and custom static assets used by the docs site.
- `docs/_build/html/`: Generated HTML output after building the Sphinx docs.

## Main Components

- `motor.py`: Motor driver wrapper for PWM, direction, and sleep control.
- `encoder.py`: Quadrature encoder interface with position and speed tracking.
- `IMU_driver.py`: BNO055 IMU driver methods for heading, yaw rate, and acceleration.
- `task_motor.py`: Scheduler task for motor control and data logging.
- `task_controller.py`: Closed-loop control, line following, and game-track logic.
- `task_observer.py`: Pose/state observer using encoder and IMU measurements.
- `task_user.py`: Serial UI for tuning gains, starting tests, and exporting logs.
- `main.py`: Hardware setup and scheduler startup entry point.

## Documentation Workflow

Edit the reStructuredText files in `docs/` and rebuild the site with Sphinx.
Do not edit the generated HTML files in `docs/_build/html/`, because they are
overwritten on every build.

Build the docs from the `docs/` folder:

```bash
make html
```

Then open:

```text
docs/_build/html/index.html
```

## Runtime Notes

- The firmware is written for MicroPython and expects the `pyb`, `utime`, and
  related embedded modules provided on the target board.
- The Sphinx docs are configured to document the code on desktop Python, so the
  runtime startup in `main.py` is guarded to avoid launching the scheduler
  during doc builds.

## Authors

- Max Soury
- Shafiq Amat
