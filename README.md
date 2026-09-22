# Interactive Thrust Vector Control Demonstrator

An interactive senior design project that demonstrates how thrust vector control (TVC) can stabilize a rocket-like body.

Instead of combustion, the system uses an electric ducted fan (EDF) mounted on a **single-axis gimbal**. An inertial measurement unit (IMU) measures the rocket's angle, while a feedback controller redirects the thrust to keep the rocket upright.

The final system is intended to become an interactive science-center exhibit for elementary-school students, families, and school groups.

> **Project Status:** Active development. The main concept has been selected, but components and design specifications are still being tested and refined.

## Team

- Nate Sapp
- Warren Schindler
- Kailash Gupta-Verma
- Alec Garcia

## Project Goals

Here are our goals:
- Demonstrate thrust vector control in a visible and intuitive way.
- Actively stabilize a rocket-like body about one rotational axis.
- Allow visitors to interact with the system using a joystick.
- Provide immediate visual feedback.
- Be understandable without prior engineering knowledge.
- Operate safely and reliably in a public environment.
- Include a brief explanation for visitors interested in the engineering.

## Concept Overview

The demonstrator consists of a lightweight rocket-shaped body mounted to a single-axis pivot. The pivot is positioned below the system's center of gravity, making the rocket naturally unstable.

A gimbaled EDF produces thrust along the rocket's body. A servo rotates the EDF about one axis, changing the direction of the thrust and creating a corrective moment about the pivot.

An IMU continuously measures the rocket's angle and angular velocity. A feedback controller compares the measured angle with the desired angle and adjusts the gimbal to move the rocket toward its target orientation.

The user will interact with the system through a joystick. Depending on the final operating mode, the joystick may command a desired rocket angle, change the thrust level, or apply a temporary disturbance.

## How It Works

1. The IMU measures the rocket's angle and angular velocity.
2. The controller compares the measured angle with the desired angle.
3. A feedback algorithm calculates the required correction.
4. A servo rotates the EDF gimbal.
5. The redirected thrust creates a moment about the pivot.
6. The rocket moves toward the desired orientation.
7. The control loop repeats continuously to maintain stability.

The controller error is defined as:

error = desired angle - measured angle