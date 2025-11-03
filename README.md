# 6DOF GNC Simulation (Simulink/C++)

A high-fidelity 6-DOF (Six-Degrees-of-Freedom) simulation of a Falcon 9-like launch vehicle's atmospheric flight phase, from launch pad to stage separation. Features a complete Guidance, Navigation, and Control system written in C++ that runs in a co-simulation environment with a detailed physics model built in Simulink.



1. [Overview & Architecture](#overview--architecture)
2. [The Simulation Model (Simulink)](#the-simulation-model-simulink)
   - [6-DOF Physics Engine](#6-dof-physics-engine)
   - [Variable Mass & Gravity Model](#variable-mass--gravity-model)
   - [Aerodynamics Model](#aerodynamics-model)
   - [UDP Send/Receive](#udp-send-receive)
3. [Flight Software (C++)](#the-flight-software-c)
   - [State Machine](#state-machine)
   - [Communication](#state-machine)
   - [Guidance](#guidance)
   - [Navigation & Control](#navigation--control)
4. [Monte Carlo Analysis](#monte-carlo-analysis)
5. [How to Run](#how-to-run)


## Overview & Architecture
The goal of the project is to simulate a symmetrical, aerodynamically instable rocket up until stage seperation. It is architected as a co-simulation environment that seperates the vehicles physics from its control logic. Communication between Simulink and the Flight Software is handled via UDP.

1.  **Plant Model (Simulink):** 6-DOF simulation that models and calculates all major forces and moments acting on the rocket, including engine thrust, gravity, and a realistic aerodynamics model. It serves as the physics engine and is also responsible for visualization using Simulink 3D Animation.

2.  **Flight Software (C++):** The standalone C++ application runs the entire GNC system. It processes incoming sensor data from the Simulink model via UDP, guides and steers the rocket using PID controllers and a predefined flight config.

Together they create a Software-in-the-Loop environment that allows rapid development and testing of both components.

The control objective is to guide the vehicle through a **gravity turn**. The GNC system is responsible for full **attitude control** to maintain stability, especially during Max Q, while actively compensating for variable disturbances like wind.

The simulation was verified using Monte-Carlo-Analysis to simulate 100 runs with different variables, proving that the flight software is able to adapt to different situations.

## Simulation Model (Simulink)

#6-DOF Physics Engine
