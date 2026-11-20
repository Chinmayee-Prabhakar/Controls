# Robot Controls — Feedback, Robust & Adaptive Control

A series of MATLAB and ROS/Gazebo projects on manipulator dynamics, stabilization, trajectory tracking, and uncertain-parameter control. 

## Controller gallery

| Project | Method | Source code | Original report |
| --- | --- | --- | --- |
| [1 — RRBot passive dynamics](Project%201/) | Two-joint dynamics and passive response | [`main.m`](Project%201/main.m) | [Simulation screenshots](Project%201/) |
| [2 — State feedback](Project%202/) | Joint effort from measured state feedback in ROS/Gazebo | [`rrbot_control.m`](Project%202/rrbot_control.m) | [PDF](Project%202/Prabhakar_ProgAssignment2_Report.pdf) |
| [3 — Feedback linearization](Project%203/) | Model-based trajectory tracking and pole-placement design | [`feedback_linearization.m`](Project%203/feedback_linearization.m), [`main.m`](Project%203/main.m) | [PDF](Project%203/Prabhakar_ProgAssignment3_Report.pdf) |
| [4 — Robust tracking](Project%204/) | Model-based robust control and Lyapunov analysis | [`robust_control.m`](Project%204/robust_control.m), [`main.m`](Project%204/main.m) | [PDF](Project%204/Prabhakar_ProgAssignment4_report.pdf) |
| [5 — Adaptive tracking](Project%205/) | Online estimation of uncertain dynamic parameters | [`adaptive_control.m`](Project%205/adaptive_control.m), [`main.m`](Project%205/main.m) | [PDF](Project%205/Prabhakar_ProgAssignment5_report.pdf) |

### RRBot simulation

![Robot simulation](Project%201/Screenshot%20from%202023-01-24%2022-23-05.png)

## Control architecture

```mermaid
flowchart LR
 R[Joint reference] --> C[Feedback controller]
 X[Measured joint states] --> C
 C --> U[Joint torques]
 U --> P[Robot dynamics / simulation]
 P --> X
```

The later modules model manipulator dynamics in the form `M(q)q̈ + C(q,q̇)q̇ + G(q) = τ`. Project 2 uses a fixed state-feedback gain in ROS; Project 4 computes a Lyapunov matrix and simulates desired versus actual joint trajectories; Project 5 additionally plots estimated model parameters `alpha_hat` over time.

## How to inspect or reproduce

- For Projects 3–5, open MATLAB with the corresponding `Project N` as the working folder and run `main.m`. MATLAB's Symbolic Math and Control System toolboxes may be required for associated design scripts.
- Project 2 additionally needs ROS 1, a matching Gazebo RRBot simulation, MATLAB ROS connectivity, and the effort topics named in `rrbot_control.m`.
- The original PDF reports give figures and the experimental assumptions for each assignment.


