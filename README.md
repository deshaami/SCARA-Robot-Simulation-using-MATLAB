
# SCARA Robot Simulation using MATLAB

This project simulates a SCARA (Selective Compliance Assembly Robot Arm) robot with 3 degrees of freedom (DOF) using MATLAB. The simulation models the kinematics and motion of the robot arm in a 2D workspace.

---

## Project Overview

The SCARA robot is widely used in assembly lines and pick-and-place operations due to its high precision and speed. This simulation focuses on:

- Modeling the robot’s kinematics with 3 rotational joints  
- Visualizing the robot arm movement in MATLAB  
- Calculating the forward kinematics to find the position of the end effector  

---

## Features

- 3 DOF SCARA robot model  
- Forward kinematics calculation  
- Graphical visualization of robot arm movement  
- User input for joint angles to simulate different poses  

---

## Tools Used

- MATLAB R2023a or later  
- Robotics Toolbox (optional)  
- Basic MATLAB plotting functions  

---

## How to Run

1. Open MATLAB and load the `scara_simulation.m` file.  
2. Run the script.  
3. Enter the joint angles (in degrees) and vertical displacement (in cm) when prompted.  
4. Observe the SCARA robot movement in the plot window and check the printed end effector position.  

---

## Code (scara_simulation.m)

```matlab
% SCARA Robot Simulation with 3 DOF

% Link lengths (in cm)
L1 = 10;
L2 = 10;
L3 = 5;  % vertical prismatic joint length (fixed here for simplicity)

% Get joint angles from user (in degrees)
theta1 = input('Enter joint angle theta1 (degrees): ');
theta2 = input('Enter joint angle theta2 (degrees): ');
theta3 = input('Enter vertical displacement (cm): ');

% Convert degrees to radians
t1 = deg2rad(theta1);
t2 = deg2rad(theta2);

% Forward Kinematics
% Calculate (x, y) position of each joint
x1 = L1*cos(t1);
y1 = L1*sin(t1);

x2 = x1 + L2*cos(t1 + t2);
y2 = y1 + L2*sin(t1 + t2);

z3 = -theta3; % vertical displacement of the end effector

% Plot the robot arm
figure;
plot([0 x1], [0 y1], 'r', 'LineWidth', 3); hold on;
plot([x1 x2], [y1 y2], 'g', 'LineWidth', 3);
plot(x2, y2, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
title('SCARA Robot Simulation');
xlabel('X-axis (cm)');
ylabel('Y-axis (cm)');
grid on;
axis equal;
xlim([-25 25]);
ylim([-25 25]);

% Display end effector position
fprintf('End Effector Position: X = %.2f cm, Y = %.2f cm, Z = %.2f cm\n', x2, y2, z3);
