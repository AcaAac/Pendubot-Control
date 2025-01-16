# Advanced Control Labs

This repository contains two lab assignments for the Advanced Control course. Each lab folder includes the problem sheet, the submitted code, and the report.

## Table of Contents

- [Lab 1: MagLev Control](#lab-1-maglev-control)
  - [Description](#description)
- [Lab 2: Pendubot Control](#lab-2-pendubot-control)
  - [Description](#description)

## Lab 1: MagLev Control

### Description

This lab focuses on controlling a Magnetic Levitation (MagLev) system using P, PI, PID, and IFT PID controllers. The lab folder contains the problem sheet, the data taken from the lab computers, and a robust control approach.

In this lab, we were tasked with designing three different controllers for the MagLev system. All of the controllers were discrete-time controllers.

1. **PID Controller**  
   The first controller designed was a PID Controller, tuned using both manual and heuristic (Ziegler-Nichols) tuning techniques.  
   - **Quantitative Objective:** The controller was considered reasonably good if it satisfied the following criterion: cancel the error as quickly as possible (in a few seconds) over a setpoint range of at least 1 V.

2. **Switched Integral PID Controller**  
   The second controller was a switched integral PID controller.  
   - **Goal:** The goal was to design a controller, based on one or several linear PID controllers, to enhance the range of setpoints reachable with the ball (MATLAB and Simulink enabled the implementation).  
   - **Quantitative Objective:** The controller was considered good if it allowed the ball to cover the entire range of the position sensor with a triangle wave as a setpoint. Additionally, we investigated the robustness of the controller.

3. **IFT PID Control (Iterative Feedback Tuning PID)**  
   The third controller was an advanced controller: the Iterative Feedback Tuning PID (IFT PID) technique.  
   - **Cost Function (Hjalmarsson et al., 1994):**
     \[
     J(\rho) = \frac{1}{2N} + \left[\sum L_y \tilde{y}_t(\rho)^2 + \lambda \sum L_u u_t(\rho)^2\right]
     \]
     where:
     - \(\rho\) is the vector of controller parameters to be optimized.
     - \(\tilde{y}_t(\rho)\) is the error between the output \(y_t(\rho)\) of the actual system controlled by the controller \(C(\rho) = [Cr(\rho), Cy(\rho)]\) and a desired output signal \(y_d\).
     - \(u_t(\rho)\) is the control signal.
     - \(L_y\) and \(L_u\) are frequency weighting filters.
     - \(\lambda\) expresses the relative importance of the penalty on the control signal versus the tracking error.
     - \(N\) is the number of data points.
     - \(E\) stands for the expected value.

   The main contribution of this method was to show that one could compute an unbiased estimate of the gradient of the cost function without knowledge of the system, using only signal information and applying a special “feedback” experiment to the actual system. A local minimum of the cost function was reached by iterative computations of the gradient and the use of a stochastic approximation algorithm to update the controller parameter vector:
   \[
   \rho_{i+1} = \rho_i - \gamma_i R^{-1}_i \frac{\partial J}{\partial \rho} (\rho_i)
   \]

## Lab 2: Pendubot Control

### Description

This lab focuses on controlling a Pendubot using both linear and nonlinear controllers. The lab folder contains the problem sheet, the submitted code, and the comprehensive report.
