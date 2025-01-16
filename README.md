# Advanced Control Labs

This repository contains two lab assignments for the Advanced Control course. Each lab folder includes the problem sheet, the submitted report, and a scrapped robust control approach. The code for these experiments was developed on a different machine.

## Table of Contents

- [Lab 1: MagLev Control](#lab-1-maglev-control)
  - [Description](#description)
- [Lab 2: Pendubot Control](#lab-2-pendubot-control)
  - [Description](#description)

## Lab 1: MagLev Control

### Description

This lab focuses on controlling a Magnetic Levitation (MagLev) system using discrete-time controllers, specifically the PID and IFT PID methods. The lab folder includes the problem sheet, the report, and a scrapped robust control approach.

In this lab, three different controllers were designed to control the MagLev system. 

1. **PID Controller**  
   The first controller developed was a traditional PID controller. It was tuned using both manual and heuristic techniques like Ziegler-Nichols. This controller aims to minimize the error quickly when the system is asked to stabilize at a given setpoint within the required range.

2. **Switched Integral PID Controller**  
   The second controller is based on a switched integral PID design, where multiple PID controllers are used to address different control regimes based on the system's behavior. The approach is intended to enhance the setpoint range that the system can handle, allowing for more flexible control as the setpoints change. This controller combines the robustness of a PID controller with the ability to switch between different controller modes depending on the system's state.

3. **IFT PID Control (Iterative Feedback Tuning PID)**  
   The third controller is based on Iterative Feedback Tuning (IFT) PID, a more advanced control technique. In this method, the controller parameters are optimized iteratively by minimizing a cost function that penalizes both tracking errors and control efforts. The cost function, as proposed by Hjalmarsson et al. (1994), is designed to tune the PID parameters to achieve optimal performance based on real system feedback, without requiring a model of the system. The iterative approach adjusts the controller parameters until the desired performance is achieved, using a stochastic approximation method to converge to an optimal solution.

   The cost function is defined as:

   $
   J(\rho) = \frac{1}{2N} + \left[\sum L_y \tilde{y}_t(\rho)^2 + \lambda \sum L_u u_t(\rho)^2\right]
   $

   where:
   - $\rho$ is the vector of controller parameters to be optimized.
   - $\tilde{y}_t(\rho)$ is the error between the actual output of the system and the desired output.
   - $u_t(\rho)$ is the control signal.
   - $L_y$ and $L_u$ are frequency weighting filters applied to the tracking error and the control signal.
   - $\lambda$ is a weighting factor that balances the importance of minimizing control efforts against tracking accuracy.
   - $N$ is the number of data points used in the optimization process.

   This method allows for an unbiased estimate of the gradient of the cost function, based purely on signal information and experimental feedback. The controller parameters are updated iteratively using the gradient and a stochastic approximation algorithm until an optimal set of parameters is found.

## Lab 2: Pendubot Control

### Description

This lab focuses on controlling a Pendubot using both linear and nonlinear controllers. The lab folder includes the problem sheet, the submitted report, and the comprehensive details on the system's behavior and the implemented control techniques.
