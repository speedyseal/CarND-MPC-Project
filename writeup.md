# MPC project

## Model

The model consists of a 6 dimensional state vector comprising x position, y position, vehicle heading psi, velocity.
The acuator outputs are steering angle and throttle. The criteria to minimize are cross track error and orientation error eps.

The model is implemented in the solver constraints on lines 128-136 of MPC.cpp

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);

Because the simulator flips the steering output delta0, the updates to psi have flipped signs. Otherwise the update equations model the kinematics of the system given position, heading, and throttle and steering inputs.

## Timestep length and step size

The timestep length was chosen to be short to minimize computational latency but long enough so that it captures the curvature of the path, which is needed to solve for the steering angle.

The step size was kept small at 50 ms so that the vehicle does not deviate far from the path between each optimized point. 50 ms at 40 mph is 2.9 feet. This constrains the prediction errors propagated from errors in the initial state.

## Polynomial fitting and MPC preprocessing
The waypoints and vehicle state are preprocessed to be relative to the vehicles current position. They are transformed - translated and rotated to be centered on the vehicles position and heading. This allows the waypoints to be fit to a simple polynomial passing through (0,0), and allows the CTE to be computed more simply. The MPC optimization can then be done relative to this reference point.

## Latency compensation
Controller latency is compensated by predicting the vehicle state across the latency period based on current state. Because acceleration is not provided by the vehicle, a simplification is made to assume velocity is constant over the delay. The velocity and last steering angle is used to update the heading, which is then used to update x and y. The waypoints and CTE are then computed relative to this predicted position. This prediction allows the predicted actuator outputs to be more relevant and helps to stabilize the controller.