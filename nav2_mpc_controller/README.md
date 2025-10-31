# nav2_mpc_controller

The `nav2_mpc_controller` package implements a sampling-based Model Predictive Control (MPC)
local planner for the Nav2 controller server. The plugin evaluates a grid of constant control
inputs over a finite prediction horizon and selects the command with the lowest accumulated cost.
The cost function combines path-tracking error, orientation error, commanded velocity regularity
and the underlying costmap value of the predicted states.

## Parameters

All parameters are declared under the configured controller ID. The defaults match the values used
in `nav2_bringup`.

| Parameter | Description |
|-----------|-------------|
| `prediction_horizon` | Number of discrete steps evaluated in each rollout. |
| `time_step` | Duration in seconds of each step in the prediction horizon. |
| `max_linear_velocity` | Maximum forward speed allowed when sampling commands (m/s). |
| `max_angular_velocity` | Maximum angular speed allowed when sampling commands (rad/s). |
| `path_lookahead_distance` | Forward distance along the global plan used to build the reference set. |
| `goal_tolerance` | Distance threshold used to return a zero command when the goal is reached. |
| `obstacle_cost_weight` | Weight applied to the raw costmap value during optimisation. |
| `orientation_cost_weight` | Weight applied to heading error relative to the reference path. |
| `velocity_cost_weight` | Weight applied to deviation from the maximum linear velocity. |
| `linear_samples` | Number of linear velocity samples taken across the feasible range. |
| `angular_samples` | Number of angular velocity samples taken across the feasible range. |
| `allow_reverse_motion` | Enables symmetric sampling around zero to allow backward motion. |

The controller honours dynamic speed limit updates provided through the standard Nav2 speed limit
interface.

## Plugin registration

The plugin is registered as `nav2_mpc_controller::MPCController` and exported through
`mpc_controller.xml`. Add the controller ID and plugin type to the `controller_plugins` array and
parameter map in the controller server configuration to enable it.
