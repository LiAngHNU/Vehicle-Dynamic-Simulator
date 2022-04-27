# Vehicle-Dynamic-Simulator
A vehicle dynamic simulator based on Matlab Simulink and CarSim/TruckSim.

# Module CarSim/TruckSim
This Module is used to:
  Manage I/O between CarSim/TruckSim and Matlab Simulink
  Unit Unification
  Input Processing
  ...
## IO Definition
### Input
| Input Type | Name | Note |
|---|---|---|
|Displacements & Angles|||
|Velocity & Angular Velocity|||
|Force & Torque|||
#### Thorttle Command
#### Brake Command
#### Steer Command

### Output
| Input Type | Name | Note | Unit |
|---|---|---|---|
| Vehicle-Path Informations | x_path |  | [m] -> [m] |
|                           | y_path || [m] -> [m] |
|                           | z_path || [m] -> [m] |
| | bank_path |||
| | slope_path |||
| | theta_path |||
| | kappa_path |||

| Vehicle Body Kinematics & Dynamics | x_veh_ref | X coordinate of sprung mass | [m] -> [m] |
| | s_veh_1 | ||
| | s_veh_2 | ||
| | s_veh_3 | ||
| | s_veh_4 | ||
| | s_veh_5 | ||
| | l_veh_1 | ||
| | l_veh_2 | ||
| | l_veh_3 | ||
| | l_veh_4 | ||
| | l_veh_5 | ||
|                                    | x_veh_ref | X coordinate of sprung mass | [m] -> [m] |
|                                    | y_veh_ref | Y coordinate of sprung mass | [m] -> [m] |
|                                    | z_veh_ref | Z coordinate of sprung mass | [m] -> [m] |
|                                    | r_veh_ref  | Rotate angle around X-axis of sprung mass | [deg] -> [rad] |
|                                    | p_veh_ref | Rotate angle around Y-axis of sprung mass | [deg] -> [rad] |
|                                    | y_veh_ref   | Rotate angle around Z-axis of sprung mass | [deg] -> [rad] |
|                                    | vx_veh_ref | Linear velocity along X-axis of sprung mass | [km/h] -> [m/s] |
|                                    | vy_veh_ref | Linear velocity along Y-axis of sprung mass | [km/h] -> [m/s] |
|                                    | vz_veh_ref | Linear velocity along Z-axis of sprung mass | [km/h] -> [m/s] |
|                                    | avx_veh_ref | Angular velocity around X-axis of sprung mass| [deg/s] -> [rad/s] |
|                                    | avy_veh_ref | Angular velocity around Y-axis of sprung mass| [deg/s] -> [rad/s] |
|                                    | avz_veh_ref | Angular velocity around Z-axis of sprung mass| [deg/s] -> [rad/s] |
|                                    | ax_veh_ref | Linear acceleration along X-axis of sprung mass | [g's] -> [m/s2] |
|                                    | ay_veh_ref | Linear acceleration along Y-axis of sprung mass | [g's] -> [m/s2] |
|                                    | az_veh_ref | Linear acceleration along Z-axis of sprung mass | [g's] -> [m/s2] |
|                                    | aax_veh_ref | Angular acceleration around X-axis of sprung mass| [rad/s2] -> [rad/s2] |
|                                    | aay_veh_ref | Angular acceleration around Y-axis of sprung mass| [rad/s2] -> [rad/s2] |
|                                    | aaz_veh_ref | Angular acceleration around Z-axis of sprung mass| [rad/s2] -> [rad/s2] |
| Vehicle Susp Kinematics & Dynamics |||
| Vehicle Tyre Kinematics & Dynamics |||

# Module Localization
This Module is used to:
  Load Map
  Calculate Errors between Vehicle and Road 
  Update Vehicle Attitude
  ...

# Module Chassis
This Module is used to:
  Update Vehicle Reference State
  ...

# Module Planning (Undeveloped)
This Module is used to:
  Vehicle Motion Planning
  ...
  
# Module Control
This Module is used to:
  Implement and Test Control Algorthms
  ...
