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
| Vehicle-Path | x_path |  | |
| | y_path |||
| | z_path |||
| | bank_path |||
| | slope_path |||
| | theta_path |||
| | s_veh | ||
| | l_veh | ||
| Vehicle Body Kinematics & Dynamics | x_veh_ref | X coordinate of sprung mass | [m] -> [m] |
|                                    | y_veh_ref | Y coordinate of sprung mass | [m] -> [m] |
|                                    | z_veh_ref | Z coordinate of sprung mass | [m] -> [m] |
|                                    | roll_veh_ref  | Roll angle of sprung mass  | [deg] -> [rad] |
|                                    | pitch_veh_ref | Picth angle of sprung mass | [deg] -> [rad] |
|                                    | yaw_veh_ref   | Yaw angle of sprung mass   | [deg] -> [rad] |
|                                    | vx_veh_ref | Longitudinal linear velocity of sprung mass | [km/h] -> [m/s] |
|                                    | vy_veh_ref | Lateral linear velocity of sprung mass | [km/h] -> [m/s] |
|                                    | vz_veh_ref | Vertical linear velocity of sprung mass | [km/h] -> [m/s] |
| | avx_veh_ref | |
| | avy_veh_ref | |
| | avz_veh_ref | |
| | ax_veh_ref | |
| | ay_veh_ref | |
| | az_veh_ref | |
| | aax_veh_ref | |
| | aay_veh_ref | |
| | aaz_veh_ref | |
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
