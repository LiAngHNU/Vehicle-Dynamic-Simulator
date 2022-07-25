# Vehicle-Dynamic-Simulator
A vehicle dynamic simulator based on Matlab Simulink and CarSim/TruckSim.

# Module： Sim_Module
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
#### Wind Angle
#### Wind Speed
#### Thorttle Command
#### Brake Command
#### Steer Command

### Output
| Input Type | Name | Note | Unit |
|---|---|---|---|
| Obstacle Informations | | | |
| Path Info | x_path || [m] -> [m] |
|           | y_path || [m] -> [m] |
|           | z_path*|| [m] -> [m] |
|           | Rx_path || [deg] -> [rad] |
|           | Py_path || [deg] -> [rad] |
|           | Yz_path*|| [deg] -> [rad] |
| | kappa_path || [1/m] -> [1/m] |
| | MuX_path_1l || [N/A] -> [N/A] |
| | MuX_path_1r || [N/A] -> [N/A] |
| | MuX_path_2l || [N/A] -> [N/A] |
| | MuX_path_2r || [N/A] -> [N/A] |
| | MuX_path_3l || [N/A] -> [N/A] |
| | MuX_path_3r || [N/A] -> [N/A] |
| | MuX_path_4l || [N/A] -> [N/A] |
| | MuX_path_4r || [N/A] -> [N/A] |
| | MuY_path_1l || [N/A] -> [N/A] |
| | MuY_path_1r || [N/A] -> [N/A] |
| | MuY_path_2l || [N/A] -> [N/A] |
| | MuY_path_2r || [N/A] -> [N/A] |
| | MuY_path_3l || [N/A] -> [N/A] |
| | MuY_path_3r || [N/A] -> [N/A] |
| | MuY_path_4l || [N/A] -> [N/A] |
| | MuY_path_4r || [N/A] -> [N/A] |
|                                    |            |||
| Vehicle Body Kinematics & Dynamics | s_veh | Station of front bumper at reference path | [m] -> [m] |
|                                    | l_veh_1 | Lateral distance of front bumper at reference path  | [m] -> [m] |
|                                    | l_veh_2 | Lateral distance of front axle at reference path    | [m] -> [m] |
|                                    | l_veh_3 | Lateral distance of CoG at reference path           | [m] -> [m] |
|                                    | l_veh_4 | Lateral distance of rear axle at reference path     | [m] -> [m] |
|                                    | l_veh_5 | Lateral distance of rear bumper at reference path   | [m] -> [m] |
|                                    | a_veh_1 | Lateral error area of front bumper at reference path  | [m2] -> [m2] |
|                                    | a_veh_2 | Lateral error area of front axle at reference path    | [m2] -> [m2] |
|                                    | a_veh_3 | Lateral error area of CoG at reference path           | [m2] -> [m2] |
|                                    | a_veh_4 | Lateral error area of rear axle at reference path     | [m2] -> [m2] |
|                                    | a_veh_5 | Lateral error area of rear bumper at reference path   | [m2] -> [m2] |
|                                    | x_veh_ref | X coordinate of sprung mass | [m] -> [m] |
|                                    | y_veh_ref | Y coordinate of sprung mass | [m] -> [m] |
|                                    | z_veh_ref | Z coordinate of sprung mass | [m] -> [m] |
|                                    | delta_sw  | Rotate angle around X-axis of sprung mass | [deg] -> [rad] |
|                                    | r_veh_ref | Rotate angle around X-axis of sprung mass | [deg] -> [rad] |
|                                    | p_veh_ref | Rotate angle around Y-axis of sprung mass | [deg] -> [rad] |
|                                    | y_veh_ref  | Rotate angle around Z-axis of sprung mass | [deg] -> [rad] |
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
|                                    |            |||
| Vehicle Susp Kinematics & Dynamics | jnc_whl_1l || [mm] -> [mm] |
|                                    | jnc_whl_1r || [mm] -> [mm] |
|                                    | jnc_whl_2l || [mm] -> [mm] |
|                                    | jnc_whl_2r || [mm] -> [mm] |
|                                    | jnc_whl_3l || [mm] -> [mm] |
|                                    | jnc_whl_3r || [mm] -> [mm] |
|                                    | jnc_whl_4l || [mm] -> [mm] |
|                                    | jnc_whl_4r || [mm] -> [mm] |
|                                    |            |||
|                                    | cmp_spring_1l || [mm] -> [mm] |
|                                    | cmp_spring_1r || [mm] -> [mm] |
|                                    | cmp_spring_2l || [mm] -> [mm] |
|                                    | cmp_spring_2r || [mm] -> [mm] |
|                                    | cmp_spring_3l || [mm] -> [mm] |
|                                    | cmp_spring_3r || [mm] -> [mm] |
|                                    | cmp_spring_4l || [mm] -> [mm] |
|                                    | cmp_spring_4r || [mm] -> [mm] |
|                                    |            |||
|                                    | cmp_damper_1l || [mm] -> [mm] |
|                                    | cmp_damper_1r || [mm] -> [mm] |
|                                    | cmp_damper_2l || [mm] -> [mm] |
|                                    | cmp_damper_2r || [mm] -> [mm] |
|                                    | cmp_damper_3l || [mm] -> [mm] |
|                                    | cmp_damper_3r || [mm] -> [mm] |
|                                    | cmp_damper_4l || [mm] -> [mm] |
|                                    | cmp_damper_4r || [mm] -> [mm] |
|                                    |            |||
|                                    | jnc_rate_whl_1l || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_1r || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_2l || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_2r || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_3l || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_3r || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_4l || [mm/s] -> [mm/s] |
|                                    | jnc_rate_whl_4r || [mm/s] -> [mm/s] |
|                                    |            |||
|                                    | cmp_rate_damper_1l || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_1r || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_2l || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_2r || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_3l || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_3r || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_4l || [mm/s] -> [mm/s] |
|                                    | cmp_rate_damper_4r || [mm/s] -> [mm/s] |
|                                    |            |||
|                                    | cmp_acc_damper_1l || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_1r || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_2l || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_2r || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_3l || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_3r || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_4l || [mm/s2] -> [mm/s2] |
|                                    | cmp_acc_damper_4r || [mm/s2] -> [mm/s2] |
|                                    |            |||
|                                    | f_spring_1l || [N] -> [N] |
|                                    | f_spring_1r || [N] -> [N] |
|                                    | f_spring_2l || [N] -> [N] |
|                                    | f_spring_2r || [N] -> [N] |
|                                    | f_spring_3l || [N] -> [N] |
|                                    | f_spring_3r || [N] -> [N] |
|                                    | f_spring_4l || [N] -> [N] |
|                                    | f_spring_4r || [N] -> [N] |
|                                    |            |||
|                                    | f_damper_1l || [N] -> [N] |
|                                    | f_damper_1r || [N] -> [N] |
|                                    | f_damper_2l || [N] -> [N] |
|                                    | f_damper_2r || [N] -> [N] |
|                                    | f_damper_3l || [N] -> [N] |
|                                    | f_damper_3r || [N] -> [N] |
|                                    | f_damper_4l || [N] -> [N] |
|                                    | f_damper_4r || [N] -> [N] |
|                                    |            |||
|                                    | m_arb_1 || [N/m] -> [N/m] |
|                                    | m_arb_2 || [N/m] -> [N/m] |
|                                    | m_arb_3 || [N/m] -> [N/m] |
|                                    | m_arb_4 || [N/m] -> [N/m] |
|                                    |            |||
| Vehicle Tyre Kinematics & Dynamics | kappa_whl_1l || [-] -> [-] |
|                                    | kappa_whl_1r || [-] -> [-] |
|                                    | kappa_whl_2l || [-] -> [-] |
|                                    | kappa_whl_2r || [-] -> [-] |
|                                    | kappa_whl_3l || [-] -> [-] |
|                                    | kappa_whl_3r || [-] -> [-] |
|                                    | kappa_whl_4l || [-] -> [-] |
|                                    | kappa_whl_4r || [-] -> [-] |
|                                    | alpha_whl_1l || [deg] -> [rad] |
|                                    | alpha_whl_1r || [deg] -> [rad] |
|                                    | alpha_whl_2l || [deg] -> [rad] |
|                                    | alpha_whl_2r || [deg] -> [rad] |
|                                    | alpha_whl_3l || [deg] -> [rad] |
|                                    | alpha_whl_3r || [deg] -> [rad] |
|                                    | alpha_whl_4l || [deg] -> [rad] |
|                                    | alpha_whl_4r || [deg] -> [rad] |
|                                    | delta_whl_1l || [deg] -> [rad] |
|                                    | delta_whl_1r || [deg] -> [rad] |
|                                    | delta_whl_2l || [deg] -> [rad] |
|                                    | delta_whl_2r || [deg] -> [rad] |
|                                    | delta_whl_3l || [deg] -> [rad] |
|                                    | delta_whl_3r || [deg] -> [rad] |
|                                    | delta_whl_4l || [deg] -> [rad] |
|                                    | delta_whl_4r || [deg] -> [rad] |

# Module Localization
This Module is used to:
  Load Map
  Calculate Errors between Vehicle and Road 
  Update Vehicle Attitude
  ...
## Maps
### MCity
#### Outside Loop
#### Inside Loop
#### Highway Loop
#### M Loop
### Suburban Town
### HandlingCourse

# Module Estimation
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
