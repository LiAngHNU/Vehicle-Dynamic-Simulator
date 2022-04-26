Welcome to 

# Introduction
A vehicle dynamic simulator based on Matlab Simulink and CarSim/TruckSim.

# Modules Introduction
## Module CarSim/TruckSim
This Module is used to:
  Manage I/O between CarSim/TruckSim and Matlab Simulink
  Unit Unification
  Input Processing
  ...
### IO Definition
#### Input
| Input Type | Name | Note |
|---|---|---|
|Displacements & Angles|||
|Velocity & Angular Velocity|||
|Force & Torque|||

#### Output
| Input Type | Name | Note |
|---|---|---|
| Road Information |||
| Vehicle Body Kinematics & Dynamics |||
| Vehicle Susp Kinematics & Dynamics |||
| Vehicle Tyre Kinematics & Dynamics |||

## Module Localization
This Module is used to:
  Load Map
  Calculate Errors between Vehicle and Road 
  Update Vehicle Attitude
  ...

## Module Chassis
This Module is used to:
  Update Vehicle Reference State
  ...

## Module Planning (Undeveloped)
This Module is used to:
  Vehicle Motion Planning
  ...
  
## Module Control
This Module is used to:
  Implement and Test Control Algorthms
  ...
