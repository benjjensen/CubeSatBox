## Performance Evaluation 
This folder contains the code used to evaluate to test the CubeSat in the HITL textbox. 

### Diode Calibration + Sun Vector Estimation
First, the testbox was used to simulate the sun vector, and the satellite measurements were used to 
estimate the sun vector.
This data was stored and recorded, and then used to calibrate the photodiodes with linear-least-squares 
(some initial attempts at using the MEKF to solve recursively were started, but unfinished).
The calibrated diodes were then used to estimate the sun vector again.

Primary scripts used are `simulate_sun.jl` and `simulate_sun.py` to gather the data (along with 
`simulate_sun.ino`, which is essentially the same as microcontroller.ino in LightDemo), and 
`calibrate_diodes.jl` for both calibrating diodes and evaluating the uncalibrated/calibrated sun 
estimation system. 

Additional scripts include `evaluate_real_time.jl`, which allows for a rough sun estimate to be generated 
in real time (helpful for debugging), and ...


### Detumbling
Next, the system was used to simulate an orbit and generate desired outputs for the magnetorquer coils
on the CubeSat, and read in the resulting torque through the testbox to feed back into the system.

This currently does NOT work. Primary challenges include (1) Sending information to the CubeSat from a laptop 
and (2) converting from a desired torque to the current in the magnetorquers, as well as from the magnetometer 
output back into the system