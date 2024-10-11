# Introduction
This is the source template for the project of the state estimation module in AIS2202 Cybernetics.

The task is to implement the (1) parameter estimation and (2) state estimation techniques from the references listed below. 

## Parameter estimation
For parameter estimation of the sensor and end-effector tool, the following must be estimated by using the dataset and implementing the techniques from paper 1:
1. Sensor f_bias.
2. Mass of the tool.
3. Mass center of the tool.

## State estimation
For the state estimation, the contact wrench and gravity compensated force, torque and acceleration vectors for the three experiments must be estimated by using the dataset and implementing the method from paper 2.

The states of the system are defined to be the force, torque and acceleration of the end effector. 
The dataset contains orientation, FTS and IMU measurements from three different experiments with robot motion:<br>
1. Without external factors.
2. With mechanical vibrations.
3. With mechanical vibrations and contact with the environment.

# References

Dataset:<br>
[Accelerometer and Force/Torque Sensor Measurements for Parameter and State Estimation of an Unknown Robot End Effector](https://zenodo.org/records/11096791)

Paper: Parameter estimation<br>
[Bias Estimation and Gravity Compensation For Force-Torque Sensors](https://citeseerx.ist.psu.edu/document?doi=900c5de4ac54cf28df816584264fa0de71c4817f)

Paper: State estimation<br>
[A Linear Discrete Kalman Filter to Estimate the Contact Wrench of an Unknown Robot End Effector](https://ieeexplore.ieee.org/document/10671273)

# How to setup vcpkg (in manifest mode)

Call CMake with `-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake`

Add optional features by listing them with `-DVCPKG_MANIFEST_FEATURES=feature1;feature2`

Note, however, that under MinGW you'll need to specify the vcpkg triplet:
```bash
-DVCPKG_TARGET_TRIPLET=x64-mingw-[static|dynamic]  # For both of these lines, choose either 'static' or 'dynamic'.
-DVCPKG_HOST_TRIPLET=x64-mingw-[static|dynamic]    # This is needed only if MSVC cannot be found. 
```
Source: [threepp](https://github.com/markaren/threepp/blob/master/vcpkg.json).
