Self-Balancing Robot
##Overview

A self-balancing two-wheeled robot that maintains an upright position using real-time tilt feedback. Built using an ESP32 microcontroller, an MPU6050 IMU, and an L298N motor driver, with a single-loop PID controller correcting tilt angle by driving the wheels.

##Features
Real-time tilt sensing and correction
ESP32 microcontroller
MPU6050 IMU with onboard DMP for orientation
PID-based single-loop balance control
L298N dual motor driver
Fall-detection safety cutoff
Companion theoretical model in Simulink
##Hardware
ESP32 Development Board
MPU6050 IMU (accelerometer + gyroscope)
L298N Motor Driver
2x DC Gear Motors
2S 18650 Li-ion Battery Pack (7.4V nominal)
9V Battery
Custom Foam-Core Board Chassis
Wiring and Mounting Hardware
Software
Arduino Framework (ESP32)
C++
PID_v1_bc Library
MPU6050 (DMP-based orientation)
##How it Works
Read pitch angle from the MPU6050's onboard DMP.
Compare pitch against the calibrated balance setpoint.
Run the error through a PID controller to compute a motor correction output.
Drive both motors in the corrective direction and magnitude.
If tilt exceeds a safety threshold, cut motor power to prevent damage.
Repeat continuously in a tight control loop.
##Design Decisions
A single tilt-only control loop was used instead of a cascaded position/tilt loop, to keep the scope achievable within the project timeline while still demonstrating core PID control concepts.
The MPU6050's onboard DMP was used for sensor fusion instead of a manual complementary or Kalman filter, to offload orientation computation from the ESP32.
The ESP32 and IMU are powered from the L298N's onboard 5V regulator rather than a separate supply, to simplify wiring at the cost of some power-rail noise sensitivity.
A fall-detection cutoff disables the motors outside a safe tilt range, protecting the hardware during testing and tuning.
Future Improvements
Motor current sensing for quantitative performance data
Battery voltage monitoring
Encoder feedback and an outer position control loop
Wi-Fi telemetry and remote monitoring
Dedicated regulated power rail for the ESP32/IMU, separate from the motor driver

##Author

Ryan Brooke Simon Fraser University Mechatronics Systems Engineering
