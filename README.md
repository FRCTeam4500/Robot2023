# Hello
Nice to see you! 

# Important things to note
This code is built for the 2023 game: Charged Up, and thus, the code is meant to be built on the 2023 Release of WPILib and GradleRIO.

# Buttons

 
# Physical Explanations

Arm
Intake

Camera/Limelight

Swerve

Balancer -> Gyro

# Program Explanations

## Constants

Since the arm is meant to go to 5 places and be synchronized with the intake, the raw values need to be observed from future tests.

- Ground: The intake has wheels, slam the arm into the ground(gently)
- Bottom: Designed to be the ideal height for scoring in the bottom row
- Middle: For the middle row
- Top: For the top :/
- Retracted, Stay in robot frame

Executive decision made: The Intake+Arm Combo is now called **The Placer**