# ROBOTNAME-9127-2024
Official command-based code for the 9127 Robot: [ROBOTNAME].

This branch will contain the code that the team should use.

# Autonomous Mode
Using SmartDashboard or ShuffleBoard, the operator can access a sendable chooser, which will allow them to select which autonomous routine to play.

The Shuffleboard layout can be found in the file called "ShuffleBoardLayout.json"

# **Teleop Mode XBOX Controls**:
(Any of the following is subject to change)

## **Driver Controls**:
**Emergency stop:** X-button

**Tank drive:** Left/Right Joysticks

**Intake speed:** Left/Right Triggers

### Arm Positions:
**Jump between positions using left/right bumpers (left makes it go towards intake, right makes it go towards shooting)**

1. Intaking position (around 0°)
2. Inside perimeters of robot, close to intake (around 40°)
3. Down-shooting into Amp (around ?°)
4. Inside perimeters of robot, close to down-shooting into Amp (around 90°)
5. Intake from source (around ?°)

## Operator Controls
**Emergency stop:** X-button

**Arm Manual Movement:** Right Joystick *(e-stop button on either controllers must be hit prior to using this or it won't work)*

**Arm Setpoint Offset:** Left/Right Bumpers *(offsets the arm's position by increments of 3°, will automatically be reset to 0 when each limit switches are hit)*

**Intake Speed:** Left/Right Triggers

# **How To Set Up Robot**:
*TBD*

# Firmware Updating
Using Phoenix Tuner, the user can update the firmware of the connected devices (motor controllers and PDP). The firmware files are located in a folder called "ctr-device-firmware" in this project.
1. **TalonSRX (Arm):** Version 22.1
2. **VictorSPX (Drivetrain):** Version 22.1
3. **PDP:** Version 1.40

If the installed version is outdated, the frame around the device in Phoenix Tuner will show up as yellow instead of green.

If the installed version is incorrect, the frame around the device in Phoenix Tuner will show up as purple instead of green.

# **SimGUI Mapping** *(Software Simulation)*:
## **Keyboard 0**:
1. **Q/A:** Left Joystick
2. **E/D:** Right Joystick
3. **Z/X/C/V:** A/B/X/Y Buttons
4. **W/R:** Left/Right Bumpers
5. **S/F:** Left/Right Triggers
6. **Arrow Keys:** POV Buttons

## **Keyboard 1**:
1. **Y/H:** Left Joystick
2. **I/K:** Right Joystick
3. **B/N/M/,:** A/B/X/Y buttons
4. **U/O:** Left/Right Bumpers
5. **J/L:** Left/Right Triggers
6. **Number Pad Arrow Keys *(8/6/2/4)*:** POV Buttons