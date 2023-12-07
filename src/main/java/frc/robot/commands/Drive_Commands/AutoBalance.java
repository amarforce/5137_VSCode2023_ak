// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * 
 * The provided code is part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. 
 * The code is part of a class named AutoBalance in the frc.robot.commands.Drive_Commands package.

The AutoBalance class is likely to extend CommandBase, a class provided by WPILib, as it imports CommandBase. 
This class is used to create commands, which are reusable chunks of code that can be run in response to various events.

The AutoBalance class also imports AHRS from the com.kauailabs.navx.frc package, which is a class for interfacing with the NavX-MXP and NavX-Micro sensors from Kauai Labs. 
These sensors are often used in robotics for getting information about the robot's movement and orientation.

The DriveBase_Subsystem from the frc.robot.subsystems package is also imported, which suggests that this command will interact with the drive base subsystem of the robot.

The comment above the class explains that the left stick controls the direction and speed of translation, the right trigger will slow the robot down as an analog input, and the right stick controls the rate of rotation. 
This suggests that this command is used to control the movement of the robot.
 * 
 * The `AutoBalance` class is a command in the WPILib Command-Based framework, specifically designed for a robotics application in the `frc.robot.commands.Drive_Commands` package. 
 * Its primary function is to automatically balance the robot using its drive system and a gyro sensor. Here's a detailed breakdown of its components and functionalities:

### Class: `AutoBalance`
- **Extends**: `CommandBase` - Indicates that `AutoBalance` is a command in the WPILib framework.
- **Purpose**: To automate the balancing of the robot using its drive subsystem and gyroscopic sensor data.

### Member Variables
- **`driveBase_Subsystem`**: Instance of `DriveBase_Subsystem`, representing the robot's driving mechanism.
- **`motorSpeed`**: A double variable to store the speed at which motors should run to balance the robot.
- **`gyro`**: An instance of `AHRS` (Attitude and Heading Reference System), used for accessing gyroscopic data to assist in balancing.
- **`gyroAngle`**: A double variable to store the current pitch angle from the gyro sensor.

### Constructor: `AutoBalance(DriveBase_Subsystem driveBase_Subsystem)`
- **Parameters**: An instance of `DriveBase_Subsystem`.
- **Functionality**: Initializes the command with the drive subsystem and sets up the gyro sensor.

### Overridden Methods
1. **`initialize()`**: Called when the command is initially scheduled. It's currently empty in this implementation.
2. **`execute()`**: Called repeatedly while the command is active. It retrieves the motor speed required for balancing from the drive subsystem (`autoBalance()`) and reads the current pitch angle from the gyro sensor.
3. **`end(boolean interrupted)`**: Called once the command ends or is interrupted. No specific implementation is provided here.
4. **`isFinished()`**: Returns `true` when the command should end. The end condition checks if the motor speed is below a threshold (0.05) and if the absolute value of the gyro angle is less than 2 degrees, indicating that the robot is balanced.

### Usage in a Robot Program
- **Command Lifecycle**: The command is scheduled by the robot's command scheduler. 
It initializes, then executes its logic repeatedly during each scheduler run, and ends when its conditions are met or if interrupted.
- **Subsystem Interaction**: The command interacts with the `DriveBase_Subsystem` and uses the gyro sensor for balancing. 
This integration is crucial for automatic balance control of the robot.
- **Balancing Logic**: The command uses real-time gyroscopic data to adjust the robot's position, ensuring stability and balance. 
The specific balancing algorithm is encapsulated within the `DriveBase_Subsystem.autoBalance()` method.

In summary, the `AutoBalance` command is a specialized operation within a robot's control system, using the WPILib Command-Based framework to automate the balancing of the robot. 
It intelligently combines drive system control with gyroscopic data to maintain stability, highlighting the integration of mechanical and sensor systems in robotics.
 * 
 */

package frc.robot.commands.Drive_Commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase_Subsystem;


/**
 * Drive the robot.
 * 
 * Left stick controls the direction and speed of translation.
 * Right trigger will slow the robot down as an analog input.
 * 
 * Right stick controls the rate of rotation.
 *
 */
public class AutoBalance extends CommandBase {

    DriveBase_Subsystem driveBase_Subsystem;
    double motorSpeed;
    AHRS gyro;
    double gyroAngle;


    public AutoBalance(DriveBase_Subsystem driveBase_Subsystem)
    {
      addRequirements(driveBase_Subsystem);

      this.driveBase_Subsystem = driveBase_Subsystem;
      gyro = DriveBase_Subsystem.gyro;
    }


  @Override 
   public void initialize()
   {}

  @Override
  public void execute()
  {
    motorSpeed = driveBase_Subsystem.autoBalance();
    gyroAngle = gyro.getPitch();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(motorSpeed < 0.05 && Math.abs(gyroAngle) < 2)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}