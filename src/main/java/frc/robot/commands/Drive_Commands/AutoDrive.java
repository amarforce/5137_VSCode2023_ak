// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * 
 * The provided code is part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. 
 * The code is part of a class named AutoDrive in the frc.robot.commands.Drive_Commands package.

The AutoDrive class extends CommandBase, a class provided by WPILib. This indicates that AutoDrive is a command in the WPILib framework.
 The purpose of this class is to autonomously drive the robot to a target position and orientation.

The class has three member variables:

targetPose is an instance of Pose2d, representing the target position and orientation the robot should reach.
driveBase_Subsystem is an instance of DriveBase_Subsystem, representing the robot's driving mechanism.
motorSpeed is a double variable that will store the speed at which the robot's motors should run to reach the target pose.
The AutoDrive class has a constructor that takes a DriveBase_Subsystem and a Pose2d as parameters. 
The constructor initializes the command with the specified drive subsystem and target pose, and declares the subsystem dependencies.

The class also overrides four methods from CommandBase:

initialize(), which is called when the command is initially scheduled. It's currently empty in this implementation.
execute(), which is called repeatedly while the command is active. 
It computes the motor speed needed to reach the target pose by calling driveBase_Subsystem.autoDrive(targetPose).
end(boolean interrupted), which is called once the command ends or is interrupted. No specific implementation is provided here.
isFinished(), which returns true when the command should end. 
The end condition checks if the motor speed is below a certain threshold (0.1), indicating that the robot is close to or has reached the target pose.
 * 
 * The `AutoDrive` class in the `frc.robot.commands.Drive_Commands` package is a command within the WPILib Command-Based framework, designed to autonomously drive a robot to a specified pose (position and orientation) using its drive subsystem. Let's analyze its structure and functionalities:

### Class: `AutoDrive`
- **Extends**: `CommandBase` - This indicates that `AutoDrive` is a command in the WPILib framework.
- **Purpose**: To autonomously drive the robot to a target position and orientation.

### Member Variables
- **`targetPose`**: An instance of `Pose2d`, representing the target position and orientation the robot should reach.
- **`driveBase_Subsystem`**: Instance of `DriveBase_Subsystem`, representing the robot's driving mechanism.
- **`motorSpeed`**: A double variable that will store the speed at which the robot's motors should run to reach the target pose.

### Constructor: `AutoDrive(DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose)`
- **Parameters**: 
  - `driveBase_Subsystem`: The robot's drive subsystem.
  - `targetPose`: The desired position and orientation.
- **Functionality**: Initializes the command with the specified drive subsystem and target pose, and declares the subsystem dependencies.

### Overridden Methods
1. **`initialize()`**: Called when the command is initially scheduled. It's currently empty in this implementation.
2. **`execute()`**: Called repeatedly while the command is active. It computes the motor speed needed to reach the target pose by calling `driveBase_Subsystem.autoDrive(targetPose)`.
3. **`end(boolean interrupted)`**: Called once the command ends or is interrupted. No specific implementation is provided here.
4. **`isFinished()`**: Returns `true` when the command should end. The end condition checks if the motor speed is below a certain threshold (0.1), indicating that the robot is close to or has reached the target pose.

### Usage in a Robot Program
- **Command Lifecycle**: Scheduled by the robot's command scheduler, it initializes, executes its logic during each scheduler run, and ends when its conditions are met or if interrupted.
- **Subsystem Interaction**: The command interacts with the `DriveBase_Subsystem`, leveraging it to autonomously navigate to the target pose.
- **Autonomous Navigation**: The `autoDrive` method in the `DriveBase_Subsystem` likely includes algorithms for path planning and motor control to accurately drive the robot to the target pose, making use of sensor feedback for precision.

In summary, the `AutoDrive` command is a specialized operation within a robot's control system, using the WPILib Command-Based framework to enable autonomous navigation. It demonstrates the integration of motion control, path planning, and sensor feedback in robotics, allowing the robot to move autonomously to a specified location and orientation.
 * 
 */

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
public class AutoDrive extends CommandBase {

    Pose2d targetPose;
    DriveBase_Subsystem driveBase_Subsystem;
    double motorSpeed;

    public AutoDrive(DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose)
    {
      addRequirements(driveBase_Subsystem);

      this.targetPose = targetPose;
      this.driveBase_Subsystem = driveBase_Subsystem;
    }


  @Override 
   public void initialize()
   {}

  @Override
  public void execute()
  {
    motorSpeed = driveBase_Subsystem.autoDrive(targetPose);
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(motorSpeed < 0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}