
/*
 * 
 * The provided code is part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. 
 * The code is part of a class named AutoRotate in the frc.robot.commands.Drive_Commands package.

The AutoRotate class extends CommandBase, a class provided by WPILib. This indicates that AutoRotate is a command in the WPILib framework. 
The purpose of this class is to autonomously rotate the robot to a target orientation.

The class has three member variables:

targetPose is an instance of Pose2d, representing the target position and orientation the robot should reach.
driveBase_Subsystem is an instance of DriveBase_Subsystem, representing the robot's driving mechanism.
motorSpeed is a double variable that will likely store the speed at which the robot's motors should run to reach the target pose, although it's not used in the provided code.
The AutoRotate class has a constructor that takes a DriveBase_Subsystem and a Pose2d as parameters. 
The constructor initializes the command with the specified drive subsystem and target pose, and declares the subsystem dependencies using the addRequirements method.

The class also overrides the initialize method from CommandBase, which is called when the command is initially scheduled. However, the method is currently empty in this implementation.
 * 
 * 
 * The `AutoRotate` class, part of the `frc.robot.commands.Drive_Commands` package, is a command within the WPILib Command-Based framework for FIRST Robotics competition robots. 
 * This class is specifically designed to autonomously rotate the robot to a desired orientation. Let's explore its structure and functionalities:

### Class: `AutoRotate`
- **Extends**: `CommandBase` - Signifies that `AutoRotate` is a command following the WPILib framework's command-based structure.
- **Purpose**: To autonomously rotate the robot to a specific orientation defined by the `targetPose`.

### Member Variables
- **`targetPose`**: An instance of `Pose2d`, which represents the target pose (position and orientation) the robot aims to achieve.
- **`driveBase_Subsystem`**: An instance of `DriveBase_Subsystem`, the robot's driving mechanism.
- **`motorSpeed`**: A double variable, likely representing the speed or power applied to the motors during the rotation process.

### Constructor: `AutoRotate(DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose)`
- **Parameters**: 
  - `driveBase_Subsystem`: The robot's drive subsystem.
  - `targetPose`: The desired orientation and position.
- **Functionality**: Initializes the command with the given drive subsystem and target pose. It also declares the subsystem dependencies using `addRequirements()`.

### Overridden Methods
1. **`initialize()`**: Called when the command is initially scheduled. Currently empty, it could be used for setup if needed.
2. **`execute()`**: Called repeatedly while the command is active. It calculates the motor speed necessary to rotate the robot towards the `targetPose`, using the `autoRotate` method from the `driveBase_Subsystem`.
3. **`end(boolean interrupted)`**: Called once the command ends or is interrupted. It's empty in this implementation, but could be used for cleanup actions.
4. **`isFinished()`**: Returns `true` when the command should end. The condition for completion is when `motorSpeed` is below a threshold (0.1), indicating the robot is close to achieving or has achieved the desired orientation.

### Usage in a Robot Program
- **Command Lifecycle**: The command is scheduled, executed, and terminated by the robot's command scheduler. It runs its `execute` method periodically until the `isFinished` condition is met.
- **Subsystem Interaction**: The command works in conjunction with the `DriveBase_Subsystem` to perform the rotation. This showcases the interaction between commands and subsystems in the command-based architecture.
- **Autonomous Rotation**: The `autoRotate` method in `DriveBase_Subsystem` is presumably responsible for the logic and calculations needed to rotate the robot autonomously, utilizing sensor data and control algorithms.

In summary, the `AutoRotate` command is an autonomous operation within the robot's control system. It highlights the integration of motion control and sensor feedback in robotic systems, enabling the robot to autonomously rotate to a specified orientation. This type of command is crucial in scenarios where precise orientation control is required, such as in alignment for tasks or navigating complex environments.
 * 
 */

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;

public class AutoRotate extends CommandBase{
    Pose2d targetPose;
    DriveBase_Subsystem driveBase_Subsystem;
    double motorSpeed;

    public AutoRotate(DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose)
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
    motorSpeed = driveBase_Subsystem.autoRotate(targetPose);
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
