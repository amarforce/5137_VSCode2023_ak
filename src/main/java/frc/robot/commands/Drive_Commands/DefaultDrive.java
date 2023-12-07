
/*
 * 
 * DefaultDrive.java is a command that allows the robot to drive using the arcadeDrive method in DriveBase_Subsystem.java
 * 
 
 * The provided code is part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. 
 * The code is part of a class named DefaultDrive in the frc.robot.commands.Drive_Commands package.

The DefaultDrive class extends CommandBase, a class provided by WPILib. This indicates that DefaultDrive is a command in the WPILib framework. 
The purpose of this class, as described in the comments, is to allow the robot to drive using the arcadeDrive method in DriveBase_Subsystem.

The class has two member variables:

driveBase_Subsystem is an instance of DriveBase_Subsystem, representing the robot's driving mechanism.
controller is an instance of Joystick, representing the joystick controller used to drive the robot.
The DefaultDrive class has a constructor that takes a DriveBase_Subsystem and a Joystick as parameters. 
The constructor initializes the command with the specified drive subsystem and joystick controller, and declares the subsystem dependencies using the addRequirements method.

The class also overrides the execute method from CommandBase, which is called repeatedly while the command is active. 
However, the method is currently empty in this implementation.
 * 
 * 
 * The `DefaultDrive` class in the `frc.robot.commands.Drive_Commands` package is a command within the WPILib Command-Based framework, used in FIRST Robotics competition robots. This command is designed for the basic driving operation of the robot using a joystick controller. Let's dive into its structure and functionality:

### Class: `DefaultDrive`
- **Extends**: `CommandBase` - This indicates that `DefaultDrive` is a type of command in the WPILib framework.
- **Purpose**: To control the robot's movement using a joystick, typically in an arcade drive style.

### Member Variables
- **`driveBase_Subsystem`**: An instance of `DriveBase_Subsystem`, representing the robot's driving mechanism.
- **`controller`**: An instance of `Joystick`, used to capture the input from the joystick controller for driving the robot.

### Constructor: `DefaultDrive(DriveBase_Subsystem driveBase_Subsystem, Joystick controller)`
- **Parameters**:
  - `driveBase_Subsystem`: The robot's drive subsystem.
  - `controller`: The joystick controller used to drive the robot.
- **Functionality**: The constructor initializes the command with the specified drive subsystem and joystick controller. It also declares the subsystem dependencies using `addRequirements()` to ensure that this command has exclusive access to the `driveBase_Subsystem` while it is running.

### Overridden Method: `execute()`
- **Function**: Called repeatedly while the command is active.
- **Behavior**: The `execute` method calls the `arcadeDrive` method of the `driveBase_Subsystem`, passing the `controller` as a parameter. This allows the `driveBase_Subsystem` to interpret the joystick inputs and control the robot's movement accordingly.

### Usage in a Robot Program
- **Arcade Drive**: The `arcadeDrive` method in `DriveBase_Subsystem` likely combines the joystick inputs for both speed (forward/backward) and rotation (left/right) to control the robot in an arcade-style driving manner. This is a common control scheme in robotics, where one joystick axis controls the speed and another controls the turning.
- **Command Lifecycle**: The command is scheduled by the robot's command scheduler, and it continuously executes its driving logic during each scheduler run, typically until the robot is disabled or a different command requiring the same subsystem is scheduled.
- **Joystick Control**: By using a joystick for input, this command provides an intuitive way for drivers to control the robot, making it well-suited for teleoperated periods of a robotics match.

In summary, the `DefaultDrive` command is essential for controlling the robot's movements in teleoperated mode during a robotics competition. It showcases the integration of user input devices (like joysticks) with robotic control systems, allowing for responsive and intuitive control of the robot.
 * 
 */

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;

public class DefaultDrive extends CommandBase {
    
    private DriveBase_Subsystem driveBase_Subsystem;
    private Joystick controller;
  
   
    public DefaultDrive(DriveBase_Subsystem driveBase_Subsystem, Joystick controller) {
      this.driveBase_Subsystem = driveBase_Subsystem;
      this.controller = controller;
      addRequirements(driveBase_Subsystem);
    }
  
    @Override
    public void execute() {
      driveBase_Subsystem.arcadeDrive(controller);
    }
  }
