/*
 * 
 * 
 * The provided code is part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. 
 * The code is part of a class named AddVisionMeasurement in the frc.robot.commands.Vision_Commands package.

The AddVisionMeasurement class extends CommandBase, a class provided by WPILib. This indicates that AddVisionMeasurement is a command in the WPILib framework. 
The purpose of this class is likely to add a vision measurement to the robot's understanding of its environment.

The class has four member variables:

driveBase_Subsystem is an instance of DriveBase_Subsystem, representing the robot's driving mechanism.
vision_Subsystem is an instance of Vision_Subsystem, representing the robot's vision system.
estimatedPose is an instance of Pose2d, representing the robot's estimated position and orientation.
timestamp is a double variable that will likely store the time at which the vision measurement was taken, although it's not used in the provided code.
The AddVisionMeasurement class has a constructor that takes a DriveBase_Subsystem and a Vision_Subsystem as parameters. 
The constructor initializes the command with the specified drive subsystem and vision subsystem, and declares the subsystem dependencies using the addRequirements method.
 * 
 * 
 * The `AddVisionMeasurement` class in the `frc.robot.commands.Vision_Commands` package is a command within the WPILib Command-Based framework, designed for use in FIRST Robotics competition robots. 
 * This command integrates vision processing with the robot's driving system. Let's analyze its structure and functionalities:

### Class: `AddVisionMeasurement`
- **Extends**: `CommandBase` - This signifies that `AddVisionMeasurement` is a command following the WPILib framework's structure.
- **Purpose**: To process vision data and use it to inform the robot's driving system about its position and orientation.

### Member Variables
- **`driveBase_Subsystem`**: An instance of `DriveBase_Subsystem`, representing the robot's driving mechanism.
- **`vision_Subsystem`**: An instance of `Vision_Subsystem`, responsible for handling vision processing.
- **`estimatedPose`**: A `Pose2d` object to store the robot's estimated pose based on vision data.
- **`timestamp`**: A double variable to store the timestamp of the vision measurement.

### Constructor: `AddVisionMeasurement(DriveBase_Subsystem driveBase_Subsystem, Vision_Subsystem vision_Subsystem)`
- **Parameters**: 
  - `driveBase_Subsystem`: The robot's drive subsystem.
  - `vision_Subsystem`: The vision subsystem used for processing vision data.
- **Functionality**: Initializes the command with the specified subsystems and declares the vision subsystem as a requirement.

### Overridden Methods
1. **`initialize()`**: Called when the command is initially scheduled. Currently empty, but could be used for setup actions.
2. **`execute()`**: Called repeatedly while the command is active. It:
   - Retrieves vision data from two different cameras (`ar1CamResult` and `ar2CamResult`) using the `vision_Subsystem`.
   - If a result is present from either camera, it converts the estimated robot pose to `Pose2d` and records the timestamp.
   - These measurements are then added to the `driveBase_Subsystem` for further processing or use in navigation.
3. **`end(boolean interrupted)`**: Called once the command ends or is interrupted. No specific implementation is provided here.
4. **`isFinished()`**: Always returns `false`, indicating that this command continuously runs to update vision measurements.

### Usage in a Robot Program
- **Continuous Vision Processing**: This command continuously processes vision data, making it suitable for real-time navigation and positioning tasks in robotics.
- **Integration with Driving System**: By providing updated vision data to the `driveBase_Subsystem`, the command allows for sophisticated navigation strategies that can react to the robot's environment.
- **Multi-Camera Support**: The command's ability to handle data from multiple cameras (`ar1CamResult` and `ar2CamResult`) showcases the flexibility in handling diverse vision inputs.

In summary, the `AddVisionMeasurement` command is a key part of a robotic control system that integrates vision processing with motion control. It demonstrates the use of real-time vision data to inform and enhance the robot's navigation and positioning capabilities, which is crucial in dynamic and visually complex environments often encountered in robotics competitions.
 * 
 */


package frc.robot.commands.Vision_Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;
import frc.robot.subsystems.Vision_Subsystem;

public class AddVisionMeasurement extends CommandBase {

  DriveBase_Subsystem driveBase_Subsystem;
  Vision_Subsystem vision_Subsystem;
  Pose2d estimatedPose;
  double timestamp;

  public AddVisionMeasurement(DriveBase_Subsystem driveBase_Subsystem, Vision_Subsystem vision_Subsystem) {
    this.driveBase_Subsystem = driveBase_Subsystem;
    this.vision_Subsystem = vision_Subsystem;
    addRequirements(vision_Subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Optional<EstimatedRobotPose> ar1CamResult = vision_Subsystem.getPoseFromAR1CamCamera(driveBase_Subsystem.getPose());
    Optional<EstimatedRobotPose> ar2CamResult = vision_Subsystem.getPoseFromAR2CamCamera(driveBase_Subsystem.getPose());

    if (ar1CamResult.isPresent()) {
      estimatedPose = ar1CamResult.get().estimatedPose.toPose2d();
      timestamp = ar1CamResult.get().timestampSeconds;
      driveBase_Subsystem.addVisionMeasurement(estimatedPose, timestamp);
    }
    if (ar2CamResult.isPresent()) {
      estimatedPose = ar2CamResult.get().estimatedPose.toPose2d();
      timestamp = ar2CamResult.get().timestampSeconds;
      driveBase_Subsystem.addVisionMeasurement(estimatedPose, timestamp);
    }
    // Add two more Optional<EstimatedRobotPose> variables for the new cameras

    Optional<EstimatedRobotPose> ar3CamResult = vision_Subsystem.getPoseFromAR3CamCamera(driveBase_Subsystem.getPose());
Optional<EstimatedRobotPose> ar4CamResult = vision_Subsystem.getPoseFromAR4CamCamera(driveBase_Subsystem.getPose());

// Handle the results from the new cameras
if (ar3CamResult.isPresent()) {
  estimatedPose = ar3CamResult.get().estimatedPose.toPose2d();
  timestamp = ar3CamResult.get().timestampSeconds;
  driveBase_Subsystem.addVisionMeasurement(estimatedPose, timestamp);
}
if (ar4CamResult.isPresent()) {
  estimatedPose = ar4CamResult.get().estimatedPose.toPose2d();
  timestamp = ar4CamResult.get().timestampSeconds;
  driveBase_Subsystem.addVisionMeasurement(estimatedPose, timestamp);
}


    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}