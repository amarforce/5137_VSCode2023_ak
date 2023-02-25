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


    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}