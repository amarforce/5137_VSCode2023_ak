// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class MidCubePreset extends CommandBase {
  /** Creates a new MidCubePreset. */
  public MidCubePreset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.arm_Subsystem.armRotate(Constants.middleCubeDegrees);
    RobotContainer.arm_Subsystem.armExtend(Constants.middleCubeExtension);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm_Subsystem.armStop(Arm_Subsystem.booleanArmFinished);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
