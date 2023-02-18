// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Compressor_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pneumatics_Subsystem;

public class CompressorOff extends CommandBase {
  /** Creates a new CompressorOff. */
  Pneumatics_Subsystem pneumatics_Subsystem;

  public CompressorOff(Pneumatics_Subsystem pneumatics_Subsystem) {
    this.pneumatics_Subsystem = pneumatics_Subsystem;

    addRequirements(pneumatics_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics_Subsystem.compress(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
