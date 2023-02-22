// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm_Subsystem;
import frc.robot.subsystems.Intake_Subystem;

public class HybridPreset extends CommandBase {
  /** Creates a new HybridPreset. */
  Arm_Subsystem arm_Subsystem;
  Intake_Subystem intake_Subystem;
  
  public HybridPreset(Arm_Subsystem arm_Subsystem, Intake_Subystem intake_Subystem) {
    this.arm_Subsystem = arm_Subsystem;
    this.intake_Subystem = intake_Subystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm_Subsystem);
    addRequirements(intake_Subystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm_Subsystem.armMovementClear(intake_Subystem)){ 
      arm_Subsystem.moveArm(Constants.hybridRotation, Constants.hybridExtension);
      System.out.println("Hybrid");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm_Subsystem.armFinished(Constants.hybridRotation, Constants.hybridExtension);
  }
}
