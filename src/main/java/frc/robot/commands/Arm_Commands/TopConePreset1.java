// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm_Subsystem;
import frc.robot.subsystems.Arm_Subsystem1;
import frc.robot.subsystems.Intake_Subystem;

public class TopConePreset1 extends CommandBase {
  /** Creates a new TopConePreset. */
  Arm_Subsystem1 arm_Subsystem1;
  
  public TopConePreset1(Arm_Subsystem1 arm_Subsystem1) {
    this.arm_Subsystem1 = arm_Subsystem1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm_Subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   arm_Subsystem1.setGoal(30);
   System.out.println("Set arm subsytem1 target to 30");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    
  }
}
