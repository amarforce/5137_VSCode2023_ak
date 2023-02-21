package frc.robot.commands.Clamp_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamp_Subsystem;

public class ClampOpen extends CommandBase {

  Clamp_Subsystem clamp_Subsystem;

 public ClampOpen (Clamp_Subsystem clamp_Subsystem) {
    this.clamp_Subsystem = clamp_Subsystem; 
  addRequirements(clamp_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clamp_Subsystem.Release();
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
    if(!clamp_Subsystem.getClamped())
    {
      System.out.println("Clamp Opened");
      return true;
    }
    return false;
  } 
}
