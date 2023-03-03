package frc.robot.commands.Clamp_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Clamp_Subsystem;

public class ClampClose extends CommandBase {
  
  Clamp_Subsystem clamp_Subsystem;

  public ClampClose (Clamp_Subsystem clamp_Subsystem) {
     this.clamp_Subsystem = clamp_Subsystem; 
   addRequirements(clamp_Subsystem);
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clamp_Subsystem.Clamp();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(clamp_Subsystem.getClamped())
    {
      System.out.println("Game Piece Clamped");
      return true;
    }
    return false;
  } 
}
