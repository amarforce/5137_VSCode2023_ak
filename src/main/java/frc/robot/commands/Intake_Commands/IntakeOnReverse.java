package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOnReverse extends CommandBase{
    public IntakeOnReverse() {
        addRequirements(RobotContainer.intake_Subystem);
     }
 
     @Override
     public void execute() {
         RobotContainer.intake_Subystem.intake(false);
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
