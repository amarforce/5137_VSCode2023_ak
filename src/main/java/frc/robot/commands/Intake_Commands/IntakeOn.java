package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOn extends CommandBase{
    public IntakeOn() {
        addRequirements(RobotContainer.intake_Subystem);
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake on unless intake is currently off
        if (!RobotContainer.intake_Subystem.intakeActive) {
         RobotContainer.intake_Subystem.extendIntake();
        }
         RobotContainer.intake_Subystem.runIntake(true);
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
