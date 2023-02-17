package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeOff extends CommandBase {
    public IntakeOff() {
        addRequirements(RobotContainer.intake_Subystem);
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake off unless intake is currently on
        //technically unnecessary but we're keeping it 
        if (RobotContainer.intake_Subystem.intakeActive) { //whenever we get limitsitches add that here to make sure arm isn't in intake position.
            RobotContainer.intake_Subystem.retractIntake();
        }
        RobotContainer.intake_Subystem.stopIntake();
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
