package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subystem;

public class IntakeOff extends CommandBase {
    private Intake_Subystem intake_Subystem;

    public IntakeOff(Intake_Subystem intake_Subystem) {

        this.intake_Subystem = intake_Subystem;
        addRequirements(this.intake_Subystem);
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake off unless intake is currently on
        //technically unnecessary but we're keeping it 
        if (intake_Subystem.intakeActive) {
            intake_Subystem.retractIntake();
        }
        intake_Subystem.stopIntake();
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
