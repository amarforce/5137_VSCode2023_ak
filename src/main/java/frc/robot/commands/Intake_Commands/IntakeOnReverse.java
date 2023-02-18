package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subystem;

public class IntakeOnReverse extends CommandBase {
    private Intake_Subystem intake_Subystem;

    public IntakeOnReverse (Intake_Subystem intake_Subystem) {

        this.intake_Subystem = intake_Subystem;
        addRequirements(this.intake_Subystem);
     }
 
     @Override
     public void execute() {
        if (!intake_Subystem.intakeActive) {
            intake_Subystem.extendIntake();
           }
        intake_Subystem.runIntake(false);
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
