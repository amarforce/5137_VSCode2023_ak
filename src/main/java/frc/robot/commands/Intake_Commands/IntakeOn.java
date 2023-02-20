package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subystem;

public class IntakeOn extends CommandBase {
     Intake_Subystem intake_Subystem;

    public IntakeOn(Intake_Subystem intake_Subystem) {

        this.intake_Subystem = intake_Subystem;
        addRequirements(intake_Subystem);
     }
 
     @Override
     public void execute() {
        System.out.println("Intake started");
        intake_Subystem.extendIntake();
        //doesn't toggle intake on unless intake is currently off
        if (!intake_Subystem.getIntakeActive()) {
            intake_Subystem.runIntake(true);
        }
     }
 
     @Override
     public boolean isFinished() {
         if(intake_Subystem.getIntakeActive())
         {
        return true;
         }
         return false;
     }
}
