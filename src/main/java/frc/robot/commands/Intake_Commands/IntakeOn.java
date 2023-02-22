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
        intake_Subystem.extendIntake();
        //doesn't toggle intake on unless intake is currently off
     
            intake_Subystem.runIntake(true);
        
     }
 
     @Override
     public boolean isFinished() {
         if(intake_Subystem.getIntakeActive())
         {
            System.out.println("Intake started");
            return true;
         }
         return false;
     }
}
