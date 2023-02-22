package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subystem;

public class IntakeOnReverse extends CommandBase {
     Intake_Subystem intake_Subystem;

    public IntakeOnReverse (Intake_Subystem intake_Subystem) {

        this.intake_Subystem = intake_Subystem;
        addRequirements(intake_Subystem);
     }
 
     @Override
     public void execute() {
        intake_Subystem.extendIntake();
            intake_Subystem.runIntake(false);
     }

 
     @Override
     public boolean isFinished() {

         if(intake_Subystem.getIntakeActive())
         {
            System.out.println("Intake started in reverse");
            return true;
         }
         return false;
     }
}
