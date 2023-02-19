package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subystem;

public class IntakeOff extends CommandBase {
     Intake_Subystem intake_Subystem;

    public IntakeOff(Intake_Subystem intake_Subystem) {

        this.intake_Subystem = intake_Subystem;
        addRequirements(intake_Subystem);
     }
 
     @Override
     public void execute() {
        
        if (intake_Subystem.intakeActive) {
            intake_Subystem.stopIntake(); // Turns off intake
        }
        intake_Subystem.retractIntake(); //Pulls intake in

     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
} 
