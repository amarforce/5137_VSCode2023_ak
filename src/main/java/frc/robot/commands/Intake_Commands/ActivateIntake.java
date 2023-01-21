package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ActivateIntake extends CommandBase {
    public ActivateIntake() {
       addRequirements(RobotContainer.intake_Subystem);
    }

    @Override
    public void execute() {
        RobotContainer.intake_Subystem.activateIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
