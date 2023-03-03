package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shuffleboard_Subsystem extends SubsystemBase {
    SendableChooser<String> autoChooser = new SendableChooser<>();

    public Shuffleboard_Subsystem() {
        configureSendableString(autoChooser, "AutoOne", "AutoOne", "AutoTwo", "AutoThree");

        SmartDashboard.putData("Auto Choice", autoChooser);
        SmartDashboard.putData("jMoneyDrive", DriveBase_Subsystem.jMoney_Drive);

        update();
    }

    

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        SmartDashboard.putNumber("Intake Motor", RobotContainer.intake_Subystem.intakeMotor.get() );
        SmartDashboard.putNumber("Arm Rotate Motor", RobotContainer.arm_Subsystem.armRotateMotor.get());
        SmartDashboard.putNumber("Arm Extend Motor", RobotContainer.arm_Subsystem.armExtendMotor.get());
        SmartDashboard.putNumber("Arm Rotate Position", RobotContainer.arm_Subsystem.rotateEncoder.getPosition());
        SmartDashboard.putNumber("Arm Extend Position", RobotContainer.arm_Subsystem.extendEncoder.getPosition());
        SmartDashboard.putBoolean("Intake Solenoid", RobotContainer.intake_Subystem.getIntakeActive());
        SmartDashboard.putBoolean("Clamp Solenoid", RobotContainer.pneumatics_Subsystem.clampSolenoid.get());
    }

    private void configureSendableString(SendableChooser<String> chooser, String kDefault, String... kOptions) {
        chooser.setDefaultOption(kDefault, kDefault);
        for (String Option:kOptions) {
            chooser.addOption(Option, Option);
        }
    }

    private void configureSendableBoolean(SendableChooser<Boolean> chooser, Boolean kDefault) {
        if (kDefault) {
            chooser.setDefaultOption("Enabled", true);
        } else {
            chooser.setDefaultOption("Disabled", false);
        }
        chooser.addOption("Enabled", true);
        chooser.addOption("Disabled", false);
    }
}