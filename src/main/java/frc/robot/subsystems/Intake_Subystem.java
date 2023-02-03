package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
//import frc.robot.RobotContainer;

public class Intake_Subystem extends SubsystemBase {
    public boolean intakeActive = false;

    public static WPI_VictorSPX intakeMotor;

    public Intake_Subystem() {
        intakeMotor = new WPI_VictorSPX(Constants.intakePort);
    }

    public void runIntake(boolean direction) {
        if (direction) {
            intakeMotor.set(Constants.intakeSpeed);
        } else {
            intakeMotor.set(-Constants.intakeSpeed);
        }
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void extendIntake() {
        Pneumatics_Subsystem.intakeSolenoid.set(true);
        intakeActive = true;
    }

    public void retractIntake() {
        Pneumatics_Subsystem.intakeSolenoid.set(false);
        intakeActive = false;
    }
}
