package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake_Subystem extends SubsystemBase {
    private boolean intakeActive;

    public static CANSparkMax intakeMotor;
   

    public Intake_Subystem() {
        intakeMotor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
        //intakeMotor = new SparkMaxWrapper(Constants.intakePort, MotorType.kBrushless);
        intakeActive = false;
    }

    public void runIntake(boolean direction) {
        if (direction) {
            intakeMotor.set(Constants.intakeSpeed);
            intakeActive = true;
        } 
        else {
            intakeMotor.set(-Constants.intakeSpeed);
            intakeActive = true;
        }
    }

    public boolean getIntakeActive()
    {
        return intakeActive;
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
