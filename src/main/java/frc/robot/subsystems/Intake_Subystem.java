package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.simulation.SparkMaxWrapper;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake_Subystem extends SubsystemBase {
    boolean intakeActive = false;

    SparkMaxWrapper intakeMotorOne;
    SparkMaxWrapper intakeMotorTwo;

    public Intake_Subystem() {
        intakeMotorOne = new SparkMaxWrapper(Constants.intakeOnePort, MotorType.kBrushless);
        intakeMotorTwo = new SparkMaxWrapper(Constants.intakeTwoPort, MotorType.kBrushless);
    }

    public void activateIntake() {
        if (intakeActive) {
            //Retract intake
            intakeActive = false;
        } else {
            //Activate intake
            intakeActive = true;
        }
    }

    public void intake(boolean direction) {
        if (direction) {
            intakeMotorOne.set(Constants.intakeSpeed);
            intakeMotorTwo.set(-Constants.intakeSpeed);
        } else {
            intakeMotorOne.set(-Constants.intakeSpeed);
            intakeMotorTwo.set(Constants.intakeSpeed);
        }
    }

    public void stopIntake() {
        intakeMotorOne.set(0.0);
        intakeMotorTwo.set(0.0);
    }
}
