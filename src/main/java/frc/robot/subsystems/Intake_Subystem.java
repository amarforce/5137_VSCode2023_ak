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

    SparkMaxWrapper topIntakeMotor;
    SparkMaxWrapper bottomIntakeMotor;

    public Intake_Subystem() {
        topIntakeMotor = new SparkMaxWrapper(Constants.topIntakePort, MotorType.kBrushless);
        bottomIntakeMotor = new SparkMaxWrapper(Constants.bottomIntakePort, MotorType.kBrushless);
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
            topIntakeMotor.set(Constants.intakeSpeed);
            bottomIntakeMotor.set(-Constants.intakeSpeed);
        } else {
            topIntakeMotor.set(-Constants.intakeSpeed);
            bottomIntakeMotor.set(Constants.intakeSpeed);
        }
    }

    public void stopIntake() {
        topIntakeMotor.set(0.0);
        bottomIntakeMotor.set(0.0);
    }
}
