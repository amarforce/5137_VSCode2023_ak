package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.simulation.SparkMaxWrapper;
import frc.robot.Constants;
import frc.robot.subsystems.Pneumatics_Subsystem;
import frc.robot.RobotContainer;

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
