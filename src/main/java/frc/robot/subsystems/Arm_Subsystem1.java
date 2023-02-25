// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.simulation.SparkMaxWrapper;

//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm_Subsystem1 extends ProfiledPIDSubsystem {
  //CANSparkMax armRotateMotor = new CANSparkMax(Constants.armRotatePort, MotorType.kBrushless);
  //CANSparkMax armExtendMotor = new CANSparkMax(Constants.armExtendPort, MotorType.kBrushless);
  SparkMaxWrapper armRotateMotor = new SparkMaxWrapper(Constants.armRotatePort, MotorType.kBrushless);
  SparkMaxWrapper armExtendMotor = new SparkMaxWrapper(Constants.armExtendPort, MotorType.kBrushless);
  
  public RelativeEncoder rotateEncoder = armRotateMotor.getEncoder();
  RelativeEncoder extendEncoder = armExtendMotor.getEncoder();

  PIDController armPidController = new PIDController(1, 0, 0);

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        Constants.kSVolts, Constants.kGVolts,
        Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

  //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
  //int pulsePerDegree = pulse / 360;    

  //final EncoderSim rotateEncoderSim = new EncoderSim(rotateEncoder);
  //final EncoderSim extendEncoderSim = new EncoderSim(extendEncoder);

  /** Creates a new Arm. */
  public Arm_Subsystem1() {
    //possibly helpful info: 42 counts of the encoder for one rev on neos
    super(
      new ProfiledPIDController(
          Constants.aKP,
          Constants.aKD,
          Constants.aKI,
          new TrapezoidProfile.Constraints(
              Constants.armMaxVelocity,
              Constants.armMaxAccel)),
      0);
    //delete these if it creates issues
    armRotateMotor.restoreFactoryDefaults();
    armExtendMotor.restoreFactoryDefaults();

    rotateEncoder.setPositionConversionFactor(Constants.rotationToDegreeConversion);          
    extendEncoder.setPositionConversionFactor(Constants.rotationToDegreeConversion);

    // Sets origin 
    rotateEncoder.setPosition(0.0);
    extendEncoder.setPosition(0.0);

    super.enable();
  
    //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
    //int pulsePerDegree = pulse / 360;                               //figures out how many pulses per degree, so we can use that
    }
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint)
    {
      double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
      armRotateMotor.setVoltage(output + feedforward);
<<<<<<< HEAD
      System.out.println("Output" + output);
=======
    System.out.println("output" + output+feedforward);
>>>>>>> de32ac0ec4390b7a03c2f3b859998d4f57400a76
    }

    @Override
  public double getMeasurement() {
    return rotateEncoder.getPosition();
  }

  public void setPositionDegrees(double degrees)
  {
    double currentDegrees = rotateEncoder.getPosition();
    double voltageSpeed = armPidController.calculate(currentDegrees, degrees);
    super.setGoal(voltageSpeed);
  }
 
  
}

   