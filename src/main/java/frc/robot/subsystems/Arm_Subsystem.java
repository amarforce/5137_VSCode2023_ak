// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm_Subsystem extends SubsystemBase {
  CANSparkMax armRotateMotor = new CANSparkMax(Constants.armRotatePort, MotorType.kBrushless);
  CANSparkMax armExtendMotor = new CANSparkMax(Constants.armExtendPort, MotorType.kBrushless);
  
  RelativeEncoder rotateEncoder = armRotateMotor.getEncoder();
  RelativeEncoder extendEncoder = armExtendMotor.getEncoder();

  public static int currentRotation;
  public static int currentExtension;

  public static boolean booleanArmFinished = false;

  //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
  //int pulsePerDegree = pulse / 360;    

  //final EncoderSim rotateEncoderSim = new EncoderSim(rotateEncoder);
  //final EncoderSim extendEncoderSim = new EncoderSim(extendEncoder);

  /** Creates a new Arm. */
  public Arm_Subsystem() {
    //possibly helpful info: 42 counts of the encoder for one rev on neos

    //delete these if it creates issues
    armRotateMotor.restoreFactoryDefaults();
    armExtendMotor.restoreFactoryDefaults();

    rotateEncoder.setPositionConversionFactor(0.0);          //sets origin 

    int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
    int pulsePerDegree = pulse / 360;                               //figures out how many pulses per degree, so we can use that
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armRotate(int desiredDegrees){
    double rotatePosition = rotateEncoder.getPosition() * Constants.rotationToDegreeConversion; //gear ratio TT
    
    if (rotatePosition == desiredDegrees){
      //should end the command
      booleanArmFinished = true;
    }
    else if (rotatePosition < desiredDegrees){
      armRotateMotor.set(Constants.armRotateSpeed);
      //System.out.println("Current rotate position: " + rotatePosition);
    } 
    else if (rotatePosition > desiredDegrees) {
      armRotateMotor.set(-Constants.armRotateSpeed);

      currentRotation = desiredDegrees;
      //System.out.println("Rotation counter is at: " + currentRotation);
    }
  }

  public void armExtend(int desiredNumRotations){
    double extendPosition = extendEncoder.getPosition();
    
    if (extendPosition < desiredNumRotations){
      armExtendMotor.set(Constants.armExtendSpeed);
    }
    if (extendPosition >= desiredNumRotations){
      armExtendMotor.set(0);

      currentExtension = desiredNumRotations;
    }
  }

  public void armStop(Boolean stopBoolean){
    if (stopBoolean == true){
      armExtendMotor.set(0);
      armRotateMotor.set(0);
      stopBoolean = false;
    }
  }
}
