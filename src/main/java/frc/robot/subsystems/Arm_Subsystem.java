// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.simulation.SparkMaxWrapper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm_Subsystem extends SubsystemBase {
  //CANSparkMax armRotateMotor = new CANSparkMax(Constants.armRotatePort, MotorType.kBrushless);
  //CANSparkMax armExtendMotor = new CANSparkMax(Constants.armExtendPort, MotorType.kBrushless);
  SparkMaxWrapper armRotateMotor = new SparkMaxWrapper(Constants.armRotatePort, MotorType.kBrushless);
  SparkMaxWrapper armExtendMotor = new SparkMaxWrapper(Constants.armExtendPort, MotorType.kBrushless);
  
  public RelativeEncoder rotateEncoder = armRotateMotor.getEncoder();
  RelativeEncoder extendEncoder = armExtendMotor.getEncoder();

  public static double desiredRotation = 0.0;
  public static double desiredExtension = 0.0;

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

    rotateEncoder.setPositionConversionFactor(Constants.rotationToDegreeConversion);          
    extendEncoder.setPositionConversionFactor(Constants.rotationToDegreeConversion);

    // Sets origin 
    rotateEncoder.setPosition(0.0);
    extendEncoder.setPosition(0.0);

    //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
    //int pulsePerDegree = pulse / 360;                               //figures out how many pulses per degree, so we can use that
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      arcadeArm();
    }

    private void arcadeArm() {
      armRotate();
      armExtend();
    }

    public void moveArm(double rotation, double extension) {
      desiredRotation = rotation;
      desiredExtension = extension;
    }

    private void armRotate() {
      double rotatePosition = rotateEncoder.getPosition();
      //System.out.println("Desired Rotation:"+desiredRotation+"Current Rotation:"+rotatePosition);
      if (Math.abs(rotatePosition-desiredRotation) < 1) {
        armRotateMotor.stopMotor();
      }
      else if (rotatePosition < desiredRotation) {
        armRotateMotor.set(Constants.armRotateSpeed);
      } 
      else if (rotatePosition > desiredRotation) {
        armRotateMotor.set(-Constants.armRotateSpeed);
      }
    }

    private void armExtend() {
      double extendPosition = rotateEncoder.getPosition();
      //System.out.println("Desired Extension:"+desiredExtension+"Current Extension:"+extendPosition);
      if (Math.abs(extendPosition-desiredExtension) < 1){
        armExtendMotor.stopMotor();
      }
      else if (extendPosition < desiredExtension){
        armExtendMotor.set(Constants.armExtendSpeed);
      } 
      else if (extendPosition > desiredExtension) {
        armExtendMotor.set(-Constants.armExtendSpeed);
      }
    }

    public boolean armFinished(double rotation, double extention ){
      double rotatePosition = rotateEncoder.getPosition();
      double extendPosition = rotateEncoder.getPosition();
      if ((rotatePosition == rotation) && (extendPosition == extention)){
        return true;
      }
      return false;
     }

         
    //checks that arm is "above" motor or that intake is extended before moving. 
     public boolean armMovementClear(){
      if (rotateEncoder.getPosition() >= Constants.intakeRotationSafe || RobotContainer.intake_Subystem.intakeActive){
        return true;
      }
      return false;
     }
    

  
}
