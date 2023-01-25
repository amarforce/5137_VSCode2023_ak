// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveBase_Subsystem extends SubsystemBase {
  //left motors
  public static WPI_TalonSRX leftTalon;
  public static WPI_VictorSPX leftFrontVic;
  public static WPI_VictorSPX leftBackVic;
  public static MotorControllerGroup leftDrive;
  
  //right motors
  public static WPI_TalonSRX rightTalon;
  public static WPI_VictorSPX rightFrontVic;
  public static WPI_VictorSPX rightBackVic;
  public static MotorControllerGroup rightDrive;

  //DriveTrain
  DifferentialDrive testDrive;

  //Controller
  XboxController controller;

  public DriveBase_Subsystem() {
    //left motors
    leftTalon = new WPI_TalonSRX(Constants.leftTalonPort);
    leftFrontVic = new WPI_VictorSPX(Constants.leftFrontVicPort);
    leftBackVic = new WPI_VictorSPX(Constants.leftBackVicPort);
    leftDrive = new MotorControllerGroup(leftTalon, leftFrontVic, leftBackVic);



    //right motors
    rightTalon = new WPI_TalonSRX(Constants.rightTalonPort);
    rightFrontVic = new WPI_VictorSPX(Constants.rightFrontVicPort);
    rightBackVic = new WPI_VictorSPX(Constants.rightBackVicPort);
    rightDrive = new MotorControllerGroup(rightTalon, rightFrontVic, rightBackVic);

    //DriveTrain
    testDrive = new DifferentialDrive(leftDrive, rightDrive);

    //Controller
    controller = new XboxController(Constants.driverControllerPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arcadeDrive(controller);
  }

  public void arcadeDrive(XboxController controller) {
    //Gets controller values
    double speed = controller.getLeftX();
    double rotate = controller.getRightX();
    speed = adjust(speed);
    rotate = adjust(rotate);
    testDrive.curvatureDrive(-speed/Constants.driveSensitivity, -rotate/Constants.turnSensitivity, true);
  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double x) {
    if (Math.abs(x)<Constants.errormargin) {return 0.0;}
    if (Math.abs(x)>(1-Constants.errormargin)) {return (Math.abs(x)/x);}
    return x;
  }
}
