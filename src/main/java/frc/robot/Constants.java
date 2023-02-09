// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Driving
  public final static double driveSensitivity = 1.0;
  public final static double turnSensitivity = 3.0;
  public final static double errormargin = 0.1;

  //Controllers
  public final static int driverControllerPort = 0;
  public final static int assistControllerPort = 1;

  //Controller Ports
  public final static int g_LXStickAxisPort = 0;
  public final static int g_LYStickAxisPort = 1;
  public static int d_RXStickAxisPort;
  public static int a_RXStickAxisPort;
  public final static int g_RYStickAxisPort = 5;
  public static int d_LTriggerPort;
  public static int a_LTriggerPort;
  public static int d_RTriggerPort;
  public static int a_RTriggerPort;
  public static int d_ShackPort; //Shack =
  public static int a_ShackPort; //Share + Back
  public static int d_StoptionsPort; //Stoptions =
  public static int a_StoptionsPort; //Start + Options

  public static int d_AxePort;
  public static int a_AxePort;
  public static int d_BirclePort;
  public static int a_BirclePort;
  public static int d_XSquaredPort;
  public static int a_XSquaredPort;
  public final static int g_Yangle = 4;
  public final static int UpDPad = 0;
  public final static int DownDPad = 180;

  //DriveBase Motors - CAN Numbers
  public final static int leftFrontTalonPort = 1;
  public final static int leftBackTalonPort = 2;
  
  public final static int rightFrontTalonPort = 3;
  public final static int rightBackTalonPort = 4;

  //Intake
  public final static int intakePort = 5;
  public final static double intakeSpeed = 1.0;

  //Solenoids
  public static final int intakeSolChannel = 0;
  public static final int clampSolChannel = 1;
  public static final int feetSolChannel = 2;

  
  //Arm
  public final static int armRotatePort = 8; //filler number, change later
  public final static int armExtendPort = 9; //filler number, change later
  //rotationToDegreeConversion was originally 1, I think it should be 360 degrees per one rotation (possibly multiplied by gear ratio)
  public final static double rotationToDegreeConversion = 360; //filler number, will change when mechanical knows what theyre doing 

  public final static double armExtendSpeed = 0.2; //needs testing
  public final static double armRotateSpeed = 0.2; //needs testing

  //all of these are how many degrees the arm needs to turn to be in the optimal position
  //all of these values need testing to finalize
  public final static double topCubeRotation = 45;
  public final static double middleCubeRotation = 30;
  public final static double topConeRotation = 60;
  public final static double middleConeRotation = 50;
  public final static double hybridRotation = 20;

  public final static double topCubeExtension = 50;
  public final static double middleCubeExtension = 40;
  public final static double topConeExtension = 55;
  public final static double middleConeExtension = 45;
  public final static double hybridExtension = 30;



  //Vision Constants
  public final static double pi = Math.PI;
  public final static double nodeSpacing = Units.inchesToMeters(22);
  public final static double scoreDistance = Units.inchesToMeters(36);
  
  //change these later to put actual values when we know them
  public final static double CAMERA_HEIGHT_METERS = 1.27;
  public final static double TARGET_HEIGHT_METERS = 1.46;
  public final static double CAMERA_PITCH_RADIANS = 0;
  public final static double GOAL_RANGE_METERS = 0.5;//0.0254;//1 inch, can change later

  //pid for forward speed/vision
  public final static double dKP = 0.5;
  public final static double dKD = 0;
  public final static double dKI = 0;

  //pid for rotation speed/vision
  public final static double rKP = 0.5;
  public final static double rKD = 0;
  public final static double rKI = 0;

  //Initial robot values
  public final static Rotation2d initialGyro = new Rotation2d();
  public final static Pose2d initialPose = new Pose2d();
  public final static double initialLeftDistance = 0;
  public final static double initialRightDistance = 0;
  public final static DifferentialDriveKinematics trackWidth = new DifferentialDriveKinematics(Units.inchesToMeters(20.25));

  //TagField - tag4 & tag5 are the loading station targets

  //Red Alliance
  public final static Pose2d tag1 = new Pose2d(15.513558, 1.071626, new Rotation2d(pi));
  public final static Pose2d tag2 = new Pose2d(15.513558, 2.748026, new Rotation2d(pi));
  public final static Pose2d tag3 = new Pose2d(15.513558,4.424426, new Rotation2d(pi));
  public final static Pose2d tag4 = new Pose2d(16.178784, 6.749796, new Rotation2d(pi));

  //Blue Alliance 
  public final static Pose2d tag5 = new Pose2d(0.36195, 6.749796, new Rotation2d(0));
  public final static Pose2d tag6 = new Pose2d(1.02743, 4.424426, new Rotation2d(0));
  public final static Pose2d tag7 = new Pose2d(1.02743, 2.748026, new Rotation2d(0));
  public final static Pose2d tag8 = new Pose2d(1.02743, 1.071626, new Rotation2d(0));

  //AlignField 
  
  //Red Alliance Align Spots
  public final static Pose2d pose1a = new Pose2d(15.513558 - scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose1b = new Pose2d(15.513558 - scoreDistance, 1.071626, new Rotation2d(pi));
  public final static Pose2d pose1c = new Pose2d(15.513558 - scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose2a = new Pose2d(15.513558 - scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose2b = new Pose2d(15.513558 - scoreDistance, 2.748026, new Rotation2d(pi));
  public final static Pose2d pose2c = new Pose2d(15.513558 - scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose3a = new Pose2d(15.513558 - scoreDistance,4.424426 - nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose3b = new Pose2d(15.513558 - scoreDistance,4.424426, new Rotation2d(pi));
  public final static Pose2d pose3c = new Pose2d(15.513558 - scoreDistance,4.424426 + nodeSpacing, new Rotation2d(pi));
   
  //Blue Alliance align spots
  public final static Pose2d pose6a = new Pose2d(1.02743 + scoreDistance, 4.424426 + nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose6b = new Pose2d(1.02743 + scoreDistance, 4.424426, new Rotation2d(0));
  public final static Pose2d pose6c = new Pose2d(1.02743 + scoreDistance, 4.424426 - nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose7a = new Pose2d(1.02743 + scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose7b = new Pose2d(1.02743 + scoreDistance, 2.748026, new Rotation2d(0));
  public final static Pose2d pose7c = new Pose2d(1.02743 + scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose8a = new Pose2d(1.02743 + scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose8b = new Pose2d(1.02743 + scoreDistance, 1.071626, new Rotation2d(0));
  public final static Pose2d pose8c = new Pose2d(1.02743 + scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(0));

  //It's a lot more simple than it seems
  public static void updateDepConstants() {
    d_RXStickAxisPort = Supplier.DriverIS(4, 2).getAsInt();
    a_RXStickAxisPort = Supplier.AssistIS(4, 2).getAsInt();

    d_LTriggerPort = Supplier.DriverIS(2, 3).getAsInt();
    a_LTriggerPort = Supplier.AssistIS(2, 3).getAsInt();

    d_RTriggerPort = Supplier.DriverIS(3, 4).getAsInt();
    a_RTriggerPort = Supplier.AssistIS(3, 4).getAsInt();

    d_AxePort = Supplier.DriverIS(1, 2).getAsInt();
    a_AxePort = Supplier.AssistIS(1, 2).getAsInt();

    d_BirclePort = Supplier.DriverIS(2, 3).getAsInt();
    a_BirclePort = Supplier.AssistIS(2, 3).getAsInt();

    d_XSquaredPort = Supplier.DriverIS(3, 1).getAsInt();
    a_XSquaredPort = Supplier.AssistIS(3, 1).getAsInt();

    d_ShackPort = Supplier.DriverIS(7, 9).getAsInt(); //Shack =
    a_ShackPort = Supplier.AssistIS(7, 9).getAsInt(); //Share + Back

    d_StoptionsPort = Supplier.DriverIS(8, 10).getAsInt(); //Stoptions =
    a_StoptionsPort = Supplier.AssistIS(8, 10).getAsInt(); //Start + Options
  }
}
