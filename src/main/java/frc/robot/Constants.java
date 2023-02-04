// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static int g_Yangle = 4;
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
