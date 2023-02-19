// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

//import java.util.ArrayList;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Arm_Subsystem;
//Subsystems
import frc.robot.subsystems.*;

//Commands
import frc.robot.commands.Intake_Commands.*;
import frc.robot.commands.Vision_Commands.AddVisionMeasurement;
import frc.robot.commands.Compressor_Commands.*;
import frc.robot.commands.Drive_Commands.AutoBalance;
import frc.robot.commands.Drive_Commands.AutoDrive;
import frc.robot.commands.Drive_Commands.AutoRotate;
import frc.robot.commands.Clamp_Commands.*;
import frc.robot.commands.Arm_Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final  Pneumatics_Subsystem pneumatics_Subsystem = new Pneumatics_Subsystem();
  public static final  DriveBase_Subsystem driveBase_Subsystem = new DriveBase_Subsystem();
  public static final  Intake_Subystem intake_Subystem = new Intake_Subystem(); 
  public static final  Clamp_Subsystem clamp_Subsystem = new Clamp_Subsystem();
  public static final  Arm_Subsystem arm_Subsystem = new Arm_Subsystem();
  public static final  Vision_Subsystem vision_Subsystem = new Vision_Subsystem();

  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;
  public static String controllerType;

  //Triggers
  public static Trigger driver_LTrigger;
  public static Trigger driver_RTrigger;
  public static Trigger assist_LTrigger;
  public static Trigger assist_RTrigger;

  //Buttons
  public static JoystickButton driver_StartButton; //Maybe switch these to assist
  public static JoystickButton driver_BackButton;
  public static JoystickButton driver_XButton;
  public static JoystickButton driver_AButton;
  public static JoystickButton driver_BButton;
  public static JoystickButton assist_XButton;
  public static JoystickButton assist_AButton;
  public static JoystickButton assist_YButton;
  public static JoystickButton assist_BButton;
  public static POVButton DownDPad;
  public static POVButton UpDPad;
  public static HashMap<String, Command> eventMap = new HashMap<>();
 

  //New chooser for the auto mode
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  

  public RobotContainer() {

    //Adds option for the autochooser
    autoChooser.setDefaultOption("score_chargeEngage", driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.score_chargeEngage));
    autoChooser.addOption("score_mobility_chargeEngage", driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_chargeEngage));
    autoChooser.addOption("score_mobility_intake_score", driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_intake_score));
    autoChooser.addOption("Goal_Path", driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.Goal_Path));



    //Command group for scoring, relies on the isFinished of each command
    SequentialCommandGroup score = new SequentialCommandGroup(new IntakeExtend(intake_Subystem),new ClampCone(clamp_Subsystem), new TopConePreset(arm_Subsystem) ,  new ClampOpen(clamp_Subsystem));
   
   
    //Adds commands to be used at event markers during auto path. Used as a parameter in autoBuilder
    eventMap.put("Intake1", new IntakeOn(intake_Subystem));
    eventMap.put("Score1", score);
    eventMap.put("Balance1", new AutoBalance(driveBase_Subsystem));
    
    //Controllers
    driverController = Robot.driverController;
    assistController = Robot.assistController;

    //Sets the defasult command to run continously for the vision subsystem
    vision_Subsystem.setDefaultCommand(new AddVisionMeasurement(driveBase_Subsystem, vision_Subsystem));
  }
  
  public void configureBindings() {

    //Align Testing 
    driver_XButton = new JoystickButton(driverController, Constants.d_XSquaredPort);
    driver_XButton.whileTrue(new AutoRotate(driveBase_Subsystem, Constants.pose2b));

    driver_BButton = new JoystickButton(driverController, Constants.d_BirclePort);
    driver_BButton.whileTrue(new AutoDrive(driveBase_Subsystem, Constants.pose2b));
    
    driver_AButton = new JoystickButton(driverController, Constants.d_AxePort);
    driver_AButton.whileTrue(new AutoBalance(driveBase_Subsystem));

    //Intake 
    driver_LTrigger = new Trigger(Supplier.createBooleanSupplier(driverController, Constants.d_LTriggerPort, Constants.d_RTriggerPort));
    driver_LTrigger.whileTrue(new IntakeOnReverse(intake_Subystem));
    driver_LTrigger.onFalse(new IntakeOff(intake_Subystem));

    driver_RTrigger = new Trigger(Supplier.createBooleanSupplier(driverController, Constants.d_RTriggerPort, Constants.d_LTriggerPort));
    driver_RTrigger.whileTrue(new IntakeOn(intake_Subystem));
    driver_RTrigger.onFalse(new IntakeOff(intake_Subystem));

    //Compressor 
    driver_StartButton = new JoystickButton(driverController, 9);
    driver_StartButton.onTrue(new CompressorOn(pneumatics_Subsystem));

    driver_BackButton = new JoystickButton(driverController, 10);
    driver_BackButton.onTrue(new CompressorOff(pneumatics_Subsystem));

    //Arm 
    assist_XButton = new JoystickButton(assistController, Constants.a_XSquaredPort);
    assist_XButton.onTrue(new TopConePreset(arm_Subsystem));

    assist_AButton = new JoystickButton(assistController, Constants.a_AxePort);
    assist_AButton.onTrue(new MidConePreset(arm_Subsystem));

    assist_YButton = new JoystickButton(assistController, Constants.g_Yangle);
    assist_YButton.onTrue(new TopCubePreset(arm_Subsystem));

    assist_BButton = new JoystickButton(assistController, Constants.a_BirclePort);
    assist_BButton.onTrue(new MidCubePreset(arm_Subsystem));

    UpDPad = new POVButton(assistController, Constants.UpDPad);
    UpDPad.onTrue(new ArmResetToIntake(arm_Subsystem));

    DownDPad = new POVButton(assistController, Constants.DownDPad);
    DownDPad.onTrue(new HybridPreset(arm_Subsystem));

    //Clamp 
    assist_LTrigger = new Trigger(Supplier.createBooleanSupplier(assistController, Constants.a_LTriggerPort, Constants.a_RTriggerPort));
    assist_LTrigger.onTrue(new ClampCube(clamp_Subsystem));
    assist_LTrigger.onFalse(new ClampOpen(clamp_Subsystem));

    assist_RTrigger = new Trigger(Supplier.createBooleanSupplier(assistController, Constants.a_RTriggerPort, Constants.a_LTriggerPort));
    assist_RTrigger.onTrue(new ClampCone(clamp_Subsystem));
    assist_RTrigger.onFalse(new ClampOpen(clamp_Subsystem));
  }

  public Command getAutoCommand(){
    return autoChooser.getSelected();
  }
}
