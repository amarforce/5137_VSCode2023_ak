// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Subsystems
import frc.robot.subsystems.*;

//Commands
import frc.robot.commands.Intake_Commands.*;
import frc.robot.commands.Compressor_Commands.*;
import frc.robot.commands.Clamp_Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  public static DriveBase_Subsystem driveBase_Subsystem;
  public static Intake_Subystem intake_Subystem;
  public static Pneumatics_Subsystem pneumatics_Subsystem;
  public static Clamp_Subsystem clamp_Subsystem;

  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

  //Triggers
  public static Trigger driver_LTrigger;
  public static Trigger driver_RTrigger;
  public static Trigger assist_LTrigger;
  public static Trigger assist_RTrigger;

  //Buttons
  public static JoystickButton driver_StartButton; //Maybe switch these to assist
  public static JoystickButton driver_BackButton;

  public RobotContainer() {
    //Subsystems
    driveBase_Subsystem = new DriveBase_Subsystem();
    intake_Subystem = new Intake_Subystem(); 
    pneumatics_Subsystem = new Pneumatics_Subsystem();
    clamp_Subsystem = new Clamp_Subsystem();

    //Controllers
    driverController = new Joystick(Constants.driverControllerPort);
    assistController = new Joystick(Constants.assistControllerPort);

    configureBindings();    // Configures the trigger bindings
  }

  private void configureBindings() {
    //Intake 
    driver_LTrigger = new Trigger(createBooleanSupplier(driverController, Constants.XBOX_LTriggerPort, Constants.XBOX_RTriggerPort, 0.1));
    driver_LTrigger.whileTrue(new IntakeOnReverse());
    driver_LTrigger.onFalse(new IntakeOff());

    driver_RTrigger = new Trigger(createBooleanSupplier(driverController, Constants.XBOX_RTriggerPort, Constants.XBOX_LTriggerPort, 0.1));
    driver_RTrigger.whileTrue(new IntakeOn());
    driver_RTrigger.onFalse(new IntakeOff());

    //Compressor 
    driver_StartButton = new JoystickButton(driverController, Constants.XBOX_StartPort);
    driver_StartButton.onTrue(new CompressorOn());

    driver_BackButton = new JoystickButton(driverController, Constants.XBOX_BackPort);
    driver_BackButton.onTrue(new CompressorOff());

    //Clamp
    assist_LTrigger = new Trigger(createBooleanSupplier(assistController, Constants.PS4_LTriggerPort, Constants.PS4_RTriggerPort, -0.9));
    assist_LTrigger.onTrue(new ClampCube());
    assist_LTrigger.onFalse(new ClampOpen());

    assist_RTrigger = new Trigger(createBooleanSupplier(assistController, Constants.PS4_RTriggerPort, Constants.PS4_LTriggerPort, -0.9));
    assist_RTrigger.onTrue(new ClampCone());
    assist_RTrigger.onFalse(new ClampOpen());
  }

  private BooleanSupplier createBooleanSupplier(Joystick controller, int requiredPort, int dependentPort, double requirement) {
    BooleanSupplier booleanSupply;
    booleanSupply = () -> {
      if (controller.getRawAxis(requiredPort) > requirement && controller.getRawAxis(dependentPort) < requirement) {
        return true;
      } else {
        return false;
      }
    };
    return booleanSupply;
  }
}
