// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Subsystems
import frc.robot.subsystems.DriveBase_Subsystem;
import frc.robot.subsystems.Intake_Subystem;
import frc.robot.subsystems.Pneumatics_Subsystem;

//Intake Commands
import frc.robot.commands.Intake_Commands.IntakeOff;
import frc.robot.commands.Intake_Commands.IntakeOn;
import frc.robot.commands.Intake_Commands.IntakeOnReverse;

//Compressor Commands 
import frc.robot.commands.Compressor_Commands.CompressorOn;
import frc.robot.commands.Compressor_Commands.CompressorOff;

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

  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

  //Triggers
  public static Trigger LTrigger;
  public static Trigger RTrigger;

  //Buttons
  public static JoystickButton StartButton;
  public static JoystickButton BackButton;

  //Boolean Suppliers
  public static BooleanSupplier booleanSupplyLT;
  public static BooleanSupplier booleanSupplyRT;

  public RobotContainer() {
    //Subsystems
    driveBase_Subsystem = new DriveBase_Subsystem();
    intake_Subystem = new Intake_Subystem(); 
    pneumatics_Subsystem = new Pneumatics_Subsystem();

    //Controllers
    driverController = new Joystick(Constants.driverControllerPort);
    assistController = new Joystick(Constants.assistControllerPort);

    configureBindings();    // Configures the trigger bindings
  }
  private void configureBindings() {
    configureBooleanSuppliers();

    //Intake 
    LTrigger = new Trigger(booleanSupplyLT);
    LTrigger.whileTrue(new IntakeOnReverse());
    LTrigger.onFalse(new IntakeOff());

    RTrigger = new Trigger(booleanSupplyRT);
    RTrigger.whileTrue(new IntakeOn());
    RTrigger.onFalse(new IntakeOff());

    //Compressor 
    StartButton = new JoystickButton(driverController, Constants.XBOX_StartPort);
    StartButton.onTrue(new CompressorOn());

    BackButton = new JoystickButton(driverController, Constants.XBOX_BackPort);
    BackButton.onTrue(new CompressorOff());


  }

  private void configureBooleanSuppliers() {
    booleanSupplyLT = () -> {
      if (driverController.getRawAxis(Constants.XBOX_LTriggerPort) > 0.1 && driverController.getRawAxis(Constants.XBOX_RTriggerPort) < 0.1) {
        return true;
      } else {
        return false;
      }
    };
    booleanSupplyRT = () -> {
      if (driverController.getRawAxis(Constants.XBOX_RTriggerPort) > 0.1 && driverController.getRawAxis(Constants.XBOX_LTriggerPort) < 0.1) {
        return true;
      } else {
        return false;
      }
    };
  }
}
