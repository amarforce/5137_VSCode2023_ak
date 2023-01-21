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

//Intake Commands
import frc.robot.commands.Intake_Commands.ActivateIntake;
import frc.robot.commands.Intake_Commands.IntakeOff;
import frc.robot.commands.Intake_Commands.IntakeOn;
import frc.robot.commands.Intake_Commands.IntakeOnReverse;

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

  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

  //Triggers
  public static Trigger LTrigger;
  public static Trigger RTrigger;

  //Buttons
  public static JoystickButton AButton;

  //Boolean Suppliers
  public static BooleanSupplier booleanSupplyLT;
  public static BooleanSupplier booleanSupplyRT;

  public RobotContainer() {
    //Subsystems
    driveBase_Subsystem = new DriveBase_Subsystem();
    intake_Subystem = new Intake_Subystem();

    //Controllers
    driverController = new Joystick(Constants.driverControllerPort);
    assistController = new Joystick(Constants.assistControllerPort);

    // Configure the trigger bindings
    configureBindings();
  }
  private void configureBindings() {
    configureBooleanSuppliers();
    LTrigger = new Trigger(booleanSupplyLT);
    LTrigger.whileTrue(new IntakeOnReverse());
    LTrigger.onFalse(new IntakeOff());

    RTrigger = new Trigger(booleanSupplyRT);
    RTrigger.whileTrue(new IntakeOn());
    RTrigger.onFalse(new IntakeOff());

    AButton = new JoystickButton(driverController, Constants.PS4_SPort);
    AButton.onTrue(new ActivateIntake());
  }

  private void configureBooleanSuppliers() {
    booleanSupplyLT = () -> {
      if (driverController.getRawAxis(Constants.PS4_LTriggerPort) > -0.9 && driverController.getRawAxis(Constants.PS4_RTriggerPort) < -0.9) {
        return true;
      } else {
        return false;
      }
    };
    booleanSupplyRT = () -> {
      if (driverController.getRawAxis(Constants.PS4_RTriggerPort) > -0.9 && driverController.getRawAxis(Constants.PS4_LTriggerPort) < -0.9) {
        return true;
      } else {
        return false;
      }
    };
  }
}
