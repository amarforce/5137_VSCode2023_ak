// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Arm_Subsystem;
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
import frc.robot.commands.Arm_Commands.*;
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
  public static Arm_Subsystem arm_Subsystem;

  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

  //Triggers
  public static Trigger LTrigger;
  public static Trigger RTrigger;

  //Buttons
  public static JoystickButton StartButton;
  public static JoystickButton BackButton;
  public static JoystickButton XButton;
  public static JoystickButton AButton;
  public static JoystickButton YButton;
  public static JoystickButton BButton;
  public static POVButton DownDPad;
  public static POVButton UpDPad;

  public RobotContainer() {
    //Subsystems
    driveBase_Subsystem = new DriveBase_Subsystem();
    intake_Subystem = new Intake_Subystem(); 
    pneumatics_Subsystem = new Pneumatics_Subsystem();
    arm_Subsystem = new Arm_Subsystem();

    //Controllers
    driverController = new Joystick(Constants.driverControllerPort);
    assistController = new Joystick(Constants.assistControllerPort);

    configureBindings();    // Configures the trigger bindings
  }
  private void configureBindings() {
    //Intake 
    LTrigger = new Trigger(createBooleanSupplier(driverController, Constants.XBOX_LTriggerPort, Constants.XBOX_RTriggerPort, 0.1));
    LTrigger.whileTrue(new IntakeOnReverse());
    LTrigger.onFalse(new IntakeOff());

    RTrigger = new Trigger(createBooleanSupplier(driverController, Constants.XBOX_RTriggerPort, Constants.XBOX_LTriggerPort, 0.1));
    RTrigger.whileTrue(new IntakeOn());
    RTrigger.onFalse(new IntakeOff());

    //Compressor 
    StartButton = new JoystickButton(driverController, Constants.XBOX_StartPort);
    StartButton.onTrue(new CompressorOn());

    BackButton = new JoystickButton(driverController, Constants.XBOX_BackPort);
    BackButton.onTrue(new CompressorOff());

    //Arm
    XButton = new JoystickButton(assistController, Constants.XBOX_XPort);
    XButton.onTrue(new TopConePreset());

    AButton = new JoystickButton(assistController, Constants.XBOX_APort);
    AButton.onTrue(new MidConePreset());

    
    YButton = new JoystickButton(assistController, Constants.XBOX_YPort);
    YButton.onTrue(new TopCubePreset());

    BButton = new JoystickButton(assistController, Constants.XBOX_BPort);
    BButton.onTrue(new MidCubePreset());

    UpDPad = new POVButton(assistController, Constants.XBOX_UpDPad);
    UpDPad.onTrue(new ArmResetToIntake());

    DownDPad = new POVButton(assistController, Constants.XBOX_DownDPad);
    DownDPad.onTrue(new HybridPreset());
    
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
