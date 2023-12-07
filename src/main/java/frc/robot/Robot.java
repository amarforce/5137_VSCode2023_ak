// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * GitHub Copilot: The provided code is a part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. The code is part of a class named `Robot` which extends `TimedRobot`, a class provided by WPILib.

Here's a breakdown of the code:

- The `package frc.robot;` statement indicates that this code is part of the `frc.robot` package.

- The `import` statements are used to include various classes needed for the program. These include classes from the `edu.wpi.first.wpilibj` package (part of WPILib), the `edu.wpi.first.wpilibj2.command` package (also part of WPILib), and a custom command `UpdateControllerPorts` from the `frc.robot.commands.ControlCommands` package.

- The `Robot` class extends `TimedRobot`, which is a base class for a robot program that is called in a timed loop (or "periodic") from the WPILib framework.

- Inside the `Robot` class, there are two private member variables: `m_robotContainer` of type `RobotContainer` and `m_autonomousCommand` of type `Command`. The `m_robotContainer` is likely used to manage the overall state of the robot, while `m_autonomousCommand` is likely used to store the command that will be run during the autonomous period of a match.

The comment above the `Robot` class explains that the VM (Virtual Machine) is configured to automatically run this class and call functions corresponding to each mode (like autonomous, teleoperated, etc.), as described in the `TimedRobot` documentation. If the name of this class or the package is changed after creating the project, the `build.gradle` file in the project must also be updated to reflect these changes.

This code is part of a Gradle project, which is a build automation tool used primarily for Java projects. Gradle helps manage dependencies, run tests, create documentation, and perform many other tasks. The cursor being on line 6 doesn't provide any specific context or information related to the code. It's just an empty line in the current code snippet.
 * 
 * 
 * This program is a Java application for controlling a robot using the WPILib (FIRST Robotics) framework. Here's an explanation of its structure and functionality:

Overview
- **License**: The program is open-source, licensed under the WPILib BSD license, allowing modification and sharing.
- **Package**: `frc.robot` - This is the package name, which is a standard convention in FIRST robotics programs.

Class: `Robot`
- **Extends**: `TimedRobot` - This class is a type of robot program framework provided by WPILib, which handles the robot's lifecycle in timed intervals (usually 20 ms).
- **Purpose**: To define the robot's behavior in different modes: initialization, periodic updates, autonomous, teleoperated, test, and simulation.

Key Components and Methods
- **`RobotContainer m_robotContainer`**: An object to manage command bindings and robot subsystems.
- **`Command m_autonomousCommand`**: Stores the command to be executed in autonomous mode.

Methods
- **`robotInit()`**: Called once when the robot is powered on. It initializes `m_robotContainer`.
- **`robotPeriodic()`**: Called every 20 ms. It runs the `CommandScheduler` to handle command execution and scheduling.
- **`disabledInit()` and `disabledPeriodic()`**: Called when the robot is disabled.
- **`autonomousInit()`**: Prepares and schedules the autonomous command. It fetches the command from `m_robotContainer`.
- **`autonomousPeriodic()`**: Called periodically during the autonomous mode but doesn't have custom code here.
- **`teleopInit()`**: Initializes teleoperated mode. It cancels the autonomous command, updates controller ports, and configures bindings.
- **`teleopPeriodic()`**: Called periodically during teleoperated mode. No custom code is added here.
- **`testInit()` and `testPeriodic()`**: Handle the test mode, canceling all commands during test initialization.
- **`simulationInit()` and `simulationPeriodic()`**: Used for simulation purposes, these methods are called during the robot simulation mode.

Additional Notes
- The VM (Virtual Machine) configuration note indicates that the class is set up to be automatically run and to handle various modes as per the TimedRobot specifications.
- The `build.gradle` file needs to be updated if there are changes to the class name or package.
- This framework follows a command-based design pattern, which separates the robot operations into different commands and subsystems for modularity and easier management.

Overall, this program is a skeleton for a typical FIRST robotics competition robot, providing the structure to add specific commands, subsystems, and control logic.
 * 
 */


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ControlCommands.UpdateControllerPorts;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer; 
  private Command m_autonomousCommand; //Will be used to run auto

 
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  public void autonomousInit()
  {
    //We should add a command so it will score and then run the path
    m_autonomousCommand = m_robotContainer.getAutoCommand(); //Gets the auto command selected 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(); //Runs auto 
    }
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.setLedMode();
    
    //Stops auto command when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Command updateControls = new UpdateControllerPorts();
    updateControls.schedule();
    m_robotContainer.configureBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
