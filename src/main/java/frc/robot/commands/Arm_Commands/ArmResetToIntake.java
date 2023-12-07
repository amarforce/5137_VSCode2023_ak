// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * The provided code is part of a Java program that uses the WPILib library, a set of libraries used for programming robots in the FIRST Robotics Competition. 
 * The code is part of a class named ArmResetToIntake in the frc.robot.commands.Arm_Commands package.

The ArmResetToIntake class extends CommandBase, a class provided by WPILib. 
This class is used to create commands, which are reusable chunks of code that can be run in response to various events.

The ArmResetToIntake class has two instance variables: arm_Subsystem and intake_Subystem. 
These are of type Arm_Subsystem and Intake_Subystem respectively, which are likely custom classes that represent different subsystems of the robot.

The ArmResetToIntake class has a constructor that takes two parameters: an Arm_Subsystem and an Intake_Subystem. 
Inside the constructor, the instance variables arm_Subsystem and intake_Subystem are assigned the values of the parameters. 
This means that when a new ArmResetToIntake command is created, it needs to be given references to the Arm_Subsystem and Intake_Subystem that it will work with.

The constructor also calls addRequirements twice, once with arm_Subsystem and once with intake_Subystem. 
The addRequirements method is used to specify that this command requires these subsystems. 
This means that no other command that requires these subsystems can run at the same time as an ArmResetToIntake command.
 * 
 * The `ArmResetToIntake` class is a command within the WPILib Command-Based framework, specifically designed for a robotics application. It's part of the `frc.robot.commands.Arm_Commands` package and is responsible for resetting the position of an arm mechanism on the robot to a position suitable for intake operations. Let's break down its structure and functionalities:

### Class: `ArmResetToIntake`
- **Extends**: `CommandBase` - This indicates that `ArmResetToIntake` is a type of Command in the WPILib framework.
- **Purpose**: To move the robot's arm subsystem to a specific position that aligns with the intake subsystem.

### Member Variables
- **`arm_Subsystem`**: An instance of `Arm_Subsystem`, representing the robot's arm mechanism.
- **`intake_Subystem`**: An instance of `Intake_Subystem`, representing the robot's intake mechanism.

### Constructor: `ArmResetToIntake(Arm_Subsystem arm_Subsystem, Intake_Subystem intake_Subystem)`
- **Parameters**: Instances of `Arm_Subsystem` and `Intake_Subystem`.
- **Functionality**: Initializes the command with the given subsystems and declares subsystem dependencies using `addRequirements()`.

### Overridden Methods
1. **`initialize()`**: Called when the command is initially scheduled. It's empty in this implementation.
2. **`execute()`**: Called repeatedly while the command is active. It checks if the arm movement is clear (using `arm_Subsystem.armMovementClear(intake_Subystem)`) and then commands the arm subsystem to move to a specific position (defined in `Constants.armIntakeRotation` and `Constants.armIntakeExtention`).
3. **`end(boolean interrupted)`**: Called once the command ends or is interrupted. No specific implementation is provided here.
4. **`isFinished()`**: Returns `true` when the command should end. The end condition is based on whether the arm has reached the desired position (`arm_Subsystem.armFinished(Constants.armIntakeRotation, Constants.armIntakeExtention)`).

### Usage in a Robot Program
- **Command Lifecycle**: The command is scheduled by the robot's command scheduler. When scheduled, it initializes, then repeatedly executes its logic during each scheduler run, and finally ends either when interrupted or when its `isFinished` condition is met.
- **Subsystem Interaction**: The command interacts with the `Arm_Subsystem` and `Intake_Subystem`, coordinating their actions. This is crucial for ensuring that the robot's arm and intake systems work in sync.
- **Safety and Efficiency**: The command checks if the arm movement is clear before executing the move, which is important for safety and preventing mechanical conflicts.

In summary, the `ArmResetToIntake` command is a specific operation within a robot's control system, using the WPILib Command-Based framework to manage the arm's position in relation to the intake system, ensuring coordinated and safe robot operations.
 * 
 */


package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm_Subsystem;
import frc.robot.subsystems.Intake_Subystem;

public class ArmResetToIntake extends CommandBase {
  /** Creates a new ArmResetToIntake. */
  Arm_Subsystem arm_Subsystem;
  Intake_Subystem intake_Subystem;
  
  public ArmResetToIntake(Arm_Subsystem arm_Subsystem, Intake_Subystem intake_Subystem) {
    this.arm_Subsystem = arm_Subsystem;
    this.intake_Subystem = intake_Subystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm_Subsystem);
    addRequirements(intake_Subystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm_Subsystem.armMovementClear(intake_Subystem)){
      arm_Subsystem.moveArm(Constants.armIntakeRotation, Constants.armIntakeExtention);
      System.out.println("Reset");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm_Subsystem.armFinished(Constants.armIntakeRotation, Constants.armIntakeExtention);
  }
}
