package frc.robot.commands.Drive_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;

public class DefaultDrive extends CommandBase {
    
    private DriveBase_Subsystem driveBase_Subsystem;
    private Joystick controller;
  
   
    public DefaultDrive(DriveBase_Subsystem driveBase_Subsystem, Joystick controller) {
      this.driveBase_Subsystem = driveBase_Subsystem;
      this.controller = controller;
      addRequirements(driveBase_Subsystem);
    }
  
    @Override
    public void execute() {
      driveBase_Subsystem.arcadeDrive(controller);
    }
  }
