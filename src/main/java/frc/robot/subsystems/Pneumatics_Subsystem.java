// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics_Subsystem extends SubsystemBase {
  public static Compressor comp;
  public static Solenoid intakeSolenoid;
  public static Solenoid clampSolenoid;
  public static Solenoid feetSolenoid;
 
  public Pneumatics_Subsystem() {
    comp = new Compressor(PneumaticsModuleType.CTREPCM);
    intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolChannel);
    clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.clampSolChannel);
    feetSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.feetSolChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void compress(boolean active) {
    if (active) {
      comp.enableDigital();
    } else {
      comp.disable();
    } 
  } 
}
