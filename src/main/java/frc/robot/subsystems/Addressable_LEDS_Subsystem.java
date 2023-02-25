// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Addressable_LEDS_Subsystem extends SubsystemBase {
  /** Creates a new Addressabe_LEDS_Subsystem. */

  private AddressableLED leds;
  private AddressableLEDBuffer ledBuffer;

  public Addressable_LEDS_Subsystem() {

    leds = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(300);
    leds.setLength(ledBuffer.getLength());
    alternateColors(Constants.cardinal, Constants.gold);

  }

  public void alternateColors(Color RGB1, Color RGB2){

    for(var i = 0; i < ledBuffer.getLength(); i++){
      if(i%2 != 0){//odd
        ledBuffer.setLED(i, RGB2);
      }
      else{//even
        ledBuffer.setLED(i, RGB1);
      } 
    }

    leds.setData(ledBuffer);
  }

  public void solidColor(Color RGB1){

    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setLED(i, RGB1);

    }

    leds.setData(ledBuffer);
}
      


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
