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

  private AddressableLED leds1;
  private AddressableLEDBuffer ledBuffer1;

  public Addressable_LEDS_Subsystem() {

    //Creates new LED on port 9
    leds1 = new AddressableLED(Constants.ledString1Port);
    ledBuffer1 = new AddressableLEDBuffer(300);
    leds1.setLength(ledBuffer1.getLength());
  }

  public void alternateColors(Color RGB1, Color RGB2){

    //Sets Odd and Even Leds to alternating colors
    for(var i = 0; i < ledBuffer1.getLength(); i++){
      if(i%2 != 0){//odd
        ledBuffer1.setLED(i, RGB2);

      }
      else{//even
        ledBuffer1.setLED(i, RGB1);


      } 
      
    }
    //Sets the LED to the buffer sequence
    leds1.setData(ledBuffer1);
  }
    

  public void solidColor(Color RGB1){

    for(var i = 0; i < ledBuffer1.getLength(); i++){
      ledBuffer1.setLED(i, RGB1);

    }

    leds1.setData(ledBuffer1);

}
      


  @Override
  public void periodic() {

  }
}
