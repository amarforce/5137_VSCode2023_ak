/*
The selected code is a Java class named JMoneyEncoder that simulates an encoder measuring the displacement of a motor. This class is part of a package named frc.robot.simulation and it uses the CANSparkMax class from the com.revrobotics package.

The JMoneyEncoder class has three instance variables:

m_motor of type CANSparkMax, which represents the motor that the encoder is linked to.
distance, a Double that keeps track of the total displacement of the motor.
rotationPerUpdate, a Double that represents the amount of rotation per update.
The class has a constructor that takes a CANSparkMax motor and a Double timePerRotation as parameters. The constructor initializes m_motor with the provided motor, distance with 0.0, and rotationPerUpdate with the value of 0.02/timePerRotation.

There are three methods in this class:

update(): This method updates the distance by adding the product of the current motor speed (m_motor.get()) and rotationPerUpdate.
getPosition(): This method returns the current distance, which represents the total displacement of the motor.
reset(): This method resets the distance to 0.0.
This class is part of a Gradle project, which is a build automation tool used primarily for Java projects. Gradle helps manage dependencies, run tests, create documentation, and perform many other tasks.
*/

/**
 * The JMoneyEncoder class simulates an encoder measuring the displacement of a motor.
 * This class is intended for temporary use and was created for fun.
 * -- JMoney
 */


package frc.robot.simulation;

import com.revrobotics.CANSparkMax;


public class JMoneyEncoder {
    public CANSparkMax m_motor; 
    private Double distance;
    private Double rotationPerUpdate;

    /**
     * The epic JMoneyEncoder that simulates an encoder measuring DISPLACEMENT of the motor.
     * This is only temporary I just wanted to do this for fun.
     *  -- JMoney
     * @param motor The motor you want to link the epic JMoneyEncoder too. ONLY CANSparkMax
     * @param timePerRotation The amount of time in seconds until the motor does a full rotation.
    */


    public JMoneyEncoder(CANSparkMax motor, Double timePerRotation) {
        m_motor = motor;
        distance = 0.0;
        rotationPerUpdate = 0.02/timePerRotation;
    }

    public void update() {
        distance += m_motor.get()*rotationPerUpdate;
    }
    
    public double getPosition() {
        return distance;
    }

    public void reset() {
        distance = 0.0;
    }
}