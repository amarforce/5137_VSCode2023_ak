package frc.robot.simulation;

import com.revrobotics.CANSparkMax;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class JMoneyEncoderTest {

    @Test
    public void testUpdate() {
        CANSparkMax motor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
        JMoneyEncoder encoder = new JMoneyEncoder(motor, 0.1);

        // Test initial position
        assertEquals(0.0, encoder.getPosition());

        // Test position after one update
        encoder.update();
        assertEquals(0.0, encoder.getPosition()); // Assuming motor.get() returns 0.0

        // Test position after multiple updates
        motor.set(1.0); // Set motor speed to 1.0
        encoder.update();
        assertEquals(0.02, encoder.getPosition());

        encoder.update();
        assertEquals(0.04, encoder.getPosition());

        // Test position after resetting
        encoder.reset();
        assertEquals(0.0, encoder.getPosition());
    }
}