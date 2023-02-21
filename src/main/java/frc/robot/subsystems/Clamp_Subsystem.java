package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clamp_Subsystem extends SubsystemBase {

    private boolean clamped;

    public Clamp_Subsystem() {
        clamped = false;
    }


    public void Clamp(boolean type) {
        if (type) {
            Pneumatics_Subsystem.clampSolenoid.set(true);
            clamped = true;
        } else {
            //Need to make cube not die
            Pneumatics_Subsystem.clampSolenoid.set(true);
            clamped = false;
        }
    }

    public void Release() {
        Pneumatics_Subsystem.clampSolenoid.set(false);
    }

    public boolean getClamped()
    {
        return clamped;
    }
}
