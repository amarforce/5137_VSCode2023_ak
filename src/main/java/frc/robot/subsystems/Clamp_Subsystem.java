package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clamp_Subsystem extends SubsystemBase {

    private boolean clamped;

    public Clamp_Subsystem() {
        clamped = false;
    }


    public void Clamp( ) {  
        Pneumatics_Subsystem.clampSolenoid.set(true);
    }


    public void Release() {
        Pneumatics_Subsystem.clampSolenoid.set(false);
    }

    public boolean getClamped()
    {
        return clamped;
    }
}
