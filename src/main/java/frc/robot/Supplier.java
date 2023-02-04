package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import java.util.function.IntSupplier;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Supplier {
    
  public static IntSupplier DriverIS(int xbox, int ps4) {
    IntSupplier newIS;
    newIS = () -> {
      if (Robot.driverControllerType == "ps4") {
        return ps4;
      } else {
        return xbox;
      }
    };
    return newIS;
  }

  public static IntSupplier AssistIS(int xbox, int ps4) {
    IntSupplier newIS;
    newIS = () -> {
      if (Robot.assistControllerType == "ps4") {
        return ps4;
      } else {
        return xbox;
      }
    };
    return newIS;
  }

  public static DoubleSupplier DriverDS(double xbox, double ps4) {
    DoubleSupplier newIS;
    newIS = () -> {
      if (Robot.driverControllerType == "ps4") {
        return ps4;
      } else {
        return xbox;
      }
    };
    return newIS;
  }

  public static DoubleSupplier AssistDS(double xbox, double ps4) {
    DoubleSupplier newIS;
    newIS = () -> {
      if (Robot.assistControllerType == "ps4") {
        return ps4;
      } else {
        return xbox;
      }
    };
    return newIS;
  }

  public static BooleanSupplier createBooleanSupplier(Joystick controller, int requiredPort, int dependentPort) {
    BooleanSupplier booleanSupply;
    booleanSupply = () -> {
      if (controller != null) {
        if (controller.getRawAxis(requiredPort) > 0.1 && controller.getRawAxis(dependentPort) < 0.1) {
          return true;
        } else {
          return false;
        }
      } else {return false;}
    };
    return booleanSupply;
  }
}
