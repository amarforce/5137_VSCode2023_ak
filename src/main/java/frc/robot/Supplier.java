
/*
 * The provided code is a part of a Java class named Supplier in the frc.robot package. This class seems to be used for managing different types of game controllers, specifically Xbox and PS4 controllers, in a robotics application.

The class imports several classes from the java.util.function package, namely IntSupplier, BooleanSupplier, and DoubleSupplier. These are functional interfaces in Java that represent suppliers of results. A Supplier<T> is a simple interface for a service that returns instances of type T. In this case, IntSupplier is a supplier of int-valued results, BooleanSupplier is a supplier of boolean-valued results, and DoubleSupplier is a supplier of double-valued results.

The Supplier class has two static String variables, driverControllerType and assistControllerType, which are used to store the type of the driver and assist controllers respectively. Currently, both are set to "xbox".

The DriverIS method is a public static method that returns an IntSupplier. It takes two integer parameters, xbox and ps4. Inside the method, an IntSupplier named newIS is declared. This IntSupplier is assigned a lambda function that checks if the driverControllerType is "ps4". If it is, the ps4 parameter is returned; otherwise, the xbox parameter is returned. This allows the method to dynamically supply different values based on the type of the driver controller.

The cursor being on line 9 doesn't provide any specific context or information related to the code. It's just an empty line in the current code snippet.

This code is part of a Gradle project, which is a build automation tool used primarily for Java projects. Gradle helps manage dependencies, run tests, create documentation, and perform many other tasks.
 * 
 * 
 * The `Supplier` class in the `frc.robot` package is designed to provide supplier functions for robot controllers in a FIRST Robotics competition setting. It utilizes Java's functional interfaces (`IntSupplier`, `DoubleSupplier`, `BooleanSupplier`) to create customizable control inputs for different types of game controllers (like Xbox and PS4 controllers). Here's a breakdown of its components and functionalities:

### Class: `Supplier`
- **Purpose**: To create supplier functions that adapt to different controller types and input requirements.

### Static Variables
- **`driverControllerType` and `assistControllerType`**: Strings that store the type of controller being used (e.g., "xbox" or "ps4"). These are used to determine which control scheme to use.

### Static Methods
1. **`DriverIS(int xbox, int ps4)`**: Returns an `IntSupplier` for the driver controller.
   - **Parameters**: `xbox` and `ps4` are integers representing control inputs for Xbox and PS4 controllers, respectively.
   - **Functionality**: The returned `IntSupplier` will provide either the `xbox` or `ps4` value depending on the `driverControllerType`.

2. **`AssistIS(int xbox, int ps4)`**: Similar to `DriverIS`, but for the assist controller.

3. **`DriverDS(double xbox, double ps4)`**: Returns a `DoubleSupplier` for the driver controller.
   - **Parameters**: `xbox` and `ps4` are doubles representing control inputs for Xbox and PS4 controllers.
   - **Functionality**: Chooses between the `xbox` or `ps4` value based on `driverControllerType`.

4. **`AssistDS(double xbox, double ps4)`**: Similar to `DriverDS`, but for the assist controller.

5. **`createBooleanSupplier(Joystick controller, int requiredPort, int dependentPort)`**: Creates a `BooleanSupplier` based on joystick inputs.
   - **Parameters**:
     - `controller`: The `Joystick` object being used.
     - `requiredPort` and `dependentPort`: Integers representing joystick ports.
   - **Functionality**: Returns `true` if the `requiredPort` axis value is greater than 0.1 and the `dependentPort` axis value is less than 0.1, indicating a specific joystick condition is met.

### Usage and Importance
- This class allows a robotics team to easily switch between different controller types without changing the main logic of their robot code.
- The use of Java functional interfaces (`IntSupplier`, `DoubleSupplier`, `BooleanSupplier`) allows for dynamic and flexible control schemes that can be passed around and used as needed in different parts of the robot code.
- The `createBooleanSupplier` method adds additional flexibility by allowing complex conditions to be defined for boolean controls based on joystick inputs.

In summary, the `Supplier` class provides a convenient and flexible way to define and manage different control schemes and inputs for a robot in a FIRST Robotics competition, accommodating various types of game controllers and input requirements.
 */


package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import java.util.function.IntSupplier;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Supplier {

  public static String driverControllerType = "xbox";
  public static String assistControllerType = "xbox";
    
  public static IntSupplier DriverIS(int xbox, int ps4) {
    IntSupplier newIS;
    newIS = () -> {
      if (driverControllerType == "ps4") {
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
      if (assistControllerType == "ps4") {
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
      if (driverControllerType == "ps4") {
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
      if (assistControllerType == "ps4") {
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
