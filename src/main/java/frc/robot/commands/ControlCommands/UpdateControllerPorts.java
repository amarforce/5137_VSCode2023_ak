package frc.robot.commands.ControlCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Supplier;


public class UpdateControllerPorts extends CommandBase{
    
    String defaultType;
    public UpdateControllerPorts()
    {
        defaultType = "xbox";
    }

    @Override
    public void execute()
    {
    if(RobotContainer.driverControlChooser.getSelected() != null && RobotContainer.assistControlChooser.getSelected() != null)
    {
        Supplier.driverControllerType = RobotContainer.driverControlChooser.getSelected();
        Supplier.assistControllerType = RobotContainer.assistControlChooser.getSelected();
    }
    else if(RobotContainer.driverControlChooser.getSelected() != null)
    {
        Supplier.driverControllerType = RobotContainer.driverControlChooser.getSelected();
        Supplier.assistControllerType = defaultType;
    }
    else if(RobotContainer.assistControlChooser.getSelected() != null)
    {
        Supplier.driverControllerType = defaultType;
        Supplier.assistControllerType = RobotContainer.assistControlChooser.getSelected();
    }
    else
    {
        Supplier.driverControllerType = defaultType;
        Supplier.assistControllerType = defaultType;
    }
    Constants.updateDepConstants();
    
}
    @Override
    public boolean isFinished()
    {
        if(Supplier.driverControllerType != null && Supplier.assistControllerType != null)
        {
            return true;
        }
        return false;
    }
    


}

