package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.team10505.robot.subsystems.CoralSubsystem;

public class RobotContainer {
    
    priavte final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController Operator = new CommandXboxController(1);

    // Control Systems
    private final CoralSubsystemSubsystem CoralSubsystem = new CoralSubsystem();

    private void configBindings(){
    if (Utils.isSimulation()) {
        joystick.button(1).onTrue(CoralSubsystem.setFlywheelSpeed(100));
        joystick.button(2).onTrue(CoralSubsystem.setFlywheelSpeed(0));
        joystick.button(3).onTrue(CoralSubsystem.setFlywheelSpeed(-65));

    } else {
        joystick.button(1).onTrue(CoralSubsystem.setFlywheelSpeed(100));
        joystick.button(2).onTrue(CoralSubsystem.setFlywheelSpeed(0));
        joystick.button(3).onTrue(CoralSubsystem.setFlywheelSpeed(-65));
    }}

    public RobotContainer() {
        if (Utils.isSimulation()) {
            simConfigAlgaeButtonBindings();
        }else{
            configAlgaeButtonBindings();
        }
    }
}//Test
