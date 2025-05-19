package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.CoralSubsystem;

public class RobotContainer {
    
    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController Operator = new CommandXboxController(1);
    private SendableChooser<Command> autonChooser;

    // Control Systems
    private final CoralSubsystem CoralSubsystem = new CoralSubsystem();
    private CommandJoystick joystick = new CommandJoystick(0);

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
        autonChooser = AutoBuilder.buildAutoChooser();
      configBindings();
      SmartDashboard.putData("autonChooser", autonChooser);
    }
}