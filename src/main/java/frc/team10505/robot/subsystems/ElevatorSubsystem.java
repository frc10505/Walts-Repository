package frc.team10505.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Constants
    public static final int kElevatorLeadID = 1;
    public static final int kElevatorFollowID = 2;

    // Mystery Stuph
    public Boolean UsePID = true;

    // PID Constants
    public static final double KP = .3;
    public static final double KI = .0;
    public static final double KD = .0;

    public static final double KS = .3;
    public static final double KG = .0;
    public static final double KV = .0;
    public static final double KA = .0;

    // Variables
    private  TalonFX elevatorLead = new TalonFX(kElevatorLeadID);
    private TalonFX elevatorFollow= new TalonFX(kElevatorFollowID);
        private double height = 0.0;
    
        // PID and Feedforward Witchcraft
        // public static final double KP = .3;
        // public static final double KI = .0;
        // public static final double KD = .0;
    
        // public static final double KS = .3;
        // public static final double KG = .0;
        // public static final double KV = .0;
        // public static final double KA = .0;
    
        private final PIDController elevaController = new PIDController(KP, KI, KD);
    
        private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS, KG, KV, KA);
    
        // Commands
        public Command setheight(double newHeight) {
            return runOnce(() -> {
                height = newHeight;
            });
        }
    
        public Command setVoltage(double voltage) {
            return runEnd(() -> {
                UsePID = false;
                elevatorLead.setVoltage(voltage);
            },
                    () -> {
                        UsePID = true;
                        elevatorLead.setVoltage(0);
                    });
        }
    
        // Constructor
        public ElevatorSubsystem() {
            if (Utils.isSimulation()) {
                elevatorLead = new TalonFX(kElevatorLeadID);
                elevatorFollow =new TalonFX(kElevatorFollowID);
        } else {
            elevatorLead = new TalonFX(kElevatorLeadID);
            elevatorFollow = new TalonFX(kElevatorFollowID);
        }
    }

    // periodic
    @Override
    public void periodic() {

    }
}
