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

    // Mystery Stuph Defining
    public Boolean UsePID = true;
    private double elevatorEncoder = 0.0;
    private int ELEVATOR_MOTOR_CURRENT_LIMIT = 30;

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
    
    // Sim variables
    public final Mechanism2d UppyDowny = new Mechanism2d(10, 10);
    public final MechanismRoot2d UppyDownyRoot = UppyDowny.getRoot("Elevator", 0, 0);

    //Physics
    //NOTE: I copied this because I don't have access to the robot. :]
    private final ElevatorSim elevSim = new ElevatorSim(DCMotor.getFalcon500(2), 12, 10, 0.05, 0.0, 3.0, true, 0.6);

    //Motor Config
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = ELEVATOR_GEARSTACK;

    MotionMagicConfigs motionMagic = cfg.MotionMagic;
    motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10))//1000
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(20))//2400
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(1000));//500000


    //PID Stuph
    private final PIDController elevaController = new PIDController(KP, KI, KD);
    
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS, KG, KV, KA);
    
    // Mystery Stuph
    public double getElevatorEncoder() {
         return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
     }

     //NOTE: I am not sure what this is but I think it is required for Lead and Follow to stay happy...
     //TODO: Figure out what this is...
     CurrentLimitsConfigs currentLimits = cfg.CurrentLimits;
     currentLimits.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;
     
     MotorOutputConfigs motorOutput = cfg.MotorOutput;
     motorOutput.NeutralMode = NeutralModeValue.Brake;

     StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
     for (int i = 0; i < 5; ++i) {
         leaderStatus = elevatorMotor.getConfigurator().apply(cfg);
         if (leaderStatus.isOK())
             break;
     }
     if (!leaderStatus.isOK()) {
         System.out.println("Could not configure Elevator Motor. Error: " + leaderStatus.toString());
     }

     StatusCode followerStatus = StatusCode.StatusCodeNotInitialized;
     for (int i = 0; i < 5; ++i) {
         followerStatus = elevatorFollowerMotor.getConfigurator().apply(cfg);
         if (followerStatus.isOK())
             break;
     }
     if (!followerStatus.isOK()) {
         System.out.println("Could not configure Elevator Leader Motor. Error: " + followerStatus.toString());
     }

     elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
 }

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
        elevatorEncoder = elevatorLead.getRotorPosition().getValueAsDouble();
        smartdashboard.putNumber("Elevator Encoder", elevatorEncoder());

        if (Utils.isSimulation()) {

        } else {

        }

    }
}
