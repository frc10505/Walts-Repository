package frc.team10505.robot.subsystems;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.LaserCan;

public class ElevatorSubsystem extends SubsystemBase {

    // Constants
    public static final int kElevatorLeadID = 1;
    public static final int kElevatorFollowID = 2;
    public static final double ELEVATOR_GEARSTACK = 12;

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
    
    //Sim Stuph
    private final Mechanism2d UppyDownyHome = new Mechanism2d(10, 10);

    // Sim variables
    private MechanismLigament2d UppyDowny = new MechanismLigament2d("Elevator", KA, ELEVATOR_MOTOR_CURRENT_LIMIT, ELEVATOR_GEARSTACK, null);
    private MechanismRoot2d UppyDownyRoot = UppyDownyHome.getRoot("ElevatorRoot", 0, 0);

    //Physics
    //NOTE: I copied this because I don't have access to the robot. :]
    private final ElevatorSim elevSim = new ElevatorSim(DCMotor.getFalcon500(2), 12, 10, 0.05, 0.0, 3.0, true, 0.6);



    //PID Stuph
    private final PIDController elevaController = new PIDController(KP, KI, KD);
    
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS, KG, KV, KA);
    
    // Mystery Stuph
    public double getElevatorEncoder() {
         return (elevatorLead.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
     }

     //NOTE: I am not sure what this is but I think it is required for Lead and Follow to stay happy...
     //TODO: Figure out what this is...
    

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

            
    //Motor Config
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    FeedbackConfigs fdb = cfg.Feedback;
    //fdb.SensorToMechanismRatio = ELEVATOR_GEARSTACK;

    MotionMagicConfigs motionMagic = cfg.MotionMagic;
    motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10))//1000
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(20));//2400

            CurrentLimitsConfigs currentLimits = cfg.CurrentLimits;
            currentLimits.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;
            
            MotorOutputConfigs motorOutput = cfg.MotorOutput;
            motorOutput.NeutralMode = NeutralModeValue.Brake;
       
            StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                leaderStatus = elevatorLead.getConfigurator().apply(cfg);
                if (leaderStatus.isOK())
                    break;
            }
       
            if (!leaderStatus.isOK()) {
                System.out.println("Could not configure Elevator Motor. Error: " + leaderStatus.toString());
            }
       
            StatusCode followerStatus = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                followerStatus = elevatorFollow.getConfigurator().apply(cfg);
                if (followerStatus.isOK())
                    break;
            }
            if (!followerStatus.isOK()) {
                System.out.println("Could not configure Elevator Leader Motor. Error: " + followerStatus.toString());
            }
       
            elevatorFollow.setControl(new Follower(elevatorFollow.getDeviceID(), false));
        }{
            if (Utils.isSimulation()) {
                elevatorLead = new TalonFX(kElevatorLeadID);
                elevatorFollow =new TalonFX(kElevatorFollowID);
        } else {
            elevatorLead = new TalonFX(kElevatorLeadID);
            elevatorFollow = new TalonFX(kElevatorFollowID);}
        }
    

    // periodic
    @Override
    public void periodic() {
        elevatorEncoder = elevatorLead.getRotorPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder);

        if (Utils.isSimulation()) {

        } else {

        }

    }
}
