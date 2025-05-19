package frc.team10505.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.LaserCan;

public class CoralSubsystem extends SubsystemBase {

    //Konstant
    private static final int kFlywheelLeft = 1;
    private static final int kFlywheelRight = 2;

    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 10;

    //Mystery Stuph Defining
    private int kFlywheelMotorCurrentLimit = 30;
    //private double simMotorSpeed = 0.0;
    private SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeRightConfig = new SparkMaxConfig();
public double coralEncoderR = 0.0;
public double coralEncoderL = 0.0;


    //Variables
    private SparkMax flywheelRight = new SparkMax(kFlywheelLeft, MotorType.kBrushless);
    private SparkMax flywheelLeft = new SparkMax(kFlywheelRight, MotorType.kBrushless);

    //Sim Stuph
    private final Mechanism2d flywheelHome = new Mechanism2d(5, 5);

    //Sim Variables
    private MechanismLigament2d flywheelLeftVis = new MechanismLigament2d("Left Wheel", kFlywheelRight, kFlywheelMotorCurrentLimit, kFlywheelLeft, null);
    private MechanismLigament2d flywheelRightVis = new MechanismLigament2d("Right Wheel", kFlywheelLeft, kFlywheelMotorCurrentLimit, kFlywheelRight, null);

    //The Goods...
        public double getCoralEncoderL() {
                return (flywheelLeft.getAbsoluteEncoder().getPosition() * ( Math.PI * 1.751 * 2) / 12.0) * 1.0;
        }

    private final FlywheelSim intakeLeftSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1)); //All the numbers (meaning the 0.005 and 5) have been burgled from Motion Magic and Sim

    private final FlywheelSim intakeRightSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 5), DCMotor.getNEO(1)); //All the numbers (meaning the 0.005 and 5) have been burgled from Motion Magic and Sim

    private final MechanismRoot2d intakeLeftRoot = flywheelHome.getRoot("Intake Left Root", .6, 0.6); //Cords have been burgled too :)
    private final MechanismRoot2d intakeRightRoot = flywheelHome.getRoot("Intake Right Root", 2.4, 0.6); //Cords have been burgled too :)

    private final MechanismLigament2d leftIntakeViz = intakeLeftRoot
            .append(new MechanismLigament2d("leftIntakeLigament", 0.4, 000)); //All the numbers (meaning the 0.4 and 000) have been burgled from Motion Magic and Sim
    private final MechanismLigament2d rightIntakeViz = intakeRightRoot
            .append(new MechanismLigament2d("rightIntakeLigament", 0.4, 180)); //All the numbers (meaning the 0.4 and 180) have been burgled from Motion Magic and Sim

    private double simMotorSpeed = 0.0;
    private double simMotorSpeed2 = 0.0;

    //Commands
    public Command setFlywheelSpeed(double kflywheelSpeed) {
        if (Utils.isSimulation()) {
                return runOnce(() -> {
                        simMotorSpeed = kflywheelSpeed;
                });
        } else {
                return runOnce(() -> {
                        flywheelLeft.set(kflywheelSpeed);
                        flywheelRight.set(kflywheelSpeed);
                }
                );

        }
}     
       
    
    

    //Constructor
    public CoralSubsystem() {
      //Intake Config
      intakeLeftConfig.idleMode(IdleMode.kBrake);
      intakeRightConfig.idleMode(IdleMode.kBrake);
      intakeLeftConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
      intakeRightConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
      flywheelLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic (){

        if (Utils.isSimulation()) {
var leftCurrentPosition = flywheelLeftVis.getAngle();
var rightCurrentPosition = flywheelRightVis.getAngle();

SmartDashboard.putNumber("Sim motor speed", simMotorSpeed);

intakeLeftSim.setInput(simMotorSpeed);
            intakeLeftSim.update(0.001);

        flywheelLeftVis.setAngle(leftCurrentPosition + (intakeLeftSim.getAngularVelocityRPM() * 0.04));
            flywheelLeftVis.setAngle(rightCurrentPosition - (intakeRightSim.getAngularVelocityRPM() * 0.04));
        } else {

        }
    }
}
    

