package frc.team10505.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team10505.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
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

    //Konstants
    private static final int kFlywheelLeft = 1;
    private static final int kFlywheelRight = 2;

    //Mystery Stuph Defining
    private int kFlywheelMotorCurrentLimit = 30;


    //Variables
    private SparkMax flywheelRight = new SparkMax(kFlywheelLeft, MotorType.kBrushless);
    private SparkMax flywheelLeft = new SparkMax(kFlywheelRight, MotorType.kBrushless);

    //Sim Stuph
    private final Mechanism2d flywheelLeftHome = new Mechanism2d(5, 5);
    private final Mechanism2d flywheelRightHome = new Mechanism2d(5, 5);

    //Sim Variables
    private MechanismLigament2d flywheelLeftSim = new MechanismLigament2d("Left Wheel", kFlywheelRight, kFlywheelMotorCurrentLimit, kFlywheelLeft, null);
    private MechanismLigament2d flywheelRightSim = new MechanismLigament2d("Right Wheel", kFlywheelLeft, kFlywheelMotorCurrentLimit, kFlywheelRight, null);

    //Physics (The Dog)

    //Commands
    public Command setFlywheelSpeed(SparkMax kflywheelSpeed) {
        return runOnce(() -> {
        flywheelLeft = kflywheelSpeed;
        flywheelRight = kflywheelSpeed;
        });
    }

    //Constructors
    public CoralSubsystem() {
      
    }


    @Override
    public void periodic (){

        if (Utils.isSimulation()) {

        } else {

        }
    }
}
    

