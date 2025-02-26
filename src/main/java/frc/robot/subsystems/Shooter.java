package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter implements Subsystem {
    // Hardware
    private SparkMax leftShooterMotor;    
    private SparkMax rightShooterMotor; 
    private static Spark lightstrip = new Spark(Constants.ledChannel);

    private AnalogInput distSensor = new AnalogInput(0);

    // Init
    public Shooter() {
        leftShooterMotor = new SparkMax(Constants.leftShooter, MotorType.kBrushless); 
        rightShooterMotor = new SparkMax(Constants.rightShooter, MotorType.kBrushless);
    }
    /** Sets the elevator's default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::end, this
        ));
    }

    public void enableIndex() {
        SmartDashboard.putNumber("Distance Sensor", distSensor.getValue());
        
        if(distSensor.getValue() > 700){
            leftShooterMotor.set(0);
            rightShooterMotor.set(0);
            lightstrip.set(Constants.blueLights);
        }
        else {
            leftShooterMotor.set(0.15);
            rightShooterMotor.set(-0.15);
            lightstrip.set(Constants.pinkLights);
        }
    }

    public void enableShooter() {
        leftShooterMotor.set(0.26);
        rightShooterMotor.set(-0.26);
        lightstrip.set(Constants.greenLights);
    }

    public void stopShooter() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    } 

    public void reverseShooter() {
        leftShooterMotor.set(-0.1);
        rightShooterMotor.set(0.1);
    }

    /** Ends the shooter function */
    public void end() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
        lightstrip.set(Constants.yellowLights);
    }

    public static void setLightstrip( double value){
        lightstrip.set(value);
    }
    public void setDefaultLights(){
        lightstrip.set(Constants.yellowLights);
    }

    @Override
    public void periodic() {

    }
}
