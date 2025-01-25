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
    private static Spark lightstrip = new Spark(1);

    private AnalogInput distSensor = new AnalogInput(6);

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

    public boolean coralDetected(){
        return distSensor.getValue() > 700;
    }

    public void enableShooter() {
        SmartDashboard.putNumber("Distance Sensor", distSensor.getValue());
        
        if(coralDetected()){
            leftShooterMotor.set(0);
            rightShooterMotor.set(0);
            lightstrip.set(Constants.blueLights);
        }
        else {
            leftShooterMotor.set(0.1);
            rightShooterMotor.set(-0.1);
            lightstrip.set(Constants.yellowLights);
        }
    }

    public void stopShooter() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    }

    public void reverseShooter() {
        leftShooterMotor.set(-0.1);
        rightShooterMotor.set(0.1);
    }

    public void enableAutoShooter(){
        leftShooterMotor.set(0.1);
        rightShooterMotor.set(-0.1);
    }

    // public void forewardShooterState(){
    //     topShooterMotor.set(Constants.topIntakeSpeed);
    //     bottomShooterMotor.set(Constants.bottomIntakeSpeed);
    // }

    /** Ends the elevator function */
    public void end() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
        lightstrip.set(Constants.pinkLights);
    }

    public void setDefaultLights(){
        lightstrip.set(Constants.pinkLights);
    }

    public static void setLightstrip(double value) {
        lightstrip.set(value);
    }

    @Override
    public void periodic() {

    }
}
