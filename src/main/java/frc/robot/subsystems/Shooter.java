package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotContainer;


public class Shooter implements Subsystem {
    // Hardware
    private SparkMax leftShooterMotor;    
    private SparkMax rightShooterMotor; 
    public static Spark lightstrip = new Spark(Constants.ledChannel);

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
        
        if (RobotContainer.isCoralIndexEnabled) {
            if (distSensor.getValue() > 700) {
                leftShooterMotor.set(0);
                rightShooterMotor.set(0);
                lightstrip.set(Constants.blueLights);
            } else {
                leftShooterMotor.set(0.15);
                rightShooterMotor.set(-0.15);
                lightstrip.set(Constants.pinkLights);
            }
        } else {
            // Indexing is disabled, stop the motors
            end();
        }
    }

    public void isSensorWorking() {
        boolean isWorking = distSensor.getValue() > 205 && distSensor.getValue() < 209; // Sensor is between 207 and 208 when not working
        SmartDashboard.putBoolean("Sensor Working", !isWorking);
    }
    
    public void enableShooter() {
        lightstrip.set(Constants.greenLights);
        if (Elevator.targetPosition < 15){
            leftShooterMotor.set(0.2);
            rightShooterMotor.set(-0.2);
        }
        else{
            leftShooterMotor.set(0.23);
            rightShooterMotor.set(-0.23);
        }
            
    }

    public void stopShooter() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    } 

    public void L1Shot() {
        leftShooterMotor.set(0.17);
        rightShooterMotor.set(-0.17);
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
        isSensorWorking();
    }
}
