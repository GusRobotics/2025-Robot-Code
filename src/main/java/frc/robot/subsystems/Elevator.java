package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;


public class Elevator implements Subsystem {
    // Hardware
    private SparkMax leftElevatorMotor;    
    private SparkMax rightElevatorMotor; 

    // Init
    public Elevator() {
        leftElevatorMotor = new SparkMax(Constants.leftElevator, MotorType.kBrushless); 
        rightElevatorMotor = new SparkMax(Constants.rightElevator, MotorType.kBrushless);
    }
    /** Sets the elevator's default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::end, this
        ));
    }

    public void enableElevator() {
        leftElevatorMotor.set(0.1);
        rightElevatorMotor.set(0.1);
    }

    public void stopElevator() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }

    public void reverseElevator() {
        leftElevatorMotor.set(-0.1);
        rightElevatorMotor.set(-0.1);
    }

    public void enableAutoElevator(){
        leftElevatorMotor.set(0.1);
        rightElevatorMotor.set(0.1);
    }

    // public void forewardShooterState(){
    //     topShooterMotor.set(Constants.topIntakeSpeed);
    //     bottomShooterMotor.set(Constants.bottomIntakeSpeed);
    // }

    /** Ends the elevator function */
    public void end() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);

    }

    @Override
    public void periodic() {

    }
}