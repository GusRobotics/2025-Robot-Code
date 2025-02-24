package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;



public class Climber implements Subsystem {
    // Hardware
    private SparkMax leftClimberMotor;    
    private SparkMax rightClimberMotor; 

    // Init
    public Climber() {
        leftClimberMotor = new SparkMax(Constants.leftClimber, MotorType.kBrushless); 
        rightClimberMotor = new SparkMax(Constants.rightClimber, MotorType.kBrushless);
    }
    /** Sets the elevator's default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::end, this
        ));
    }

    public void enableClimber() {
        leftClimberMotor.set(-0.1);
        rightClimberMotor.set(0.1);
        Shooter.setLightstrip(Constants.blueLights);
    }

    public void stopClimber() {
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }

    public void reverseClimber() {
        leftClimberMotor.set(0.25);
        rightClimberMotor.set(-0.25);
    }

    /** Ends the elevator function */
    public void end() {
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);

    }

    @Override
    public void periodic() {

    }
}
