package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Elevator implements Subsystem {
    // Hardware
    private SparkMax leftElevatorMotor;    
    private SparkMax rightElevatorMotor;

    // Encoders
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    // PID Controllers
    private final PIDController pidController;

    // Target position
    public static double targetPosition = 0.0;

    // PID Constants
    private static final double kP = 0.07; 
    private static final double kI = 0.0;
    private static final double kD = 0.004;

    // Position tolerance
    private static final double POSITION_TOLERANCE = 0.03; 
    private static final double VELOCITY_TOLERANCE = 0.05;

    // Speed limits
    private static final double MAX_UPWARD_SPEED = 0.4;
    private static final double MAX_DOWNWARD_SPEED = 0.25;
    private static final double MIN_SPEED = 0.03;

    // Ramp rate
    private static final double RAMP_RATE = 0.018; // Adjust this for smoother motion
    private double previousOutput = 0.0;

    public Elevator() {
        leftElevatorMotor = new SparkMax(Constants.leftElevator, MotorType.kBrushless); 
        rightElevatorMotor = new SparkMax(Constants.rightElevator, MotorType.kBrushless);

        // Get encoders
        leftEncoder = leftElevatorMotor.getEncoder();
        rightEncoder = rightElevatorMotor.getEncoder();

        // Initialize PID Controller
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        pidController.setIntegratorRange(-0.5, 0.5);

        zeroEncoders();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /** Sets the elevator's default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::end, this
        ));
    }

    private void setMotors(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    public void zeroEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        targetPosition = 0;
        pidController.reset();
    }

    public void enableElevator() {
        if (targetPosition < 25.45) { // Prevents it from going too high
            targetPosition += 0.3; 
        }
    }

    public void stopElevator() {
        setMotors(0);
        targetPosition = getCurrentPosition(); // Set target to current position to maintain position
    }

    public void reverseElevator() {
        if (targetPosition > 0.05) { // Prevents it from going too low
            targetPosition -= 0.3; 
        }
    }

    public double getMotorOutput() {
        return leftElevatorMotor.get();
    }

    public void setPosition(double position) {
        targetPosition = position;
        pidController.reset(); // Reset PID when changing targets
    }

    public boolean atTargetPosition() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        double currentPosition = getCurrentPosition();
        double output = pidController.calculate(currentPosition, targetPosition);
    
        // Determine clamping limits based on direction
        if (output > 0) { // Moving up
            output = MathUtil.clamp(output, -MAX_DOWNWARD_SPEED, MAX_UPWARD_SPEED);
        } else { // Moving down
            output = MathUtil.clamp(output, -MAX_DOWNWARD_SPEED, MAX_UPWARD_SPEED);
        }
    
        // Ensure minimum speed is applied if necessary
        if (Math.abs(targetPosition - currentPosition) > POSITION_TOLERANCE) {
            if (Math.abs(output) < MIN_SPEED) {
                output = Math.copySign(MIN_SPEED, output);
            }
        }

        // Ramp rate limiting
        output = rampRateLimit(output, previousOutput, RAMP_RATE);
        previousOutput = output;

        SmartDashboard.putNumber("Elevator Position", currentPosition);
        setMotors(output);
    }

    private double rampRateLimit(double currentOutput, double previousOutput, double rampRate) {
        // Limit how fast the motor output can change
        if (currentOutput > previousOutput + rampRate) {
            return previousOutput + rampRate;
        } else if (currentOutput < previousOutput - rampRate) {
            return previousOutput - rampRate;
        } else {
            return currentOutput;
        }
    }

    public double getCurrentPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    /** Ends the elevator function */
    public void end() {
        setMotors(0);
    }
}

// public class Elevator implements Subsystem {
//     // Hardware
//     private SparkMax leftElevatorMotor;    
//     private SparkMax rightElevatorMotor;

//     // Encoders
//     private RelativeEncoder leftEncoder;
//     private RelativeEncoder rightEncoder;

//     // PID Controllers
//     private final PIDController pidController;

//     // Target position
//     private double targetPosition = 0.0;

//     // PID Constants
//     private static final double kP = 0.1;
//     private static final double kI = 0.0;
//     private static final double kD = 0.0;

//     // Position tolerance
//     private static final double POSITION_TOLERANCE = 0.2; //was 0.5
//     private static final double VELOCITY_TOLERANCE = 0.1;

//     // Speed limits
//     private static final double MAX_UPWARD_SPEED = 0.35;
//     private static final double MAX_DOWNWARD_SPEED = 0.2;
//     private static final double MIN_SPEED = 0.05;

//     public Elevator() {
//         leftElevatorMotor = new SparkMax(Constants.leftElevator, MotorType.kBrushless); 
//         rightElevatorMotor = new SparkMax(Constants.rightElevator, MotorType.kBrushless);

//         // Get encoders
//         leftEncoder = leftElevatorMotor.getEncoder();
//         rightEncoder = rightElevatorMotor.getEncoder();

//         // Initialize PID Controller
//         pidController = new PIDController(kP, kI, kD);
//         pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
//         pidController.setIntegratorRange(-0.5, 0.5);

//         zeroEncoders();

//         CommandScheduler.getInstance().registerSubsystem(this);
//     }
//     /** Sets the elevator's default command (not moving) */
//     public void initDefaultCommand() {
//         setDefaultCommand(new InstantCommand(
//             this::end, this
//         ));
//     }

//     private void setMotors(double speed) {
//         leftElevatorMotor.set(speed);
//         rightElevatorMotor.set(speed);
//     }

//     public void zeroEncoders() {
//         leftEncoder.setPosition(0);
//         rightEncoder.setPosition(0);
//         targetPosition = 0;
//         pidController.reset();
//     }

//     public void enableElevator() {
//         if (targetPosition < 25.45) { // Prevents it from going too high
//             targetPosition += 0.3; 
//             //pidController.reset(); //test out if this is necessary
//         }
//     }

//     public void stopElevator() {
//         setMotors(0);
//         targetPosition = getCurrentPosition(); // Set target to current position to maintain position
//     }

//     public void reverseElevator() {
//         if (targetPosition > 0.05) { // Prevents it from going too low
//             targetPosition -= 0.3; 
//             //pidController.reset(); //test out if this is necessary
//         }
//     }

//     public double getMotorOutput() {
//         return leftElevatorMotor.get();
//     }

//     public void setPosition(double position) {
//         targetPosition = position;
//         pidController.reset(); // Reset PID when changing targets
//     }

//     public boolean atTargetPosition() {
//         return pidController.atSetpoint();
//     }

//     @Override
//     public void periodic() {
//         double currentPosition = getCurrentPosition();
//         double output = pidController.calculate(currentPosition, targetPosition);
    
//         // Determine clamping limits based on direction
//         if (output > 0) { // Moving up
//             output = MathUtil.clamp(output, -MAX_DOWNWARD_SPEED, MAX_UPWARD_SPEED);
//         } else { // Moving down
//             output = MathUtil.clamp(output, -MAX_DOWNWARD_SPEED, MAX_UPWARD_SPEED);
//         }
    
//         // Ensure minimum speed is applied if necessary
//         if (Math.abs(targetPosition - currentPosition) > POSITION_TOLERANCE) {
//             if (Math.abs(output) < MIN_SPEED) {
//                 output = Math.copySign(MIN_SPEED, output);
//             }
//         }
    
//         SmartDashboard.putNumber("Elevator Position", currentPosition);
//         setMotors(output);
//     }

//     public double getCurrentPosition() {
//         return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
//     }
    
//     /** Ends the elevator function */
//     public void end() {
//         setMotors(0);

//     }

    
// }