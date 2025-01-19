package frc.robot.subsystems;
import java.io.IOException;
import java.util.Arrays;
// import java.util.function.Consumer;
// import java.util.function.Supplier;

//got rid of that pigeon import bc  for some reason it wasn't getting the values of old class
//import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.ctre.phoenixpro.signals.StatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.pathplanner.lib.config.RobotConfig;
//import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import frc.robot.Constants.AutoConstants;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule blue;
    private final SwerveModule red;
    private final SwerveModule green;
    private final SwerveModule orange;
    private SlewRateLimiter xLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
    private SlewRateLimiter turningLimiter = new SlewRateLimiter(0.5);
    private final Pigeon2 pigeon = new Pigeon2(Constants.kPigeonPort);
    private final SwerveDriveOdometry odometer;
    private final SwerveModule[] modules = new SwerveModule[4];
    // toggling between SwerveModelState and SwerveModelPosition, attempting to
    // debug odometer
    SwerveModuleState driveStates[] = new SwerveModuleState[4];
    SwerveModulePosition drivePositions[] = new SwerveModulePosition[4];

    public SwerveDrive () {
        blue = new SwerveModule(
                Constants.blueDrive,
                Constants.blueSteer,
                Constants.kBlueDriveEncoderReversed,
                Constants.kBlueTurningEncoderReversed,
                Constants.kBlueDriveAbsoluteEncoderPort,
                Constants.kBlueDriveAbsoluteEncoderOffset,
                Constants.kBlueDriveAbsoluteEncoderReversed);

        // SparkBaseConfig blueMotorConfig = new SparkBaseConfig();
        // blueMotorConfig.inverted(true);  // Set to true for inverted, false for normal

        // // Apply the configuration to the motor controller
        // motor.configure(motorConfig, ConfigResetMode.kNoReset, ConfigPersistMode.kPersist);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        

        orange = new SwerveModule(
                Constants.orangeDrive,
                Constants.orangeSteer,
                Constants.kOrangeDriveEncoderReversed,
                Constants.kOrangeTurningEncoderReversed,
                Constants.kOrangeDriveAbsoluteEncoderPort,
                Constants.kOrangeDriveAbsoluteEncoderOffset,
                Constants.kOrangeDriveAbsoluteEncoderReversed);

        green = new SwerveModule(
                Constants.greenDrive,
                Constants.greenSteer,
                Constants.kGreenDriveEncoderReversed,
                Constants.kGreenTurningEncoderReversed,
                Constants.kGreenDriveAbsoluteEncoderPort,
                Constants.kGreenDriveAbsoluteEncoderOffset,
                Constants.kGreenDriveAbsoluteEncoderReversed);

        red = new SwerveModule(
                Constants.redDrive,
                Constants.redSteer,
                Constants.kRedDriveEncoderReversed,
                Constants.kRedTurningEncoderReversed,
                Constants.kRedDriveAbsoluteEncoderPort,
                Constants.kRedDriveAbsoluteEncoderOffset,
                Constants.kRedDriveAbsoluteEncoderReversed);

        modules[0] = blue;
        modules[1] = green;
        modules[2] = orange;
        modules[3] = red;
        odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), getPosition());

        ModuleConfig moduleConfig = new ModuleConfig(
            0.0508,                     // wheelRadiusMeters (0.0508m = 2 inch wheels)
            Constants.kPhysicalMaxSpeedMetersPerSecond,  // maxDriveVelocityMPS
            1.0,                        // wheelCOF (coefficient of friction, typically 0.5-1.0)
            DCMotor.getNEO(1),         // driveMotor (using a single NEO motor)
            8.14,                       // driveGearing (8.14:1 ratio, adjust for your gearbox)
            50.0,                       // driveCurrentLimit (amps)
            1                          // numMotors (1 drive motor per module)
        );
        RobotConfig config = new RobotConfig(
            54.0,  // Robot mass in kilograms
            10.0,  // Moment of inertia (kg * m^2)
            moduleConfig,
            // Module offsets from robot center
            new Translation2d(Constants.kTrackWidth / 2.0, Constants.kWheelBase / 2.0),    // Front Left
            new Translation2d(Constants.kTrackWidth / 2.0, -Constants.kWheelBase / 2.0),   // Front Right
            new Translation2d(-Constants.kTrackWidth / 2.0, Constants.kWheelBase / 2.0),   // Back Left
            new Translation2d(-Constants.kTrackWidth / 2.0, -Constants.kWheelBase / 2.0)   // Back Right
        );
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }


        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );

        // private final SwerveModule[] allModules = new SwerveModule[]{
        //     blue,
        //     orange,
        //     green,
        //     red
        // driveStates[0] = blue.getState();
        // driveStates[1] = orange.getState();
        // driveStates[2] = red.getState();
        // driveStates[3] = green.getState();
        
    //     AutoBuilder.configureRamsete(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getSpeeds,// Current ChassisSpeeds supplier
    //         this::driveRobot, // Method that will drive the robot given ChassisSpeeds
    //         new ReplanningConfig(), // Default path replanning config. See the API for the options here
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements

            
    // );

    //     AutoBuilder.configureHolonomic(
    //     this::getPose, // Robot pose supplier
    //     this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //     this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     this::driveRobot, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //         AutoConstants.KTranslationHolonomicPID, // Translation PID constants
    //         AutoConstants.KRotationHolonomicPID, // Rotation PID constants
    //         AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
    //         AutoConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //         new ReplanningConfig() // Default path replanning config. See the API for the options here
    //     ),
    //     () -> {
    //         var alliance = DriverStation.getAlliance();
    //         if (alliance.isPresent()){
    //             return alliance.get() == DriverStation.Alliance.Red;
    //         }
    //         return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );

        }


    public SwerveModulePosition[] positioning(SwerveModulePosition[] positions) {
        positions[0] = new SwerveModulePosition(0, new Rotation2d(Constants.kGreenDriveAbsoluteEncoderOffset));
        positions[1] = new SwerveModulePosition(0, new Rotation2d(Constants.kBlueDriveAbsoluteEncoderOffset));
        positions[2] = new SwerveModulePosition(0, new Rotation2d(Constants.kRedDriveAbsoluteEncoderOffset));
        positions[3] = new SwerveModulePosition(0, new Rotation2d(Constants.kOrangeDriveAbsoluteEncoderOffset));
        return positions;
    }

    private void driveRobot(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = Constants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }

    private ChassisSpeeds getSpeeds() {
        return Constants.kDriveKinematics.toChassisSpeeds(getModuleStates());
      }

    public void zeroHeading() {
        pigeon.reset();
    }

     private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = Arrays.stream(modules)
      .map(module -> module.getState())
      .toArray(size -> new SwerveModuleState[size]);
    return states;
    }

    private SwerveModulePosition[] getPosition(){
        drivePositions[0] = green.getPosition();
        drivePositions[1] = blue.getPosition();
        drivePositions[2] = red.getPosition();
        drivePositions[3] = orange.getPosition();
        return drivePositions;
    }

    public void resetPose(Pose2d pose){
        odometer.resetPosition(getRotation2d(), getPosition(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        setModuleStates(moduleStates);
    }


    public double getHeading() {
        return Math.IEEEremainder(pigeon.getYaw().getValue().magnitude(), 360);
    }

    public Rotation2d getRotation2d() {
        double numDegrees = -pigeon.getYaw().getValue().magnitude();
        return Rotation2d.fromDegrees(numDegrees);
    }

    public double getBlueTurnSetPoint(){
        return blue.getTurnSetPoint();
    }

    public double getRedPosition(){
        return red.getTurningPosition();
    }
    public double getOrangePosition(){
        return orange.getTurningPosition();
    }
    public double getGreenPosition(){
        return green.getTurningPosition();
    }
    public double getBluePosition(){
        return blue.getTurningPosition();
    }

    // public double getTurningVelocity() {
    //     return pigeon.getAngularVelocityXDevice().getValue();
    // }
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

     public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getPosition(), pose);
    }


    @Override
    public void periodic() {
        // note odometry settings commented out bc of swervedrivestate and
        // swervedriveposition
        odometer.update(getRotation2d(), getPosition());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location",(
        // getPose().getTranslation().toString());
        SmartDashboard.putNumber("odometry x", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("odometry y", odometer.getPoseMeters().getY());

        odometer.update(getRotation2d(), getPosition());
    
    // Gyro and overall robot data
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber("Pigeon Raw Yaw", pigeon.getYaw().getValue().magnitude());
    
    // Module states
    SmartDashboard.putNumber("Blue Turn Angle", blue.getTurningPosition());
    SmartDashboard.putNumber("Blue Drive Speed", blue.getDriveVelocity());
    SmartDashboard.putNumber("Blue CANCoder Raw", blue.getCancoder());
    
    SmartDashboard.putNumber("Red Turn Angle", red.getTurningPosition());
    SmartDashboard.putNumber("Red Drive Speed", red.getDriveVelocity());
    SmartDashboard.putNumber("Red CANCoder Raw", red.getCancoder());
    
    SmartDashboard.putNumber("Green Turn Angle", green.getTurningPosition());
    SmartDashboard.putNumber("Green Drive Speed", green.getDriveVelocity());
    SmartDashboard.putNumber("Green CANCoder Raw", green.getCancoder());
    
    SmartDashboard.putNumber("Orange Turn Angle", orange.getTurningPosition());
    SmartDashboard.putNumber("Orange Drive Speed", orange.getDriveVelocity());
    SmartDashboard.putNumber("Orange CANCoder Raw", orange.getCancoder());
    
    // Odometry
    SmartDashboard.putNumber("Robot X Position", getPose().getX());
    SmartDashboard.putNumber("Robot Y Position", getPose().getY());
    SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
    }

    public void stopModules() {
        blue.stop();
        green.stop();
        orange.stop();
        red.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        green.setDesiredState(desiredStates[0]);
        blue.setDesiredState(desiredStates[1]);
        red.setDesiredState(desiredStates[2]);
        orange.setDesiredState(desiredStates[3]);
    }

    // public void doChassisIdfk(){
    //     SwerveDrive swerveDrive;
    //     ChassisSpeeds chassisSpeeds;
    //     SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    //     swerveDrive.setModuleStates(moduleStates);
    // }
    
    public Rotation2d getRotation2D() {
        //gets pigeon value for rotation in degrees, converts to radians
        //double numDegrees = pigeon.getYaw().getValue();
        // StatusSignal<Angle> yawSignal = pigeon.getYaw();
        // double numDegrees = yawSignal.getAngle();
        //double numDegrees = pigeon.getYaw().refresh().getValue().getRadians();
        double numDegrees = pigeon.getYaw().getValue().magnitude();
        double radians = numDegrees * (Math.PI/180);
        return new Rotation2d(radians);
    }

    public void teleopControlSwerve(double leftX, double leftY, double rightX) {
        // value originally Math.atan(leftY/leftX)/(Math.PI *2), got rid of dividing by
        // pi
        Rotation2d desRot = new Rotation2d(Math.atan2(leftY, leftX));
        double velocity = leftY;

        // this works!!! time to scale the vectors!!
        // check if lefty is not 1 or -1 (full magnitudes)
        // SlewRateLimiter xLimiter = new SlewRateLimiter(0.5);
        // SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
        if (leftY < 0.05 && leftY > -0.05) {
            if (leftX >= 0.05) {
                desRot = new Rotation2d(90);
                leftX = xLimiter.calculate(leftX) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
            } else if (leftX <= -0.05) {
                desRot = new Rotation2d(-90);
            }
            velocity = leftX;
        }
        // try mult vs division
        leftY = yLimiter.calculate(leftX) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        velocity = leftY;

        SwerveModuleState desiredState = new SwerveModuleState(velocity, desRot);

        // leave this alone this is the only thing that works
        driveStates[0] = desiredState;
        driveStates[1] = desiredState;
        driveStates[2] = desiredState;
        driveStates[3] = desiredState;

        green.setDesiredState(desiredState);
        blue.setDesiredState(desiredState);
        red.setDesiredState(desiredState);
        orange.setDesiredState(desiredState);
    }

    public void execute(double leftX, double leftY, double rightX) {
        // 1. Get real-time joystick inputs
        double xSpeed = leftX*5;
        double ySpeed = leftY*5;
        double turningSpeed = rightX;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.OIConstants ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        // ensures field orientation
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, getRotation2D());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        this.setModuleStates(moduleStates);
    }

}