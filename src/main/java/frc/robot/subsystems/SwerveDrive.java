package frc.robot.subsystems;
import java.util.Arrays;

//got rid of that pigeon import bc  for some reason it wasn't getting the values of old class
//import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
//import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
//import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import frc.robot.Constants.AutoConstants;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive instance;
    private final SwerveModule blue;
    private final SwerveModule red;
    private final SwerveModule green;
    private final SwerveModule orange;
    private final Timer timer = new Timer();
    private SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(1);
    private SlewRateLimiter turningLimiter = new SlewRateLimiter(1);
    private final Pigeon2 pigeon = new Pigeon2(Constants.kPigeonPort);
    private final SwerveDriveOdometry odometer;
    private final SwerveModule[] modules = new SwerveModule[4];
    // toggling between SwerveModelState and SwerveModelPosition, attempting to
    // debug odometer
    SwerveModuleState driveStates[] = new SwerveModuleState[4];
    SwerveModulePosition drivePositions[] = new SwerveModulePosition[4];

    public static SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }
        return instance;
    }

    public SwerveDrive () {
        blue = new SwerveModule(
                Constants.blueDrive,
                Constants.blueSteer,
                Constants.kBlueDriveEncoderReversed,
                Constants.kBlueTurningEncoderReversed,
                Constants.kBlueDriveAbsoluteEncoderPort,
                Constants.kBlueDriveAbsoluteEncoderOffset,
                Constants.kBlueDriveAbsoluteEncoderReversed);

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
            5.143,                       // driveGearing (8.14:1 ratio, adjust for your gearbox)
            45.0,                       // driveCurrentLimit (amps)
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
                        new PIDConstants(0, 0.0, 0), // Translation PID constants (5,0,0)
                        new PIDConstants(0, 0.0, 0) // Rotation PID constants (5,0,0)
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
    }


    public SwerveModulePosition[] positioning(SwerveModulePosition[] positions) {
        positions[0] = new SwerveModulePosition(0, new Rotation2d(Constants.kGreenDriveAbsoluteEncoderOffset));
        positions[1] = new SwerveModulePosition(0, new Rotation2d(Constants.kBlueDriveAbsoluteEncoderOffset));
        positions[2] = new SwerveModulePosition(0, new Rotation2d(Constants.kRedDriveAbsoluteEncoderOffset));
        positions[3] = new SwerveModulePosition(0, new Rotation2d(Constants.kOrangeDriveAbsoluteEncoderOffset));
        return positions;
    }

    public void drive(ChassisSpeeds speeds) {
        driveRobotRelative(speeds);
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
        // Ensure the odometer has been initialized
        if (odometer == null) {
            System.err.println("Odometer is not initialized.");
            return;  // Early exit if odometer is not initialized
        }
    
        // Ensure pose is not null before resetting
        if (pose == null) {
            System.err.println("Pose is null.");
            return;  // Early exit if pose is null
        }
    
        // Now reset the odometer position
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
    
    // Gyro and overall robot data
    //SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber("Pigeon Raw Yaw", pigeon.getYaw().getValue().magnitude());
    
    // Module states
    //SmartDashboard.putNumber("Drive Speed", blue.getDriveVelocity());
    SmartDashboard.putNumber("Blue CANCoder", blue.getCancoder());
    SmartDashboard.putNumber("Red CANCoder", red.getCancoder());
    SmartDashboard.putNumber("Green CANCoder", green.getCancoder());
    SmartDashboard.putNumber("Orange CANCoder", orange.getCancoder());
    
    // Odometry
    //SmartDashboard.putNumber("Robot X Position", getPose().getX());
    //SmartDashboard.putNumber("Robot Y Position", getPose().getY());
    //SmartDashboard.putNumber("Robot Rotation", getPose().getRotation().getDegrees());
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
        Rotation2d desRot = new Rotation2d(Math.atan2(leftY, leftX));
        double velocity = leftY;

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

    //stuff for leftalign function
    public boolean isTimerRunning() {
        return timer.get() > 0 && timer.get() < 3;
    }

    public void startLeftAlignTimer() {
        timer.restart();
        timer.start();
    }

    public void LeftAlign(double speed, double time) {

        // Keep moving left until the time elapses
        if (timer.get() < time) {
    
            // Orange lights when tracking
            Shooter.lightstrip.set(Constants.orangeLights);
    
            // Set forward speed to 0 as we're not moving forward, just strafing
            double forwardSpeed = 0.0;
    
            // Since we're strafing left, we use the negative of the input speed (strafe left is negative)
            double strafeSpeed = speed;
    
            // Rotation is not needed, so we set rotation speed to 0
            double rotationSpeed = 0.0;
    
            // Set chassis speeds for moving left
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(strafeSpeed, forwardSpeed, rotationSpeed);
    
            // Use kinematics to calculate swerve module states
            SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
            // Normalize speeds if necessary to ensure no module exceeds max speed
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
    
            // Set the desired state for each swerve module
            this.setModuleStates(moduleStates);
    
        } else {
            // Stop the movement once the timer exceeds the given time
            stopMovement();
        }
    }

    // Helper function to stop the robot's movement
    public void stopMovement() {
        ChassisSpeeds stopSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] stopStates = Constants.kDriveKinematics.toSwerveModuleStates(stopSpeeds);
        this.setModuleStates(stopStates);
    }



    public void alignSwerve(double targetX, double targetRotation, double tagArea) {

        // Orange lights when tracking
        Shooter.lightstrip.set(Constants.orangeLights);
        
        double forwardSpeed = 0.75; // was 0.75
        if (!DriverStation.isAutonomous()){
            forwardSpeed = 1.25;
        }

    
        targetRotation = targetRotation + 0.95;
    
        // Determine acceptable error based on distance from tag
        if (tagArea > 8) {
            targetRotation *= 0.5;
            forwardSpeed = 0.5;
        }
        else if (tagArea > 6) {
            targetX *= 0.5;
            targetRotation *= 0.5;
            forwardSpeed = 0.65;
        }
        else if (tagArea > 4) {
            targetX *= 0.75;
            targetRotation *= 0.75;
        }
    
        // Regulate speed
        double strafeSpeed = -targetX * 0.05;
        double rotationSpeed = -targetRotation * 0.04;
    
        // Gradual deceleration as target is approached
        double strafeThreshold = 0.55;  // Adjust the threshold as needed
        double rotationThreshold = 0.55;  // Adjust the threshold as needed
        strafeSpeed *= Math.min(1.0, Math.abs(targetX) / strafeThreshold);
        rotationSpeed *= Math.min(1.0, Math.abs(targetRotation) / rotationThreshold);
    
        // Apply damping to avoid aggressive overshoot
        double dampingFactor = 0.9; // Adjust as needed
        strafeSpeed *= dampingFactor;
        rotationSpeed *= dampingFactor;
    
        // Visualize the speeds
        SmartDashboard.putNumber("strafe", strafeSpeed);
        SmartDashboard.putNumber("rotation", rotationSpeed);
    
        // Determine whether or not to move based on error
        //rotationSpeed = Math.abs(rotationSpeed) > rotationOIConstant ? rotationSpeed : 0.0;
        //strafeSpeed = Math.abs(strafeSpeed) > strafeOIConstant ? strafeSpeed : 0.0;
    
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(strafeSpeed, forwardSpeed, rotationSpeed);
    
        // Use kinematics to calculate swerve module states
        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
        // Normalize speeds if necessary to ensure no module exceeds max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
    
        // Set the desired state for each swerve module
        this.setModuleStates(moduleStates);
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