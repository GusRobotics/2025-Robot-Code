package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry botPoseEntry;
    private Pose3d robotPose;
    private boolean isTracking;
    private final SwerveDrive swerveDrive;


    // Constructor
    public Limelight(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.botPoseEntry = table.getEntry("botpose");  // Fetch the botpose entry from NetworkTables
        this.robotPose = new Pose3d();  // Initialize an empty robot pose
    }

    /** Returns whether the Limelight sees a target */
    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) > 0.2;
    }

    /** Returns the horizontal offset (yaw error) from crosshair to target */
    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);  // Fetch horizontal offset
    }

    /** Updates 3D pose values from NetworkTables */
    public void updatePose() {
        double[] botpose = botPoseEntry.getDoubleArray(new double[7]);

        if (botpose.length == 6 || botpose.length == 7) {
            Translation3d translation = new Translation3d(botpose[0], botpose[1], botpose[2]);
            Rotation3d rotation = new Rotation3d(Math.toRadians(botpose[3]), Math.toRadians(botpose[4]), Math.toRadians(botpose[5]));
            robotPose = new Pose3d(translation, rotation);
        } else {
            System.out.println("Invalid pose data length: " + botpose.length);
        }
    }

    /** Gets the current robot pose */
    public Pose3d getRobotPose() {
        return robotPose;
    }

    /** Posts values to SmartDashboard */
    public void display() {
        SmartDashboard.putBoolean("Has Target", hasTarget());
        SmartDashboard.putNumber("Target TX", getTX());
        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot Rotation", robotPose.getRotation().getZ());
        SmartDashboard.putBoolean("Tracking", isTracking);

    }

    /** Check and align robot to an AprilTag */
    public void checkAndAlignToAprilTag() {
        if (hasTarget()) {
            isTracking = true;
            alignToAprilTag();
        } else {
            isTracking = false;
        }
    }

    /** Aligns robot to an AprilTag based on pose information */
    public void alignToAprilTag() {

        // Check for invalid pose values
        if (robotPose == null || (robotPose.getX() == 0 && robotPose.getY() == 0 && robotPose.getRotation().getZ() == 0)) {
            System.out.println("Invalid pose data, skipping alignment.");
            return;
        }

        // Extract translation and rotation information from pose
        double targetX = robotPose.getX();
        double targetY = robotPose.getY();
        double targetRotation = robotPose.getRotation().getZ();

        double tagArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);


        // Check if the pose values are out of realistic range
        if (Math.abs(targetX) > 10 || Math.abs(targetY) > 10 || Math.abs(targetRotation) > 180) {
            System.out.println("Pose values out of range, skipping alignment.");
            return;
        }


        // Only update the alignment if enough time has passed (to reduce jitter)
        swerveDrive.alignSwerve(targetX, targetY, targetRotation, tagArea);
    }


    @Override
    public void periodic() {
        updatePose(); 
        display();    
    }
}



