package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry botPoseEntry;
    private Pose3d robotPose;
    public static boolean isTracking;
    private final SwerveDrive swerveDrive;
    private int counter;


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

    }

    /** Check and align robot to an AprilTag */
    public void checkAndAlignToAprilTag() {
        if (hasTarget()) {
            isTracking = true;
            updatePose(); // added (to make sure we aren't using old data)
            alignToAprilTag();
        } else {
            isTracking = false;
        }
    }

    /** Aligns robot to an AprilTag based on pose information */
    public void alignToAprilTag() {
        // Get the target pose data from the Limelight
        double[] targetPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    
        // Check for invalid pose values (pose array length should be 6)
        if (targetPose.length != 6 || (targetPose[0] == 0 && targetPose[1] == 0 && targetPose[4] == 0)) {
            System.out.println("Invalid pose data, skipping alignment.");
            return;
        }
    
        // Extract the horizontal offset and yaw (rotation) from the target pose
        double targetHorizontal = getTX();
        double targetYaw = targetPose[4];  // Yaw is at index 4
    
        // Get the tag area
        double tagArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    
        // Update movement
        swerveDrive.alignSwerve(targetHorizontal, targetYaw, tagArea);
    }
    


    @Override
    public void periodic() {
        counter++;  // Increment the counter

        if (counter >= 3 || DriverStation.isAutonomous()) {  // Check if this is the third time periodic is called
            updatePose();  // Call updatePose every third time
            counter = 0;  // Reset the counter after updatePose is called
        }

        display();  // Always display information on SmartDashboard
    }
}




