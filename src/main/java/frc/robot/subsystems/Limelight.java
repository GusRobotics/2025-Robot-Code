package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Limelight subsystem to retrieve 2D and 3D vision data */
public class Limelight extends SubsystemBase {
    private final NetworkTable table;
    
    // 2D Tracking Entries
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;

    // 3D Tracking (botpose) Array
    private final NetworkTableEntry botpose;

    private double x, y, area;  // 2D tracking values
    private double[] pose;      // 3D pose array

    /** Constructor initializes NetworkTables */
    public Limelight() {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");

        // 2D vision tracking
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");

        // 3D pose estimation
        this.botpose = table.getEntry("botpose");
    }

    /** Reads 2D and 3D values from NetworkTables */
    public void getValues() {    
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        pose = botpose.getDoubleArray(new double[6]);  // 6D pose (X, Y, Z, Roll, Pitch, Yaw)
    }

    /** Returns the estimated robot pose as a Pose2d object */
    public Pose2d getEstimatedPose() {
        if (pose.length < 6) {
            return new Pose2d(); // Default empty pose
        }
        return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
    }

    /** Posts values to SmartDashboard */
    public void display() {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("BotPoseX", pose[0]);
        SmartDashboard.putNumber("BotPoseY", pose[1]);
        SmartDashboard.putNumber("BotPoseZ", pose[2]);
        SmartDashboard.putNumber("BotPoseRoll", pose[3]);
        SmartDashboard.putNumber("BotPosePitch", pose[4]);
        SmartDashboard.putNumber("BotPoseYaw", pose[5]);
    }

    /** Forces Limelight to stay in vision mode and updates dashboard */
    @Override
    public void periodic() {
        getValues();   // Update vision tracking data
        display();     // Send values to SmartDashboard
        table.getEntry("camMode").setNumber(0); // Ensure Vision Processing Mode is enabled
    }
}
