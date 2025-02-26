// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.cameraserver.CameraServer;

// public class Limelight implements Subsystem {
//     private final NetworkTable table;
//     private final NetworkTableEntry tx;
//     private final NetworkTableEntry ty;
//     private final NetworkTableEntry ta;

//     private double x;
//     private double y;
//     private double area;

//     // Constructor initializes NetworkTables
//     public Limelight() {
//         CameraServer.startAutomaticCapture();
//         this.table = NetworkTableInstance.getDefault().getTable("limelight");
//         this.tx = table.getEntry("tx");
//         this.ty = table.getEntry("ty");
//         this.ta = table.getEntry("ta");
//     }

//     /** Reads values from NetworkTables */
//     public void getValues() {    
//         x = tx.getDouble(0.0);
//         y = ty.getDouble(0.0);
//         area = ta.getDouble(0.0);
//     }

//     /** Posts values to SmartDashboard */
//     public void display() {
//         SmartDashboard.putNumber("LimelightX", x);
//         SmartDashboard.putNumber("LimelightY", y);
//         SmartDashboard.putNumber("LimelightArea", area);
//     }

//     /** Displays Limelight camera stream on SmartDashboard */
//     public void displayCameraStream() {
//         SmartDashboard.putString("CameraStream", "limelight");
//     }

//     /** Forces Limelight to stay in vision mode and updates dashboard */
//     @Override
//     public void periodic() {
//         getValues();   // Update vision tracking data
//         display();     // Send values to SmartDashboard
//         displayCameraStream(); // Display camera stream
//         table.getEntry("camMode").setNumber(0); // Ensure Vision Processing Mode is enabled
//         System.out.println("test");
//     }

// }
