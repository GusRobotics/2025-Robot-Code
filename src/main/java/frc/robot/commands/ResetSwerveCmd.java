package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d; // Import Pose2d
import edu.wpi.first.math.geometry.Rotation2d;


public class ResetSwerveCmd extends Command {
    Pose2d zeroPose = new Pose2d(0, 0, new Rotation2d(0));

    // Constructor to pass in a Pose2d
    public ResetSwerveCmd() {
        
    }

    // Start
    @Override
    public void initialize() {
        // Use the passed-in pose to reset odometry
        SwerveDrive.getInstance().resetOdometry(zeroPose);
    }

    // Optional: if you want to use execute, uncomment and add any logic if necessary
    // @Override
    // public void execute() {
    // }

    @Override
    public void end(boolean terminated) {
        // Add any cleanup logic here if necessary
    }
}
