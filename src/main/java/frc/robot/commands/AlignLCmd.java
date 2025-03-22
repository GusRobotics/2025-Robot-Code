package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class AlignLCmd extends Command {
    private final Limelight limelight;
    private final Timer timer = new Timer();  // Timer to track command duration
    private int falseTrackingCount = 0;  // Counter to track how many times isTracking is false
    private final double timeLimit = 1.0;  // Time limit in seconds

    public AlignLCmd(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(limelight);  // Ensure the subsystem is required
    }

    @Override
    public void initialize() {
        falseTrackingCount = 0;  // Reset the counter when the command is initialized
        timer.reset();  // Reset the timer
        timer.start();  // Start the timer
    }

    @Override
    public void execute() {
        // Call the checkAndAlignToAprilTag method to manage tracking and alignment
        limelight.checkAndAlignToLAprilTag();

        // If tracking is false, increment the false tracking count
        if (!Limelight.isTracking) {
            falseTrackingCount++;  // Increments when tracking is lost
        } else {
            // Reset the counter if tracking is successful
            falseTrackingCount = 0;
        }
    }
}