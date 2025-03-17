package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class AlignCmd extends Command {
    private final Limelight limelight;
    private final Timer timer = new Timer();  // Timer to track command duration
    private int falseTrackingCount = 0;  // Counter to track how many times isTracking is false
    private final double timeLimit = 1.0;  // Time limit in seconds

    public AlignCmd(Limelight limelight) {
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
        limelight.checkAndAlignToAprilTag();

        // If tracking is false, increment the false tracking count
        if (!Limelight.isTracking) {
            falseTrackingCount++;  // Increments when tracking is lost
        } else {
            // Reset the counter if tracking is successful
            falseTrackingCount = 0;
        }
    }

    // @Override
    // public boolean isFinished() {
    //     // Command ends if isTracking is false for more than 5 consecutive cycles or the time limit is exceeded
    //     return falseTrackingCount > 5 || timer.hasElapsed(timeLimit);
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     // Actions to take when the command ends, either due to timeout or interruption
    //     timer.stop();  // Stop the timer
    //     if (interrupted) {
    //         System.out.println("Alignment interrupted.");
    //     } else if (falseTrackingCount > 5) {
    //         System.out.println("Alignment ended due to lost tracking.");
    //     } else if (timer.hasElapsed(timeLimit)) {
    //         System.out.println("Alignment ended due to time limit.");
    //     }
    // }
}
