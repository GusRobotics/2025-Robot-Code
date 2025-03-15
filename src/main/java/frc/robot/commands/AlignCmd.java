package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class AlignCmd extends Command {
    private final Limelight limelight;
    private int falseTrackingCount = 0;  // Counter to track how many times isTracking is false

    public AlignCmd(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(limelight);  // Ensure the subsystem is required
    }

    @Override
    public void initialize() {
        falseTrackingCount = 0;  // Reset the counter when the command is initialized
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
        System.out.println(falseTrackingCount);
    }

    @Override
    public boolean isFinished() {
        // Command ends if isTracking is false for more than 5 consecutive cycles
        return falseTrackingCount > 5;
    }


}
