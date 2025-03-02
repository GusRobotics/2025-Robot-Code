package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class AlignCmd extends Command {
    private final Limelight limelight;

    public AlignCmd(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(limelight);  // Ensure the subsystem is required
    }

    @Override
    public void initialize() {
        System.out.println("Starting alignment to AprilTag...");
    }

    @Override
    public void execute() {
        limelight.checkAndAlignToAprilTag();
    }

}