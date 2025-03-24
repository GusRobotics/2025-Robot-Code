package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;


public class AlignLCmd extends Command {
    private final Limelight limelight;
    private final SwerveDrive swerveDrive;
    //private final Timer timer = new Timer();  // Timer to track command duration
    // private int falseTrackingCount = 0;  // Counter to track how many times isTracking is false
    // private final double timeLimit = 1.0;  // Time limit in seconds

    public AlignLCmd(Limelight limelight, SwerveDrive swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        addRequirements(limelight);  // Ensure the subsystem is required
    }

    @Override
    public void initialize() {
        //falseTrackingCount = 0;  // Reset the counter when the command is initialized
        //timer.reset();  // Reset the timer
        //timer.start();  // Start the timer
    }

    @Override
    public void execute() {
        double tagArea = limelight.getTagArea();
    
        // Start the left align movement if the tag area is too large (>= 15)
        if (tagArea >= 50 || !limelight.hasTarget()) {
            if (!swerveDrive.isTimerRunning()) { // Check if timer has already started
                swerveDrive.startLeftAlignTimer();  // Start the timer for leftward movement
            }
            swerveDrive.LeftAlign(1, 0.53);  // Keep moving left
        } else {
            limelight.checkAndAlignToLAprilTag();
        }
    }
    
}