package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CoralIndexCmd extends Command {
    private Shooter shooter;
    private boolean direction;
    private boolean secondDirection;
    private Timer ourTimer;

    public CoralIndexCmd(Shooter shooter, boolean direction, boolean secondDirection) {
        this.shooter = shooter;
        this.direction = direction;
        this.ourTimer = new Timer();
        addRequirements(shooter);
    }

    // Start
    @Override
    public void initialize() {
        if (direction) 
        {
            shooter.enableIndex();
        }
        else if(secondDirection){
            shooter.stopShooter();
        }
        ourTimer.restart();
    }

    @Override
    public void execute() {
        // Continuously check shooter when in direction mode
        if (direction) {
            shooter.enableIndex();
        }
    }

    @Override
    public void end(boolean terminated) {
        shooter.end();
    }

    @Override
    public boolean isFinished() { 
        return DriverStation.isAutonomous() && ourTimer.get() > 3; 
    }
}
