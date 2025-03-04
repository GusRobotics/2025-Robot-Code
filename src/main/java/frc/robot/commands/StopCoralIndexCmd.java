package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopCoralIndexCmd extends Command {
    private Shooter shooter;
    //private boolean direction;
    private Timer ourTimer;

    public StopCoralIndexCmd(Shooter shooter, boolean direction) {
        this.shooter = shooter;
        //this.direction = direction;
        this.ourTimer = new Timer();
        addRequirements(shooter);
    }

    // Start
    @Override
    public void initialize() {
        shooter.stopShooter();

    }

    @Override
    public void execute() {
            shooter.stopShooter();
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous())
        {
            shooter.end();
        }
    }

    @Override
    public boolean isFinished() { 
        return DriverStation.isAutonomous() && ourTimer.get() > 3; 
    }
}
