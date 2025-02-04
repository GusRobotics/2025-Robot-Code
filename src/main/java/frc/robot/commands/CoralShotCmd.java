package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer;

public class CoralShotCmd extends Command {
    private Shooter shooter;
    private boolean direction;
    private Timer ourTimer;

    public CoralShotCmd(Shooter shooter, boolean direction) {
        this.shooter = shooter;
        this.direction = direction;
        this.ourTimer = new Timer();
        addRequirements(shooter);
    }

    // Start
    @Override
    public void initialize() {
        if(direction){
            shooter.enableShooter();
        }
        else{
            shooter.enableAutoShooter();
        }
    }

    @Override
    public void execute() {
            shooter.enableShooter();
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
