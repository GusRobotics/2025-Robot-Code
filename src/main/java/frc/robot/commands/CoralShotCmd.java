package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
//import frc.robot.RobotContainer;

public class CoralShotCmd extends Command {
    private Shooter shooter;

    public CoralShotCmd(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    // Start
    @Override
    public void initialize() {
        shooter.enableShooter();
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
}
