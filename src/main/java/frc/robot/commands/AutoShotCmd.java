package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
//import frc.robot.RobotContainer;

public class AutoShotCmd extends Command {
    private Shooter shooter;

    public AutoShotCmd(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    // Start
    @Override
    public void initialize() {
        shooter.AutoShot();
    }

    @Override
    public void execute() {
        shooter.AutoShot();
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous())
        {
            shooter.end();  
        }
    }
}
