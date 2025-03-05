package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
//import frc.robot.RobotContainer;

public class L1ShotCmd extends Command {
    private Shooter shooter;

    public L1ShotCmd(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    // Start
    @Override
    public void initialize() {
        shooter.L1Shot();
    }

    @Override
    public void execute() {
        shooter.L1Shot();
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous())
        {
            shooter.end();  
        }
    }
}
