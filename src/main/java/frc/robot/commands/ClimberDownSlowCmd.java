package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDownSlowCmd extends Command {
    private Climber climber;
    private boolean direction;
    private boolean secondDirection;
    private Timer ourTimer;

    public ClimberDownSlowCmd(Climber climber, boolean direction, boolean secondDirection) {
        this.climber = climber;
        this.direction = direction;
        this.ourTimer = new Timer();
        addRequirements(climber);
    }

    // Start
    @Override
    public void initialize() {
        if (direction) 
        {
            climber.enableSlowClimber();
        }
        else if(secondDirection){
            climber.stopClimber();
        }
        ourTimer.restart();
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous())
        {
            climber.end();
        }
    }

    @Override
    public boolean isFinished() { 
        return DriverStation.isAutonomous() && ourTimer.get() > 3; 
    }
}