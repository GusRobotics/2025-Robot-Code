package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberUpCmd extends Command {
    private Climber climber;
    private boolean direction;
    private boolean secondDirection;

    public ClimberUpCmd(Climber climber, boolean direction, boolean secondDirection) {
        this.climber = climber;
        this.direction = direction;
        addRequirements(climber);
    }

    // Start
    @Override
    public void initialize() {
        if (direction) 
        {
            climber.reverseClimber();
        }
        else if(secondDirection){
            climber.stopClimber();
        }
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous())
        {
            climber.end();
        }
    }

}