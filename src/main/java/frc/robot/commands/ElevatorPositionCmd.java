package frc.robot.commands;

//import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPositionCmd extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    public ElevatorPositionCmd(Elevator elevator, double position) {
        this.elevator = elevator;
        this.targetPosition = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        boolean finished = elevator.atTargetPosition();
        return finished;
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous())
        {
            elevator.end();
        }
    }
}