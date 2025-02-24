package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorODownCmd extends Command {
    private Elevator elevator;
    private boolean direction;
    private boolean secondDirection;
    private Timer ourTimer;

    public ElevatorODownCmd(Elevator elevator, boolean direction, boolean secondDirection) {
        this.elevator = elevator;
        this.direction = direction;
        this.secondDirection = secondDirection;
        this.ourTimer = new Timer();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        ourTimer.restart();
    }

    @Override
    public void execute() {
        if (direction) {
            elevator.reverseElevator(); 
        } else if (secondDirection) {
            elevator.stopElevator();
        }
    }

    @Override
    public void end(boolean terminated) {
        if (!DriverStation.isAutonomous()) {
            elevator.end();
        }
    }

    @Override
    public boolean isFinished() { 
        return DriverStation.isAutonomous() && ourTimer.get() > 3; 
    }
}