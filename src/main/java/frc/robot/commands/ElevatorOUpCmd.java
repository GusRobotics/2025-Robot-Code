package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorOUpCmd extends Command {
    private Elevator elevator;
    private boolean direction;
    private boolean secondDirection;

    public ElevatorOUpCmd(Elevator elevator, boolean direction, boolean secondDirection) {
        this.elevator = elevator;
        this.direction = direction;
        this.secondDirection = secondDirection;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (direction) {
            elevator.enableElevator(); 
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
}