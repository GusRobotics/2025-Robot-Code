// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
//import com.pathplanner.lib.path.PathConstraints;

//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.ElevatorOUpCmd;
import frc.robot.commands.ElevatorPositionCmd;
import frc.robot.commands.ElevatorODownCmd;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.CoralIndexCmd;
import frc.robot.commands.CoralShotCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AlignCmd;
import frc.robot.commands.ClimberDownCmd;
import frc.robot.commands.ClimberUpCmd;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.Limelight;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // subsystems
  public static Shooter shooter = new Shooter();
  public static Elevator elevator = new Elevator();
  public static SwerveDrive drive = new SwerveDrive();
  public static Climber climber = new Climber();
  private final Limelight limelight = new Limelight(drive);

  
  // controllers
  public static CommandPS4Controller baseController = new CommandPS4Controller(0);

  // TRIGGERS

  // forward and reverse override for elevator
  private Trigger ElevatorOUp = baseController.povUp();
  private Trigger ElevatorODown = baseController.povDown();
  
  // coral shooter
  private static final double TRIGGER_THRESHOLD = 0.1;
  private Trigger CoralShot = new Trigger(() -> baseController.getRightX() > TRIGGER_THRESHOLD);

  // coral indexer
  public static boolean isCoralIndexEnabled = true;
  //private Trigger CoralIndex = new Trigger(() -> isCoralIndexEnabled); 

  // Climber down and up
  private Trigger ClimberDown = baseController.R2();
  private Trigger ClimberUp = baseController.L2();

  // presets
  private Trigger ElevatorBottom = baseController.square();
  private Trigger ElevatorL2 = baseController.cross();
  private Trigger ElevatorL3 = baseController.triangle();
  private Trigger ElevatorL4 = baseController.circle();
  private Trigger AutoAlign = baseController.L1();

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {

    // COMMANDS

    // forward override elevator
    ElevatorOUp.whileTrue(new ElevatorOUpCmd(elevator, true, false));

    // reverse override elevator
    ElevatorODown.whileTrue(new ElevatorODownCmd(elevator, true, false));

    // coral indexing
    baseController.povLeft().onTrue(new InstantCommand(() -> {
      isCoralIndexEnabled = !isCoralIndexEnabled;
    }));

    // coral shooting
    CoralShot.whileTrue(new CoralShotCmd(shooter));

    // climber down
    ClimberDown.whileTrue(new ClimberDownCmd(climber, true, false));

    // climber up
    ClimberUp.whileTrue(new ClimberUpCmd(climber, true, false));

    // elevator presets
    ElevatorBottom.whileTrue(new ElevatorPositionCmd(elevator, Constants.GroundPos));
    ElevatorL2.whileTrue(new ElevatorPositionCmd(elevator, Constants.L2Pos));
    ElevatorL3.whileTrue(new ElevatorPositionCmd(elevator, Constants.L3Pos));
    ElevatorL4.whileTrue(new ElevatorPositionCmd(elevator, Constants.L4Pos));

    // auto align
    AutoAlign.whileTrue(new AlignCmd(limelight));

  }

  // PATH PLANNER

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    // Named Commands for auto
    NamedCommands.registerCommand("Elevator Override Up", new ElevatorOUpCmd(elevator, true, false));
    NamedCommands.registerCommand("Elevator Override Down", new ElevatorODownCmd(elevator, true, false));
    NamedCommands.registerCommand("Coral Index", new CoralIndexCmd(shooter, true, false));
    NamedCommands.registerCommand("Climber Down", new ClimberDownCmd(climber, true, false));
    NamedCommands.registerCommand("Climber Up", new ClimberUpCmd(climber, true, false));
    NamedCommands.registerCommand("Coral Shot", new CoralShotCmd(shooter));
    //lower powered coral shot??
    NamedCommands.registerCommand("Ground", new ElevatorPositionCmd(elevator, Constants.GroundPos));
    //L1?
    NamedCommands.registerCommand("L2", new ElevatorPositionCmd(elevator, Constants.L2Pos));
    NamedCommands.registerCommand("L3", new ElevatorPositionCmd(elevator, Constants.L3Pos));
    NamedCommands.registerCommand("L4", new ElevatorPositionCmd(elevator, Constants.L4Pos));
    NamedCommands.registerCommand("Auto Align", new AlignCmd(limelight));
    
    // Set default command for the shooter to run Coral Index immediately
    shooter.setDefaultCommand(new CoralIndexCmd(shooter, true, false));
    
    // set pink lights default
    shooter.setDefaultLights();

    // Build an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}