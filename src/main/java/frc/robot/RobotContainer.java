// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.ElevatorOUpCmd;
import frc.robot.commands.ElevatorODownCmd;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.CoralIndexCmd;
import frc.robot.commands.StopCoralIndexCmd;
import frc.robot.commands.CoralShotCmd;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.ClimberDownCmd;
import frc.robot.commands.ClimberUpCmd;
import frc.robot.subsystems.Climber;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static Shooter shooter = new Shooter();
  public static Elevator elevator = new Elevator();
  public static SwerveDrive drive = new SwerveDrive();
  public static Climber climber = new Climber();
  

  public static CommandPS4Controller baseController = new CommandPS4Controller(0);


  // // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //forward and reverse override for elevator
  private Trigger ElevatorOUp = baseController.povUp();
  private Trigger ElevatorODown = baseController.povDown();
  
  //coral shooter
  private static final double TRIGGER_THRESHOLD = 0.1;
  private Trigger CoralShot = new Trigger(() -> baseController.getRightX() > TRIGGER_THRESHOLD);

  //coral indexer
  public static boolean isCoralIndexEnabled = false;
  private Trigger CoralIndex = new Trigger(() -> isCoralIndexEnabled); 

  //Climber down and up
  private Trigger ClimberDown = baseController.R2();
  private Trigger ClimberUp = baseController.L2();

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //forward override elevator
    ElevatorOUp.whileTrue(new ElevatorOUpCmd(elevator, true, false));

    //reverse override elevator
    ElevatorODown.whileTrue(new ElevatorODownCmd(elevator, true, false));

    //coral indexing
    baseController.povLeft().onTrue(new InstantCommand(() -> {
      isCoralIndexEnabled = !isCoralIndexEnabled;
    }));
    CoralShot.onTrue(new InstantCommand(() -> {
      isCoralIndexEnabled = !isCoralIndexEnabled;
    }));

    CoralIndex.whileTrue(new StopCoralIndexCmd(shooter, false, false));

    //coral shooting
    CoralShot.whileTrue(new CoralShotCmd(shooter, true));

    //climber down
    ClimberDown.whileTrue(new ClimberDownCmd(climber, true, false));

    //climber up
    ClimberUp.whileTrue(new ClimberUpCmd(climber, true, false));

  }


  //PATH PLANNER

     
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    NamedCommands.registerCommand("Elevator Override Up", new ElevatorOUpCmd(elevator, true, false));
    NamedCommands.registerCommand("Elevator Override Down", new ElevatorODownCmd(elevator, true, false));
    NamedCommands.registerCommand("Coral Index", new CoralIndexCmd(shooter, true, false));
    NamedCommands.registerCommand("Climber Down", new ClimberDownCmd(climber, true, false));
    NamedCommands.registerCommand("Climber Up", new ClimberUpCmd(climber, true, false));
    
    // Set default command for the shooter to run Coral Index immediately
    shooter.setDefaultCommand(new CoralIndexCmd(shooter, true, false));

    //set pink lights
    shooter.setDefaultLights();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();


    autoChooser.addOption("New Auto", new PathPlannerAuto("New Auto"));
    
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
