// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final double kTrackWidth = Units.inchesToMeters(24.75);
  // Distance between right and left wheels
  public static final double kWheelBase = Units.inchesToMeters(24.75);
  // Distance between front and back wheels
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

  //Drive ID Constants 
        //Blue pod ids
        public static final int blueDrive = 8;
        public static final int blueSteer = 10;
        public static final int kBlueDriveAbsoluteEncoderPort = 9;
        //green pod ids
        public static final int greenDrive = 5;
        public static final int greenSteer = 7;
        public static final int kGreenDriveAbsoluteEncoderPort = 6;
        //orange drive ids
        public static final int orangeDrive = 11;
        public static final int orangeSteer = 13;
        public static final int kOrangeDriveAbsoluteEncoderPort = 12;
        //red drive ids
        public static final int redDrive = 2;
        public static final int redSteer = 4;
        public static final int kRedDriveAbsoluteEncoderPort = 3;

        public static final int kPigeonPort = 1;
  
  
  //Elevator ID Constants
  public static final int leftElevator = 30; //change
  public static final int rightElevator = 31;//change


  //Shooter ID Constants
  public static final int leftShooter = 20; //change
  public static final int rightShooter = 21;//change


  //Climber ID Constants
  public static final int leftClimber = 40; //Change 
  public static final int rightClimber = 41;//Change


  //Light PWM constant
  public static final int ledChannel = 0;


  //Light Constants
  public static final double pinkLights = 0.57;
  public static final double blueLights = 0.83;
  public static final double yellowLights = 0.69;
  public static final double greenLights = 0.77;
  public static final double orangeLights = 0.63;



  // Encoder Constants
  public static final double kBlueDriveAbsoluteEncoderOffset = -0.16722265625;
  public static final double kGreenDriveAbsoluteEncoderOffset = 0.416259765625; 
  public static final double kOrangeDriveAbsoluteEncoderOffset = 0.37546484375;
  public static final double kRedDriveAbsoluteEncoderOffset = -0.18896484375;

  public static final boolean kBlueTurningEncoderReversed = false;
  public static final boolean kGreenTurningEncoderReversed = false;
  public static final boolean kOrangeTurningEncoderReversed = false;
  public static final boolean kRedTurningEncoderReversed = false;

  public static final boolean kBlueDriveEncoderReversed = false;
  public static final boolean kGreenDriveEncoderReversed = false;
  public static final boolean kOrangeDriveEncoderReversed = false;
  public static final boolean kRedDriveEncoderReversed = false;

  public static final boolean kBlueDriveAbsoluteEncoderReversed = true;
  public static final boolean kGreenDriveAbsoluteEncoderReversed = true;
  public static final boolean kOrangeDriveAbsoluteEncoderReversed = true;
  public static final boolean kRedDriveAbsoluteEncoderReversed = true;


  // Swerve Drive
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Front Left
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Back Left
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Back Right
);

    //Swerve Settings Constants
    public static final int kWheelRadius = 2;
    public static final double kDriveMotorGearRatio = 1 / 5.8462;
    public static final double KTurningMotorGearRatio = 1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
      * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = KTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 9.3576;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2.5;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                    kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5.64558;


    //FOR TUNING - PID VALUES
    public static final double kPTurning = .35; //original value was .3
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kPhysicalMaxSpeedMetersPerSecond, kPhysicalMaxAngularSpeedRadiansPerSecond);
    TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State();


    //OI Constants
    public static final double OIConstants = 0.05;



    //Elevator Level Constants
    public static final double GroundPos = 0;
    public static final double L1Pos = 5;
    public static final double L2Pos = 6.8;
    public static final double L2PosAuto = 6.8;
    public static final double L3Pos = 13.45;
    public static final double L4Pos = 25.8; // was 25.8


    //Shooter Constants in Shooter.java



  

}
