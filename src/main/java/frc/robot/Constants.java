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
  public static final double kTrackWidth = Units.inchesToMeters(29.75);
  // Distance between right and left wheels
  public static final double kWheelBase = Units.inchesToMeters(29.75);
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

    public static final double kBlueDriveAbsoluteEncoderOffset = 0.17822265625;
    public static final double kGreenDriveAbsoluteEncoderOffset = -0.416259765625;
    public static final double kOrangeDriveAbsoluteEncoderOffset = -0.37646484375;
    public static final double kRedDriveAbsoluteEncoderOffset = 0.18896484375;
}
