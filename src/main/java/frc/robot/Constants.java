// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import team4400.Util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 18.01; //Two options: 5.50 or 6.55
    public static final double kTurningMotorGearRatio = 10.29;
    public static final double kDriveEncoderRot2Meter = 
                                kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5; 
  }

  public static final class DriveConstants{
    /* Specific module constants from FRC 95:
     * https://github.com/first95/FRC2023/blob/f0e881c39ade544b3b71936995f7f075105f0b93/Clarke/src/main/java/frc/robot/Constants.java#LL136C16-L136C23
     * Gives us a tool of a cleaner and readable swerve code
    */

    //TODO: PLACEHOLDER VALUES
    public static final class Module0{
      public static final int DRIVE_ID = 1;
      public static final int TURN_ID = 2;
      public static final int ENCODER_ID = 3;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, ENCODER_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final class Module1{
      public static final int DRIVE_ID = 4;
      public static final int TURN_ID = 5;
      public static final int ENCODER_ID = 6;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, ENCODER_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final class Module2{
      public static final int DRIVE_ID = 7;
      public static final int TURN_ID = 8;
      public static final int ENCODER_ID = 9;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, ENCODER_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final class Module4{
      public static final int DRIVE_ID = 10;
      public static final int TURN_ID = 11;
      public static final int ENCODER_ID = 12;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = 0.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, ENCODER_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    /*Free speed of each gearing:
    * 5.50 = 20.25 ft/s
    * 6.55 = 17.01 ft/s
    */
    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(20.25);
  }
}
