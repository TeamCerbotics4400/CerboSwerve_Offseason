// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

 /*************** DRIVE ****************/

 /*
 * Two options: 5.50 or 6.55
 * All of this data available in
 * https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
 */

public final class Constants {
  public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.50; //Drive Gear Ratio, 5.50 or 6.55
    public static final double kTurningMotorGearRatio = 1 / 10.29; //Turning Gear Ratio
    public static final double kDriveEncoderRot2Meter = 
                                kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
    
    public static final double drivekP = 0,
                               drivekI = 0,
                               drivekD = 0,
                               drivekFF = 0;
    
    public static final double turnkP = 0.13,
                               turnkI = 0,
                               turnkD = 0,
                               turnkFF = 0;
  }

  public static final class DriveConstants{
    /* Specific module constants from FRC 95:
     * https://github.com/first95/FRC2023/blob/f0e881c39ade544b3b71936995f7f075105f0b93/Clarke/src/main/java/frc/robot/Constants.java#LL136C16-L136C23
     * Gives us a tool for a cleaner and readable swerve code
    */

    //TODO: PLACEHOLDER VALUES

    /*    
     *                   F             
     *   ┌───────┬─────────────────┬───────┐
     *   │       │                 │       │
     *   │ Mod 0 │                 │ Mod 1 │
     *   │       │                 │       │
     *   ├───────┘                 └───────┤
     *   │                                 │
     *   │            Modules              │
     * L │            Diagram              │ R
     *   │                                 │
     *   │                                 │
     *   │                                 │
     *   ├───────┐                 ┌───────┤
     *   │       │                 │       │
     *   │ Mod 3 │                 │ Mod 2 │
     *   │       │                 │       │
     *   └───────┴─────────────────┴───────┘
     *                   B
     */

    public static final class Module0{
      public static final int DRIVE_ID = 16;
      public static final int TURN_ID = 17;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = Units.degreesToRadians(201.0);//197.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final class Module1{
      public static final int DRIVE_ID = 4;
      public static final int TURN_ID = 3;
      public static final boolean driveReversed = false;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = Units.degreesToRadians(156.0);//153.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final class Module2{
      public static final int DRIVE_ID = 2;
      public static final int TURN_ID = 1;
      public static final boolean driveReversed = true;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = Units.degreesToRadians(105.0);//284.0;

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final class Module3{
      public static final int DRIVE_ID = 18;
      public static final int TURN_ID = 19;
      public static final boolean driveReversed = true;
      public static final boolean turnReversed = false;
      public static final boolean encoderReversed = false;
      public static double encoderOffset = Units.degreesToRadians(277.0);//33.0;// Algo raro

      public static final SwerveModuleConstants CONSTANTS = 
      new SwerveModuleConstants(DRIVE_ID, TURN_ID, driveReversed, 
      turnReversed, encoderReversed, encoderOffset);
    }

    public static final int IMU_ID = 13;

    //Distance between left and right wheels
    public static final double kTrackWidth = 0.5842;
    //Distance between front and back wheels
    public static final double kWheelBase = 0.6604;
    //Original
    //+ / +
    //- / +
    //- / -
    //+ / -
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 3, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 3, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 3, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 3, -kTrackWidth / 2 ));

      /*
       * Kinematics order:
       * 1. Mod0
       * 2. Mod1
       * 3. Mod2
       * 4. Mod3
       */

    /*Free speed of each gearing:
    * 5.50 = 18.01 ft/s
    * 6.55 = 15.12 ft/s
    */
    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(18.01);
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 
                kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  /*************** SUPERSTRUCTURE ****************/

  public static final class ShooterConstants {
    public static final byte LEFT_FLYWHEEL_ID = 6; 
    public static final byte RIGHT_FLYWHEEL_ID = 5; //Pending

    public static double kP= 0.078061,//0.087,
                        kI = 0.0,
                        kD = 0.0,//0.95,
                        kIz = 0,
                        kFF = 0.0495;

    public static final double shooterTreshold = 300;
  }

  public static final class ArmConstants {
    public static byte LEFT_ARM_ID = 8; 
    public static byte RIGHT_ARM_ID = 7;

    public static double ARM_DEGREES_PER_MOTOR_ROTATION = 32.89;

    //TODO: RETUNE ARM FF AND PID VALUES

    public static double kP = 0.0,//0.013821,//0.0011773,//0.012904,//4.3755E-09,
                         kI = 0.0,
                         kD = 0.0,//0.021835,//0.00017643,//0.0024401,//8.274E-10,
                         kFF = 0.0,//0.000156,
                         kMaxVelocityRadPerSecond = 100,//320,
                         kMaxAccelerationMetersPerSecondSquared = 55,//250,//320,
                         kS = 0.0,//0.94615,//0.82172,
                         kV = 0.0,//0.0021715,//0.0047927,
                         kA = 0.0,//0.0019641,//0.003212,
                         kG = 0.0;//0.12588;//0.44033;

    public static double IDLE_POSITION = 90.0,
                         SUBSTATION_POSITION = 130.5,
                         SCORING_POSITION = 100,//115;
                         BACK_FLOOR_POSITION = 75,
                         FRONT_FLOOR_POSITION = 115,
                         AVE_MARIA_SHOOT_POSITION = 120.0,
                         COUNTER_BALANCE_POSITION = 60.0,
                         TESTING_ANGLE = 90.0;

    public static double ARM_THRESHOLD = 9.5;
  }

  public static final class WristConstants {
    public static byte WRIST_ID = 9; 

    public static double kP = 0.18221,
                         kI = 0.0,
                         kD = 0.0151835,
                         kFF = 0.0,
                         kMaxVelocityRadPerSecond = 130,//440,
                         kMaxAccelerationMetersPerSecondSquared = 130,//440,
                         kS = 0.46147,
                         kV = 0.020646,
                         kA = 0.008245,
                         kG = 0.45406;

    public static double LEFT_POSITION = 90.0,
                         IDLE_POSITION = 0.0,
                         RIGHT_POSITION = -95.0;

    public static double WRIST_THRESHOLD = 4;
  }

  /*************** MISC ****************/

  public static final class VisionConstants {

    public static final String tapeLimelight = "limelight-tape";
    public static final String tagLimelightName = "limelight-tags";

    public static double HEIGHT_OF_HIGH_NODE = 0.90; //Elevation of Target
    public static double HEIGHT_OF_MID_NODE = 0.60;
    public static double HEIGHT_OF_TAG = 0.45;
    public static double LIMELIGHT_FLOOR_CLEREANCE= 0.04819; //Elevation of the Limelight
    public static double LIMELIGHT_VERTICAL_ANGLE = 0;

    public static final int normalTracking_Pipeline = 0,
                            lowAlign_Pipeline = 1,
                            midAlign_Pipeline = 2,
                            highAlign_Pipeline = 3;

  }

  public static final class IOConstants{
    public static final double kDeadband = 0.05;
  }
}
