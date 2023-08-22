// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400.Util;

/** Add your docs here. */
/* Special class for helping us organize Swerve Module Constants */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final boolean driveReversed;
    public final boolean turnReversed;
    public final boolean absoluteEncoderReversed;
    public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID,
    boolean driveReversed, boolean turnReversed, boolean absoluteEncoderReversed, 
    double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.driveReversed = driveReversed;
        this.turnReversed = turnReversed;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.angleOffset = angleOffset;
    }
}
