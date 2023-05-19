// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.Module0.CONSTANTS);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.Module1.CONSTANTS);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.Module2.CONSTANTS);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.Module4.CONSTANTS);

  private final Pigeon2 imu = new Pigeon2(DriveConstants.IMU_ID);

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
