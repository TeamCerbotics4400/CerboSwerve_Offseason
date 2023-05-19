// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  private SwerveModule[] swerveModules = new SwerveModule[]{
    new SwerveModule(0, DriveConstants.Module0.CONSTANTS),
    new SwerveModule(1, DriveConstants.Module1.CONSTANTS),
    new SwerveModule(2, DriveConstants.Module2.CONSTANTS),
    new SwerveModule(3, DriveConstants.Module3.CONSTANTS)
  };

  private final Pigeon2 imu = new Pigeon2(DriveConstants.IMU_ID);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void zeroHeading(){
    imu.setYaw(0);
  }

  public double getHeading(){
    return Math.IEEEremainder(imu.getYaw(), 360);
  }

  public void resetModuleEncoders(){
    for(SwerveModule mod : swerveModules){
      mod.resetEncoders();
    }
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : swerveModules){
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : swerveModules){
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }
}
