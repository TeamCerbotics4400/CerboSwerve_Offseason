// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  public SwerveModule[] swerveModules = new SwerveModule[]{
    //Left Front Module
    new SwerveModule(0, DriveConstants.Module0.CONSTANTS),
    //Right Front Module
    new SwerveModule(1, DriveConstants.Module1.CONSTANTS),
    //Back Right Module
    new SwerveModule(2, DriveConstants.Module2.CONSTANTS),
    //Back Left Module
    new SwerveModule(3, DriveConstants.Module3.CONSTANTS)
  };

  private final Pigeon2 imu = new Pigeon2(DriveConstants.IMU_ID);

  //private VisionSystem m_vision = new VisionSystem(this);

  private final PIDController xPID = new PIDController(0, 0, 0);
  private final PIDController yPID = new PIDController(0, 0, 0);
  private final PIDController rotationPID = new PIDController(0, 0, 0);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){}
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for(SwerveModule mod : swerveModules){
      SmartDashboard.putNumber("Module [" + mod.moduleNumber + "] Absolute Encoder", 
      Units.radiansToDegrees(swerveModules[mod.moduleNumber].getAbsoluteEncoderRad()));
    }

    for(SwerveModule mod : swerveModules){
      SmartDashboard.putNumber("Module [" + mod.moduleNumber + "] Integrated Encoder", 
      Units.radiansToDegrees(swerveModules[mod.moduleNumber].getTurningPosition()));
    }

    SmartDashboard.putNumber("IMU Angle", getHeading());    
  }

  public void zeroHeading(){
    imu.setYaw(0);
  }

  public double getHeading(){
    return Math.IEEEremainder(-imu.getYaw(), 360);
  }

  public void resetModuleEncoders(){
    for(SwerveModule mod : swerveModules){
      mod.resetEncoders();
    }
  }

  public void stopModules(){
    for(SwerveModule mod : swerveModules){
      mod.stop();
    }
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, 
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    for(SwerveModule mod : swerveModules){
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
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

  //AUTO RAMSETE
  /*public Command followTrajectoryCommand(PathPlannerTrajectory trajectory){
    PPSwerveControllerCommand ramseteCommand = new PPSwerveControllerCommand(
     trajectory, 
     () -> m_vision.estimatedPose2d(),
     DriveConstants.kSwerveKinematics,
     xPID,
     yPID, 
     rotationPID, 
     this::setModuleStates, 
     this);

    //Estoy cansado jefe

      return ramseteCommand;
  }*/
}