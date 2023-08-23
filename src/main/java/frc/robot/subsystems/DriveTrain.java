// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  private SwerveModule[] swerveModules = new SwerveModule[]{
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

  private VisionSystem m_vision = new VisionSystem(this);

  double currentAzimuth = 45;

  private final PIDController xPID = new PIDController(0, 0, 0);
  private final PIDController yPID = new PIDController(0, 0, 0);
  private final PIDController rotationPID = new PIDController(0, 0, 0);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    zeroHeading();
    SmartDashboard.putNumber("Current Azimuth", currentAzimuth);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Current Sensor Encoder", swerveModules[0].getAnalogSensorPos());
    
    
    double newAzimuth = SmartDashboard.getNumber("Current Azimuth", currentAzimuth);
    //SmartDashboard.putNumber("Current Azimuth", currentAzimuth);

    if(currentAzimuth != newAzimuth){
      currentAzimuth = newAzimuth;
    }
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

  public void stopModules(){
    for(SwerveModule mod : swerveModules){
      mod.stop();
    }
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

  //DEBUG
  public void setModuleAngle(){
    swerveModules[0].setTurnMotorAngle(currentAzimuth);
  }

  public void setTestMotorPower(double power){
    swerveModules[0].setTurnMotorPower(power);
  }

  //AUTO RAMSETE
  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory){
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
  }
}