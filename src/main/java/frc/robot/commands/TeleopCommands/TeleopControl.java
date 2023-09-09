// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveTrain;

public class TeleopControl extends CommandBase {
  private final DriveTrain m_drive;
  private Supplier<Double> translationSup;
  private Supplier<Double> strafeSup;
  private Supplier<Double> rotationSup;
  private Supplier<Boolean> robotCentricSup;
 
  /** Creates a new TeleopControl. */
  public TeleopControl(DriveTrain m_drive, Supplier<Double> translationSup, 
  Supplier<Double> strafeSup,
  Supplier<Double> rotationSup, 
  Supplier<Boolean> robotCentricSup) {

    this.m_drive = m_drive;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationSup.get(), 
                                                              IOConstants.kDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.get(), 
                                                              IOConstants.kDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.get(), 
                                                              IOConstants.kDeadband); 
                                                              
    m_drive.drive(
      new Translation2d(translationVal, strafeVal)
      .times(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond), 
      rotationVal * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, 
      !robotCentricSup.get(), 
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
