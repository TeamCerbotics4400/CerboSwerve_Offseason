// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DebugCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class StickRotationControl extends CommandBase {
  /** Creates a new StickRotationControl. */
  DriveTrain m_drive;
  Joystick m_joy;
  public StickRotationControl(DriveTrain m_drive, Joystick m_joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_joy = m_joy;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setTestMotorPower(m_joy.getRawAxis(4) * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
