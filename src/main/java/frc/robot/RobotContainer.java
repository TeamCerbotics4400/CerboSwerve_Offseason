// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.StateIntake;
import frc.robot.commands.StateShooterCommand;
import frc.robot.commands.TeleopCommands.TeleopControl;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;
import team4400.StateMachines;
import team4400.StateMachines.IntakeState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveTrain m_drive = new DriveTrain();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final FalconShooter m_shooter = new FalconShooter();
  
  private final Joystick chassisDriver = new Joystick(0);
  private final Joystick subsystemsDriver = new Joystick(1);

  private final NodeSelector m_selector = new NodeSelector(subsystemsDriver);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> -chassisDriver.getRawAxis(1), 
    () -> chassisDriver.getRawAxis(0), 
    () -> -chassisDriver.getRawAxis(4), 
    () -> !chassisDriver.getRawButton(4)));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(chassisDriver, 1).onTrue(
      new InstantCommand(() -> m_drive.zeroHeading()));

    //Left bumper
    /* 
   new JoystickButton(chassisDriver, 5)
   .onTrue(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION))
  .whileTrue(m_wrist.goToPosition(WristConstants.RIGHT_POSITION))
  .whileTrue(new StateIntake(m_shooter, m_arm, IntakeState.INTAKING))
  .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
  .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

  //Right bumper
  new JoystickButton(chassisDriver, 6)
  .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
  .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
  .whileTrue(new StateIntake(m_shooter, m_arm, IntakeState.INTAKING))
  .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
  .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));*/

  //Controller 2
   //Pov up
   new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_selector.updateSelectionUp()));

   //Pov down
   new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_selector.updateSelectionDown()));

   //Left bumper
   /* 
    new JoystickButton(subsystemsDriver, 5)
   .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   //Right bumper
   new JoystickButton(subsystemsDriver, 6)
   .onTrue(m_arm.goToPosition(ArmConstants.SCORING_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 4)
   .whileTrue(new StateShooterCommand(m_shooter, m_arm, m_wrist, IntakeState.SHOOTING, 
                                                                            m_selector));

   new JoystickButton(subsystemsDriver, 2).toggleOnTrue(
    new InstantCommand(() -> StateMachines.setIntakeIdle()));*/
    //m_drive.setDefaultCommand(new StickRotationControl(m_drive, driverJoystick));
    //new JoystickButton(driverJoystick, 1).onTrue(new GoToAngle(m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public DriveTrain getDrive(){
    return m_drive;
  }
}
