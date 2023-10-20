// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.AutoCommands.ArmIntake;
import frc.robot.commands.AutoCommands.ArmShoot;
import frc.robot.commands.AutoCommands.IdleArm;
//import frc.robot.commands.DebugCommands.PIDModuleTuner;
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

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  private final String m_DefaultAuto = "NO AUTO";
  private String m_selectedAuto;
  private final String[] m_autoNames = {"NO AUTO", "DRIVE TUNER", "ROTATION TUNER"};

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_autoChooser.setDefaultOption("No Auto", m_DefaultAuto);
    m_autoChooser.addOption("Drive Tuner", m_autoNames[1]);
    m_autoChooser.addOption("Rotation Tuner", m_autoNames[2]);
    
    SmartDashboard.putData("Auto Choices", m_autoChooser);

    NamedCommands.registerCommand("ArmIdle", new IdleArm(m_arm, m_wrist));
    NamedCommands.registerCommand("ArmIntake", new ArmIntake(m_arm, m_wrist, m_shooter));
    NamedCommands.registerCommand("ArmShoot", 
                                          new ArmShoot(m_arm, m_wrist, m_shooter, m_selector));


    m_drive.setDefaultCommand(new TeleopControl
    (m_drive, 
    () -> chassisDriver.getRawAxis(1), 
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
  .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

  //Controller 2
   //Pov up
   new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_selector.updateSelectionUp()));

   //Pov down
   new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_selector.updateSelectionDown()));

   //Left bumper
   
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
    new InstantCommand(() -> StateMachines.setIntakeIdle()));

   /**********  DEBUGGING  **********/
   //new JoystickButton(chassisDriver, 1).whileTrue(new DebugShooterCommand(m_shooter));

   //new JoystickButton(chassisDriver, 2).whileTrue(new PIDModuleTuner(m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String autoSelected = null;
    m_selectedAuto = m_autoChooser.getSelected();
    
    System.out.println("Auto seleted:" + m_selectedAuto);

    switch(m_selectedAuto){
      case "NO AUTO":
        autoSelected = null;
      break;
      
      case "DRIVE TUNER":
        autoSelected = "DrivePID";
      break;
      
      case "ROTATION TUNER":
        autoSelected = "RotationPID";
      break;
    }

    return new PathPlannerAuto("DrivePID");
  }

  public DriveTrain getDrive(){
    return m_drive;
  }
}
