// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax leftMotor = 
              new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = 
              new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);
  
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(2);

  private final ArmFeedforward m_feedForward = 
  new ArmFeedforward(
    ArmConstants.kS, 
    ArmConstants.kG, 
    ArmConstants.kV,
    ArmConstants.kA);

  public ArmSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            ArmConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocityRadPerSecond,
              ArmConstants.kMaxAccelerationMetersPerSecondSquared)));

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);

    leftMotor.setSmartCurrentLimit(80);
    rightMotor.setSmartCurrentLimit(80);

    leftMotor.setCANTimeout(0);
    rightMotor.setCANTimeout(0);
    

    m_encoder.setDistancePerRotation(-360.0);

    //m_encoder.reset();

    setGoal(90.3);
  }

  @Override
  public void periodic() {
      super.periodic();

      SmartDashboard.putNumber("Arm Angle", getMeasurement());

      SmartDashboard.putNumber("Left Arm Current", leftMotor.getOutputCurrent());
      SmartDashboard.putNumber("Right Arm Current", rightMotor.getOutputCurrent());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedForward =  m_feedForward.calculate(Units.radiansToDegrees(setpoint.position), 
    setpoint.velocity);

    leftMotor.setVoltage(output + feedForward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    //TODO: Always intialize the arm upright to apply this offset
    //Initializing arm from any other position will give other measruments
    return m_encoder.getDistance() + 47;// + 406; //- 226.5;
  }

  public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
      () -> {
        this.setGoal(position);
        this.enable();
      },
      this);

    return ejecutable;
  }

  //For use in autonomous methods to shoot after the Arm is in position
  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInPosition(){
    return isWithinThreshold(getMeasurement(), 
                             getController().getGoal().position, 
                             ArmConstants.ARM_THRESHOLD);
  }
}
