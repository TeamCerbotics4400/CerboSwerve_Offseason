// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import team4400.Util.RevModuleOptimizer;
import team4400.Util.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    public final int moduleNumber;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final SparkMaxAnalogSensor thriftEncoder;

    private final SparkMaxPIDController driveSparkController;
    private final SparkMaxPIDController turnSparkController;

    private final PIDController turnController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
 
    private final SimpleMotorFeedforward feedForward = 
    new SimpleMotorFeedforward(
        ModuleConstants.turnkS, 
        ModuleConstants.turnkV, 
        ModuleConstants.turnkA);

    private Rotation2d lastAngle;
    
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);

        absoluteEncoderOffsetRad = moduleConstants.angleOffset;
        absoluteEncoderReversed = moduleConstants.absoluteEncoderReversed;
        thriftEncoder = turnMotor.getAnalog(Mode.kAbsolute);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(moduleConstants.driveReversed);
        turnMotor.setInverted(moduleConstants.turnReversed);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kCoast);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        thriftEncoder.setPositionConversionFactor(ModuleConstants.kAbsoluteEncoderVolts2Rad);

        driveSparkController = driveMotor.getPIDController();
        turnSparkController = turnMotor.getPIDController();

        driveSparkController.setP(ModuleConstants.drivekP);
        driveSparkController.setI(ModuleConstants.drivekI);
        driveSparkController.setD(ModuleConstants.drivekD);
        driveSparkController.setFF(ModuleConstants.drivekFF);

        turnSparkController.setP(ModuleConstants.turnkP);
        turnSparkController.setI(ModuleConstants.turnkI);
        turnSparkController.setD(ModuleConstants.turnkD);
        turnSparkController.setFF(ModuleConstants.turnkFF);

        turnSparkController.setFeedbackDevice(thriftEncoder);

        turnController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        lastAngle = getState().angle;

        resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
       return turnEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        return thriftEncoder.getPosition();
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = RevModuleOptimizer.optimize(desiredState, getState().angle);
        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);

        SmartDashboard.putString(
            "Swerve [" + moduleNumber + "] state", desiredState.toString());
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / 
                                            DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            //Code for PID Drive
            double velocity = desiredState.speedMetersPerSecond;
            driveSparkController.setReference(velocity, ControlType.kVelocity);
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <=
         DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01) ? lastAngle : desiredState.angle;

        turnSparkController.setReference(angle.getRadians(), ControlType.kPosition);
        lastAngle = angle;
    }

    /*public void setDesiredState(SwerveModuleState state){

        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        state = RevModuleOptimizer.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond 
        / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);//Check when the swerve chassis is done
        turnMotor.set(turnController.calculate(getTurningPosition(), state.angle.getRadians()));

        SmartDashboard.putString(
            "Swerve [" + moduleNumber + "] state", state.toString());
    }*/

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(), 
            new Rotation2d(getTurningPosition()));
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }
}