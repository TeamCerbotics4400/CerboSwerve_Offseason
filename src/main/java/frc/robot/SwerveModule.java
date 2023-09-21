// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import team4400.Util.Swerve.RevModuleOptimizer;
import team4400.Util.Swerve.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    public final int moduleNumber;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final AnalogEncoder absoluteEncoder;

    private final SimpleMotorFeedforward feedForward;

    private final SparkMaxPIDController driveController;
    private final PIDController turnController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;
    
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);

        absoluteEncoderOffset = moduleConstants.angleOffset;
        absoluteEncoderReversed = moduleConstants.absoluteEncoderReversed;
        absoluteEncoder = new AnalogEncoder(moduleConstants.absoluteEncoderID);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(moduleConstants.driveReversed);
        turnMotor.setInverted(moduleConstants.turnReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        feedForward = new SimpleMotorFeedforward(0, 0, 0);

        driveController = driveMotor.getPIDController();

        driveController.setP(0);
        driveController.setI(0);
        driveController.setD(0);
        driveController.setFF(0);

        turnController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        Timer.delay(1.0);
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

    public double getRawAbsoluteAngle(){
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getAbsoluteAngle(){
        double rawAngle = 
                (absoluteEncoder.getAbsolutePosition() * 360 - absoluteEncoderOffset) % 360;
        double angle;
        if(rawAngle > 180.0 && rawAngle < 360.0){
            angle = -180 + rawAngle % 180.0;
        } else {
            angle = rawAngle;
        }

        return angle;
    }

    public double absoluteAngleToRadians(){
        return getAbsoluteAngle() * Math.PI / 180.0;
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        //turnEncoder.setPosition(getAbsolutePos());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = 
                desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 
            0, feedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        turnMotor.set(turnController.calculate(absoluteAngleToRadians(), 
                        desiredState.angle.getRadians()));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        desiredState = 
            RevModuleOptimizer.optimize(desiredState, new Rotation2d(absoluteAngleToRadians()));

        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);  
    }

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