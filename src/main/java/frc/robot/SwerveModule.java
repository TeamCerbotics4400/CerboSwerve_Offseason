// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
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

    private final PIDController turnController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    
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

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turnController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

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
        double angle = thriftEncoder.getVoltage() / RobotController.getVoltage3V3();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){

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