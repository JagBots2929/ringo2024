// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.moduleConstants;




import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  CANSparkMax driveMotor;
  CANSparkMax turnMotor;

  RelativeEncoder driveEncoder;
  RelativeEncoder turnEncoder;

PIDController PIDCont;


  private double magOffSet;
  
  private boolean turnReversed;

  public SwerveModule(int driveID, int turnID, boolean driveReversed, boolean turnReversed, double magOffSet) 
  {
    
    this.magOffSet = magOffSet;
    this.turnReversed = turnReversed;
    

    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveMotor.setInverted(driveReversed);
    turnMotor.setInverted(turnReversed);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    turnEncoder.setPositionConversionFactor(moduleConstants.kTurningEncoderRot2Rad);
    turnEncoder.setVelocityConversionFactor(moduleConstants.kTurningEncoderRPM2RadPerSec);
    driveEncoder.setPositionConversionFactor(moduleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(moduleConstants.kDriveEncoderRPM2MeterPerSec);

    PIDCont = new PIDController(moduleConstants.kPTurning ,0 , 0);
    PIDCont.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    
  }

  public double getTurnPosition() 
  { 
    return turnEncoder.getPosition();
  }

  public double getTurnVelocity() 
  { 
    return turnEncoder.getVelocity();

  }

  public double getDrivePosition() 
  { 
  return driveEncoder.getPosition();
  }

  public double getDriveVelocity() 
  { 
   return driveEncoder.getVelocity();
  }

public double getWheelRad() { 
  double angle = getTurnPosition();
  // angle -= magOffSet;
  return angle * (turnReversed ? -1.0 : 1.0);
}

public SwerveModuleState getSwerveModuleState() { 
  return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getWheelRad()));
}
public SwerveModulePosition getSwerveModPosition () {
  return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getWheelRad()));
  
}

public void setDesiredStates(SwerveModuleState state) { 
  if ((Math.abs(state.speedMetersPerSecond)) < .001) { 
    stop();
    return;
  }
  state = SwerveModuleState.optimize(state, getSwerveModuleState().angle);
  driveMotor.set(state.speedMetersPerSecond/driveConstants.kPhysicalMaxSpeedMetersPerSecond);
  turnMotor.set(PIDCont.calculate(getWheelRad(), state.angle.getRadians()));
  //turnMotor.set(PIDCont.calculate(getWheelRad(), state.angle.getRadians()));
  //PIDCont.calculate(getTurnPosition(), state.angle.getRadians())
}

public void resetEncoders() { 
  driveMotor.set(0);
  turnEncoder.setPosition(0);
}

public void stop() { 
  driveMotor.set(0);
  turnMotor.set(0);
}
}

