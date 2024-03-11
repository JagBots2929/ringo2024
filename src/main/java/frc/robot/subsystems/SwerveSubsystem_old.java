// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.driveConstants;
import frc.robot.Constants.moduleIDS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem_old extends SubsystemBase {
  /** Creates a new swerveSubSystem. */
  
    final static SwerveModule frontLeft = new SwerveModule 
    (moduleIDS.kFrontLeftDriveMotorID, 
    moduleIDS.kFrontLeftTurningMotorID, 
    driveConstants.kFrontLeftDriveEncoderReversed, 
    driveConstants.kFrontLeftTurningEncoderReversed, 
    driveConstants.kFrontLeftDriveMagEncoderOffsetRad 
    
   );

   final static SwerveModule frontRight = new SwerveModule 
    (moduleIDS.kFrontRightDriveMotorID, 
    moduleIDS.kFrontRightTurningMotorID, 
    driveConstants.kFrontRightDriveEncoderReversed, 
    driveConstants.kFrontRightTurningEncoderReversed, 
    driveConstants.kFrontRightDriveMagEncoderOffsetRad
    
   );

   final static SwerveModule backLeft = new SwerveModule 
    (moduleIDS.kBackLeftDriveMotorID, 
    moduleIDS.kBackLeftTurningMotorID, 
    driveConstants.kBackLeftDriveEncoderReversed, 
    driveConstants.kBackLeftTurningEncoderReversed, 
    driveConstants.kBackLeftDriveMagEncoderOffsetRad
    );

   final static SwerveModule backRight = new SwerveModule 
    (moduleIDS.kBackRightDriveMotorID, 
    moduleIDS.kBackRightTurningMotorID, 
    driveConstants.kBackRightDriveEncoderReversed, 
    driveConstants.kBackRightTurningEncoderReversed, 
    driveConstants.kBackRightDriveMagEncoderOffsetRad
    
   );
  
  private static SwerveModulePosition[] swervemodposition = new SwerveModulePosition[]
  { 
    frontLeft.getSwerveModPosition(),
    frontRight.getSwerveModPosition(),
    backLeft.getSwerveModPosition(),
    backRight.getSwerveModPosition()


  };
  private static Pigeon2 gyro = new Pigeon2(moduleIDS.gyroID);
  private static SwerveDriveOdometry odometry = new SwerveDriveOdometry(driveConstants.kDriveKinematics, gyro.getRotation2d(), swervemodposition);
  
  public void SwerveSubsystem() {
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();
}


  public void zeroHeading() { 
    gyro.reset();
  }

  public static  double getHeading() { 
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public static Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() { 
    return odometry.getPoseMeters();
  }

  public static void resetOdometry(Pose2d pose) { 
    odometry.resetPosition(getRotation2d(), swervemodposition, pose);
  }
    

  @Override

  public void periodic() { 
    SmartDashboard.putString("pose", getPose().getTranslation().toString());
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("fl", frontLeft.getTurnPosition());
    SmartDashboard.putNumber("fr", frontRight.getTurnPosition());
    SmartDashboard.putNumber("bl", backLeft.getTurnPosition());
    SmartDashboard.putNumber("br", backRight.getTurnPosition());
  }

  public void setState(SwerveModuleState[] desiredStates) { 
    System.out.println("SwerveSubsystem::setState" + desiredStates[0] + " " + desiredStates[1]);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, driveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredStates(desiredStates[0]);
    frontRight.setDesiredStates(desiredStates[1]);
    backLeft.setDesiredStates(desiredStates[2]);
    backRight.setDesiredStates(desiredStates[3]);

  }
}

