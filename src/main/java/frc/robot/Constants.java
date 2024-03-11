// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class driveConstants { 
    

    public static final double kTrackWidth = Units.inchesToMeters(23.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.75 );
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));


    public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final double kFrontLeftDriveMagEncoderOffsetRad = 3.349427; //set these onez when robot is made
        public static final double kBackLeftDriveMagEncoderOffsetRad = 6.302268;
        public static final double kFrontRightDriveMagEncoderOffsetRad = 0.228573;
        public static final double kBackRightDriveMagEncoderOffsetRad = 0.023122;

        
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        

  }

  public static class moduleIDS { 
    public static final int kFrontLeftDriveMotorID = 17;
    public static final int kBackLeftDriveMotorID = 19;
    public static final int kFrontRightDriveMotorID = 4;
    public static final int kBackRightDriveMotorID = 3;

    public static final int kFrontLeftTurningMotorID = 16;
    public static final int kBackLeftTurningMotorID = 18;
    public static final int kFrontRightTurningMotorID = 5;
    public static final int kBackRightTurningMotorID = 2;

    public static final int kIntakeMotorID = 15;
    public static final int kLeftMoverMotorID = 13;
    public static final int kRightMoverMotorID = 6; 
    public static final int kLeftShooterMotorID = 14;
    public static final int kRightShooterMotorID = 21;


    public static final int gyroID = 22;

  }

  public static class moduleConstants { 
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.625);
    public static final double kRubberWheelDiameterMeters = Units.inchesToMeters(2);
    public static final double kDriveMotorGearRatio = 1 / 8.14;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kIntakeMotorGearRatio =  1 / 48;
    public static final double kMoverMotorGearRatio = 1 / 16;
    public static final double kMoverRotattoMeter = kMoverMotorGearRatio * Math.PI * kRubberWheelDiameterMeters;
    public static final double kMoverRPMtoMeterPerSec = kMoverRotattoMeter/60;
    public static final double kIntakeRotattoMeter = kIntakeMotorGearRatio * Math.PI * kRubberWheelDiameterMeters;
    public static final double kIntakeRPMtoMeterPerSec = kIntakeRotattoMeter/60;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 1;

  }



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis= 0;
    public static final int kDriverRotAxis= 4;
    public static final int kDriverFieldOriented = 6;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    public static final double kDeadband = .1;
  }
}
