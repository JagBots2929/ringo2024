// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMoverSubsystem extends SubsystemBase {
  CANSparkMax intake;
  CANSparkMax leftMover;
  CANSparkMax rightMover;
  public IntakeMoverSubsystem(int intakeID, int leftMoverID, int rightMoverID) {
    intake = new CANSparkMax(intakeID, MotorType.kBrushless);
    leftMover = new CANSparkMax(leftMoverID, MotorType.kBrushless);
    rightMover = new CANSparkMax(rightMoverID, MotorType.kBrushless);
  }

  public void startIntake() { 
    intake.set(1);
    leftMover.set(1);
    rightMover.set(-1);
  }
  
  public void stopIntake() { 
    intake.set(0);
    leftMover.set(0);
    rightMover.set(0);
  }

}
