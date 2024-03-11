// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax leftShooter;
  CANSparkMax rightShooter;

  public ShooterSubsystem(int leftShooterID, int rightShooterID) 
  {
   leftShooter = new CANSparkMax(leftShooterID, MotorType.kBrushless);
   rightShooter = new CANSparkMax(rightShooterID, MotorType.kBrushless);
  }

  public void startShooter() { 
    leftShooter.set(1);
    rightShooter.set(-1);
  }

  public void stopShooter() { 
    leftShooter.set(0);
    rightShooter.set(0);
  }

  public void recieveRing() { 
    leftShooter.set(-.1);
    rightShooter.set(.1);

  }
}
