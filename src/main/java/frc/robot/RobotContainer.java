// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem_old;
import frc.robot.subsystems.IntakeMoverSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.moduleConstants;
import frc.robot.Constants.moduleIDS;
import frc.robot.commands.JoyBoy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final SwerveSubsystem_old swervesub = new SwerveSubsystem_old();
    private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  // private final Joystick joyBoy = new Joystick(OperatorConstants.kDriverControllerPort);
  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final ShooterSubsystem shooter = new ShooterSubsystem(moduleIDS.kLeftShooterMotorID, moduleIDS.kRightShooterMotorID);
  private final IntakeMoverSubsystem intake = new IntakeMoverSubsystem(moduleIDS.kIntakeMotorID, moduleIDS.kLeftMoverMotorID, moduleIDS.kRightMoverMotorID);

  
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // swervesub.setDefaultCommand(new JoyBoy(
    //     swervesub,
    //     () -> -joyBoy.getRawAxis(OperatorConstants.kDriverYAxis),
    //     () -> joyBoy.getRawAxis(OperatorConstants.kDriverXAxis),
    //     () -> joyBoy.getRawAxis(OperatorConstants.kDriverRotAxis),
    //     () -> !joyBoy.getRawButton(OperatorConstants.kDriverFieldOriented)
    //   ));

    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY());

    m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

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
    // new JoystickButton(joyBoy, 1).onTrue(swervesub.runOnce(swervesub::zeroHeading));
    // new JoystickButton(joyBoy, 2).onTrue(intake.runOnce(intake::startIntake));
    // new JoystickButton(joyBoy, 3).onTrue(intake.runOnce(intake::stopIntake));
    // new JoystickButton(joyBoy, 4).onTrue(shooter.runOnce(shooter::startShooter));
    // new JoystickButton(joyBoy, 5).onTrue(shooter.runOnce(shooter::stopShooter));
    // new JoystickButton(joyBoy, 7).onTrue(shooter.runOnce(shooter::recieveRing));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // An example command will be run in autonomous
    
  }
}
