// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Define Joysticks
  private final LogitechPro joyStick = new LogitechPro(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driverJoystick = new XboxController(0);

  // Define subsystems
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(joyStick);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Define triggers
    Trigger xButton = new JoystickButton(driverJoystick, Constants.OIConstants.kXButton);
    Trigger yButton = new JoystickButton(driverJoystick, Constants.OIConstants.kYButton);
    Trigger aButton = new JoystickButton(driverJoystick, Constants.OIConstants.kAButton);
    Trigger bButton = new JoystickButton(driverJoystick, Constants.OIConstants.kBButton);
    Trigger leftBumper = new JoystickButton(driverJoystick, Constants.OIConstants.kLeftBumper);
    Trigger rightBumper = new JoystickButton(driverJoystick, Constants.OIConstants.kRightBumper);
    Trigger startButton = new JoystickButton(driverJoystick, Constants.OIConstants.kStartButton);

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> joyStick.getBackToFront(),
        () -> joyStick.getLeftToRight(),
        () -> joyStick.getYaw(),
        () -> swerveSubsystem.throttleAdjust(joyStick.getThrottle()),
        () -> joyStick.getTrigger()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    aButton.onTrue(new SwerveControllerCommand(
        swerveSubsystem.getGoToTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(1, 0, Rotation2d.fromDegrees(45))),
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        swerveSubsystem.xController,
        swerveSubsystem.yController,
        swerveSubsystem.thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem));
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
