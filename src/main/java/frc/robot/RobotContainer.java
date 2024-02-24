// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDriveTime;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final Drivetrain drivetrain = new Drivetrain();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {      
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
  public void configureBindings() {
    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    // Using bumpers as a modifier and combining it with the buttons so that we can have both sets
    // of bindings at once

    driverController
        .a()
        .and(driverController.rightBumper())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController
        .b()
        .and(driverController.rightBumper())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverController
        .x()
        .and(driverController.rightBumper())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController
        .y()
        .and(driverController.rightBumper())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    driverController
        .a()
        .and(driverController.leftBumper())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController
        .b()
        .and(driverController.leftBumper())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverController
        .x()
        .and(driverController.leftBumper())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController
        .y()
        .and(driverController.leftBumper())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoDriveTime(drivetrain, 1, 0.5, 0);
  }
}