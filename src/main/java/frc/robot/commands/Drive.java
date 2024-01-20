// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DrivetrainConstants;

public class Drive extends Command {
  /** Creates a new Drive. */
  private Drivetrain drivetrain;
  private CommandXboxController drivercontroller;
  public Drive(Drivetrain drivetrain, CommandXboxController drivercontroller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain; 
    this.drivercontroller = drivercontroller;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -drivercontroller.getLeftY();
    double turn = -drivercontroller.getRightX(); 

    if (Math.abs(speed) < DrivetrainConstants.deadBan) {
      speed = 0;
    }

    if (Math.abs(turn) < DrivetrainConstants.deadBan) {
      turn = 0;
    }

    drivetrain.arcadeDrive(speed, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
