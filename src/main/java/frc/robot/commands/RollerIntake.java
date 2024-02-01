// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerClawConstants;
import frc.robot.subsystems.RollerClaw;


public class RollerIntake extends Command {
  private RollerClaw rollerClaw;
  /** Creates a new RollerIntake. */
  public RollerIntake(RollerClaw rollerClaw) {
    this.rollerClaw = rollerClaw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollerClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollerClaw.setRollerClaw(RollerClawConstants.clawSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerClaw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
