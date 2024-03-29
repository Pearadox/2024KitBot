// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
// import edu.wpi.first.wpilibj.Timer;

public class AutoSpin extends Command {

  // initializes drivetrain object
  private Drivetrain drivetrain;

  // intiializes timeSinceStart 
  private long startTime;
  private double stopTime;
  private double rotVelocity;

  /** Creates a new CrossTheLine. */
  public AutoSpin(Drivetrain drivetrain, double stopTime, double rotVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.stopTime = stopTime * 1000;

    this.rotVelocity = rotVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(0.0, rotVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime >= stopTime);
  }
}
