// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants;
// import frc.robot.Constants.*;

public class AutoCrossTheLine extends Command {

  // initializes drivetrain object
  private Drivetrain drivetrain;

  // intiializes timeSinceStart 
  private Timer timeSinceStart = new Timer();

  private double stopTime = 2;

  /** Creates a new CrossTheLine. */
  public AutoCrossTheLine(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceStart.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(0.3, 0.0);    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0,0.0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {      
    return (timeSinceStart.get() >= stopTime);
  }
}
