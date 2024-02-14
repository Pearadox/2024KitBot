// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Paths;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants;
// import frc.robot.Constants.*;

public class AutoDrive extends Command {

  // initializes drivetrain object
  private Drivetrain drivetrain;
  private long startTime;
  private double stopTime;
  private double velocity;

  // intiializes timeSinceStart 

  /** Creates a new CrossTheLine. */
  public AutoDrive(Drivetrain drivetrain, double stopTime, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.stopTime = stopTime * 1000;
    this.velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timeSinceStart.restart();
    drivetrain.resetEncoders();
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(velocity, 0.0);    
    SmartDashboard.putNumber("Encoder", drivetrain.getDistance());
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0,0.0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {      
    return (System.currentTimeMillis() - startTime >= stopTime); //change 
  }
}