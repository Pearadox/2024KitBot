// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.lib.drivers.PearadoxSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  DifferentialDrive m_drivetrain;

  public Drivetrain() {
    PearadoxSparkMax leftFront = new PearadoxSparkMax(DrivetrainConstants.LEFT_FRONT_ID, 
      MotorType.kBrushless, IdleMode.kCoast, DrivetrainConstants.DRIVE_CURRENT_LIMIT, true);
    PearadoxSparkMax rightFront = new PearadoxSparkMax(DrivetrainConstants.RIGHT_FRONT_ID, 
      MotorType.kBrushless, IdleMode.kCoast, DrivetrainConstants.DRIVE_CURRENT_LIMIT, false);
    PearadoxSparkMax leftBack = new PearadoxSparkMax(DrivetrainConstants.LEFT_BACK_ID, 
      MotorType.kBrushless, IdleMode.kCoast, DrivetrainConstants.DRIVE_CURRENT_LIMIT, true);
    PearadoxSparkMax rightBack = new PearadoxSparkMax(DrivetrainConstants.RIGHT_FRONT_ID, 
      MotorType.kBrushless, IdleMode.kCoast, DrivetrainConstants.DRIVE_CURRENT_LIMIT, false);
    
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  public void arcadeDrive(double throttle, double twist) {
    m_drivetrain.arcadeDrive(throttle, twist);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
