// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;

public class DriveTrain extends SubsystemBase {

  DifferentialDrive drivetrain;
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    PearadoxSparkMax leftFront = new PearadoxSparkMax(DriveTrainConstants.leftFrontID, MotorType.kBrushless, IdleMode.kCoast,45, true);
    PearadoxSparkMax leftBack = new PearadoxSparkMax(DriveTrainConstants.leftBackID, MotorType.kBrushless, IdleMode.kCoast,45, true);
    PearadoxSparkMax rightFront = new PearadoxSparkMax(DriveTrainConstants.rightFrontID, MotorType.kBrushless, IdleMode.kCoast,45, false);
    PearadoxSparkMax rightBack = new PearadoxSparkMax(DriveTrainConstants.rightBackID, MotorType.kBrushless, IdleMode.kCoast,45, false);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);


    drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  public void arcadeDrive(double speed, double rotation) {
    drivetrain.arcadeDrive(speed,rotation);
  }
  
  

  @Override
  public void periodic() {

}
}

