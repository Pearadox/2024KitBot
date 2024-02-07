// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private final PearadoxSparkMax leftFront = new PearadoxSparkMax(DrivetrainConstants.leftFrontID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, true);
      
  private final PearadoxSparkMax rightFront = new PearadoxSparkMax(DrivetrainConstants.rightFrontID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, false);
  
  private final PearadoxSparkMax leftBack = new PearadoxSparkMax(DrivetrainConstants.leftBackID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, true, leftFront, 0);
  
  private final PearadoxSparkMax rightBack = new PearadoxSparkMax(DrivetrainConstants.rightBackID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, false, rightFront, 0);
  
  /** Creates a new Drivetrain. */
  DifferentialDrive m_drivetrain;

  public Drivetrain() {
    // leftBack.follow(leftFront);
    // rightBack.follow(rightFront);

    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
    
    // 6 inch wheel, 10.71:1 gear ratio
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);    
  }

  public void arcadeDrive(double throttle, double twist) {
    m_drivetrain.arcadeDrive(throttle, twist);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Front", leftFront.getOutputCurrent());
    SmartDashboard.putNumber("Left Back", leftBack.getOutputCurrent());
    SmartDashboard.putNumber("Right Front", rightFront.getOutputCurrent());
    SmartDashboard.putNumber("Right Back", rightBack.getOutputCurrent());
  }

  public double getDistance() {
    return leftFrontEncoder.getPosition(); // tp do: use right encoder
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
  }
}
