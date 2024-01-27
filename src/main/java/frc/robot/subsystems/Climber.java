// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final PearadoxSparkMax climber = new PearadoxSparkMax(ClimberConstants.climberID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kCoast, ClimberConstants.climberLimit, true);
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
