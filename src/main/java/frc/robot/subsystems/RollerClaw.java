// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.RollerClawConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;

public class RollerClaw extends SubsystemBase {
  private final PearadoxSparkMax rollerClaw = new PearadoxSparkMax(RollerClawConstants.rollerClawID, 
    MotorType.kBrushed, PearadoxSparkMax.IdleMode.kCoast, RollerClawConstants.rollerClawLimit, false);
  /** Creates a new RollerClaw. */
  public RollerClaw() {}

  public void setClimber(double speed) {
    rollerClaw.set(speed);
  }

  public void stop() {
    rollerClaw.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
