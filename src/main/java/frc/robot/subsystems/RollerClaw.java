// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerClawConstants;

public class RollerClaw extends SubsystemBase {
  //maybe change to a VictorSPX later (brushed)
  VictorSPX rollerClaw;
  /** Creates a new RollerClaw. */
  public RollerClaw() {
    rollerClaw = new VictorSPX(RollerClawConstants.rollerClawID);

    rollerClaw.configFactoryDefault();

    rollerClaw.setInverted(true); // now mounted on left

    rollerClaw.setNeutralMode(NeutralMode.Brake);
  }

  public void setRollerClaw(double speed) {
    rollerClaw.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    rollerClaw.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
