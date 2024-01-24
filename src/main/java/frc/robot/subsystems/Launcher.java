// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final VictorSPX launchWheel = new VictorSPX(LauncherConstants.launcherID);
  private final VictorSPX feedWheel = new VictorSPX(LauncherConstants.feederID);
  
  /** Creates a new Launcher. */
  public Launcher() {}

  public void setLaunchWheel(double speed) {
    launchWheel.set(ControlMode.PercentOutput, speed);
  }

  public void setFeedWheel(double speed) {
    feedWheel.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    launchWheel.set(ControlMode.PercentOutput, 0);
    feedWheel.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}