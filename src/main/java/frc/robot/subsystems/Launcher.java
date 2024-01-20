// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.lib.drivers.PearadoxSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  PearadoxSparkMax m_launchWheel;
  PearadoxSparkMax m_feedWheel;

  /** Creates a new Launcher. */
  public Launcher() {
    m_launchWheel = new PearadoxSparkMax(LauncherConstants.LAUNCHER_ID, 
      MotorType.kBrushed, IdleMode.kCoast, LauncherConstants.LAUNCHER_CURRENT_LIMIT, false);
    m_feedWheel = new PearadoxSparkMax(LauncherConstants.FEEDER_ID, 
      MotorType.kBrushed, IdleMode.kCoast, LauncherConstants.FEEDER_CURRENT_LIMIT, false);
  }

  public Command getIntakeCommand() {
    return this.startEnd(
      // sets wheels to intake speed values upon init
      () -> {
        setLaunchWheel(LauncherConstants.LAUNCH_FEEDER_SPEED);
        setFeedWheel(LauncherConstants.INTAKE_FEEDER_SPEED);
      },
      // stop wheels when command stops
      () -> { stop(); });
  }

  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
  }

  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
