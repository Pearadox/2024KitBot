// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  VictorSPX m_launchWheel;
  VictorSPX m_feedWheel;

  /** Creates a new Launcher. */
  public Launcher() {
    // m_launchWheel = new PearadoxSparkMax(LauncherConstants.launcherID, 
    //    MotorType.kBrushed, PearadoxSparkMax.IdleMode.kCoast, LauncherConstants.launcherCurrentLimit, false);
    //  m_feedWheel = new PearadoxSparkMax(LauncherConstants.feederID, 
    //    MotorType.kBrushed, PearadoxSparkMax.IdleMode.kCoast, LauncherConstants.feederCurrentLimit, false);
      
    m_launchWheel = new VictorSPX(LauncherConstants.launcherID);
    m_feedWheel = new VictorSPX(LauncherConstants.feederID);

    m_launchWheel.configFactoryDefault();
    m_feedWheel.configFactoryDefault();

    m_launchWheel.setInverted(false);
    m_feedWheel.setInverted(false);

    m_launchWheel.setNeutralMode(NeutralMode.Coast);
    m_feedWheel.setNeutralMode(NeutralMode.Coast);

    // to do -- figure out current limit, burn flash (not on victor spx?)
  }

  public void setLaunchWheel(double speed) {
    m_launchWheel.set(ControlMode.PercentOutput, speed);
  }

  public void setFeedWheel(double speed) {
    m_feedWheel.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    m_launchWheel.set(ControlMode.PercentOutput, 0);
    m_feedWheel.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}