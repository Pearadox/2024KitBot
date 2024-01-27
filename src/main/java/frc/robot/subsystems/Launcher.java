// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.drivers.PearadoxSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
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

       
  }

  public Command getIntakeCommand() {
    return this.startEnd(
      // sets wheels to intake speed values upon init
      () -> {
        setLaunchWheel(LauncherConstants.launchFeederSpeed);
        setFeedWheel(LauncherConstants.intakeFeederSpeed);
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