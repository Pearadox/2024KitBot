// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchGroup extends SequentialCommandGroup {
  /** Creates a new LaunchGroup. */
  public LaunchGroup(Launcher launcher) {
    // Add your commands in the addCommands() call, e.g.

    // TODO: Have you tested this fully in teleop?  This was structured in a way that it fit in robotContainer,
    //       but it doesn't quite translate to a command group like this.

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PrepearLaunch(launcher)
    .withTimeout(LauncherConstants.launcherDelay)
    .andThen(new LaunchNote(launcher))
    .handleInterrupt(() -> launcher.stop())
    .withTimeout(2)); // change? to do: make constant
  }
}
