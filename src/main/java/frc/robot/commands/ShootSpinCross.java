// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSpinCross extends SequentialCommandGroup {
  /** Creates a new ShootSpinCross. */
  public ShootSpinCross(Drivetrain drivetrain, Launcher launcher) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LaunchGroup(launcher), new WaitCommand(2),
     new AutoSpin(drivetrain, 1), new WaitCommand(2), new AutoCrossTheLine(drivetrain, 1));
     // change stopTimes before next time
  }
}
