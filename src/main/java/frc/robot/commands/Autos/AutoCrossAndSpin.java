// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LaunchGroup;
import frc.robot.commands.Paths.AutoDriveTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCrossAndSpin extends SequentialCommandGroup {

  // Drivetrain drivetrain;

  /** Creates a new AutoCrossAndSpin. */
  public AutoCrossAndSpin(Drivetrain drivetrain, Launcher launcher) {
    
    // this.drivetrain = drivetrain; 
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoDriveTime(drivetrain, 3, 0.4, 0), new WaitCommand(1), 
      new AutoDriveTime(drivetrain, 2, 0, -0.4), new WaitCommand(2), new LaunchGroup(launcher));
  }
}
