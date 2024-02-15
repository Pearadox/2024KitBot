// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LaunchGroup;
import frc.robot.commands.Paths.AutoDrive;
import frc.robot.commands.Paths.AutoSpin;
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
    addCommands(
      new LaunchGroup(launcher),
      new WaitCommand(2),
      new AutoDrive(drivetrain, 3, -0.4),
      new WaitCommand(2),
      new AutoSpin(drivetrain, 1.414, 0.4),
      new WaitCommand(2),
      new AutoDrive(drivetrain, 2, 0.4)

    );
    
    // DONE: change stopTimes before next time

    // DONE: LaunchGroup() depends on an interrupt and interrupts don't naturally happen during auton, 
    //       so the launcher will just continue running.  You should stop it from running before running your next commands.

    // DONE: Spinning based on time is unpredictable, so if you do it before crossing the line you may not be pointing in
    //       your intended direction.  Consider spinning after driving across the line.  (Remember you can drive backwards.)
  }
}
