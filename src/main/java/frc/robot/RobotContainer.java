// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Autos.AutoCrossAndSpin;
import frc.robot.commands.Autos.ShootSpinCross;
import frc.robot.commands.Paths.AutoDriveDistance;
import frc.robot.commands.Paths.AutoDriveTime;
import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final Drivetrain drivetrain = new Drivetrain();
  private final Launcher launcher = new Launcher();
  private final Climber climber = new Climber();
  private final RollerClaw rollerClaw = new RollerClaw();

  private final SendableChooser<Boolean> chooserChooser = new SendableChooser<Boolean>();
  private final SendableChooser<Command> manualChooser = new SendableChooser<Command>();  
  private final SendableChooser<Command> pathPlannerChooser;
  //private final SendableChooser<Boolean> controllerChoose = new SendableChooser<Boolean>();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {  
    drivetrain.setDefaultCommand(new Drive(drivetrain, driverController));
    
    SmartDashboard.putData("Time Chooser", manualChooser);
    manualChooser.setDefaultOption("Auto Spin", new AutoDriveTime(drivetrain, 2, 0, -0.4));
    manualChooser.addOption("Auto Cross And Spin", new AutoCrossAndSpin(drivetrain, launcher));
    manualChooser.addOption("Auto Shoot Spin Cross", new ShootSpinCross(drivetrain, launcher));
    manualChooser.addOption("Launch Group", new LaunchGroup(launcher));
    //manualChooser.addOption("auto 1 (path planner)", new PathPlannerAuto("auto1"));
    manualChooser.addOption("Drive Distance", new AutoDriveDistance(drivetrain, 5, 1, 0));
  
    SmartDashboard.putData("manual or pathplanner", chooserChooser);
    chooserChooser.setDefaultOption("manual", true);
    chooserChooser.addOption("pathplanner", false);
    
    configureBindings();
    
    pathPlannerChooser = AutoBuilder.buildAutoChooser("auto1");
    SmartDashboard.putData("path planner chooser", pathPlannerChooser);
    }

  public void configureBindings() {
    // when operator holds right bumper, run PrepareLaunch for 1 sec, then run LaunchNote
    operatorController.rightBumper().whileTrue(new LaunchGroup(launcher));

    // intakes when operator holds left bumper
    operatorController.leftBumper().whileTrue(new Intake(launcher));
    
    // climbs up when operator holds dpad up
    operatorController.povUp().whileTrue(new ClimbUp(climber));
    // climbs down when operator holds dpad down
    operatorController.povDown().whileTrue(new ClimbDown(climber));

    // intakes with roller when x button is pressed
    operatorController.x().whileTrue(new RollerIntake(rollerClaw));
    // shoots with roller when b button is pressed
    operatorController.b().whileTrue(new RollerLaunch(rollerClaw));
    
    // when driver holds roight bumper, run PrepareLaunch for 1 sec, then run LaunchNote
    driverController.rightBumper().whileTrue(new LaunchGroup(launcher));

    // intakes when driver holds left bumper
    driverController.leftBumper().whileTrue(new Intake(launcher));
    
    // climbs up when driver holds dpad up
    driverController.povUp().whileTrue(new ClimbUp(climber));
    // climbs down when driver holds dpad down
    driverController.povDown().whileTrue(new ClimbDown(climber));

    // intakes with roller when x button is pressed
    driverController.x().whileTrue(new RollerIntake(rollerClaw));
    // shoots with roller when b button is pressed
    driverController.b().whileTrue(new RollerLaunch(rollerClaw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // returns either manualChooser or pathPlannerChooser depending on the value of the chooserChooser
    return chooserChooser.getSelected() ? manualChooser.getSelected() : pathPlannerChooser.getSelected();
  }
}