// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Autos.AutoCrossAndSpin;
import frc.robot.commands.Autos.ShootSpinCross;
import frc.robot.commands.Paths.AutoDriveDistance;
import frc.robot.commands.Paths.AutoDriveTime;
import frc.robot.subsystems.*;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
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

    driverController.start().onTrue(
      new RunCommand(() -> drivetrain.resetOdometry(new Pose2d()), drivetrain)
      .andThen(new RunCommand(() -> drivetrain.resetGyro(), drivetrain)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // returns either manualChooser or pathPlannerChooser depending on the value of the chooserChooser
    //return chooserChooser.getSelected() ? manualChooser.getSelected() : pathPlannerChooser.getSelected();

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DrivetrainConstants.kS,
                DrivetrainConstants.kV,
                DrivetrainConstants.kA),
            DrivetrainConstants.kinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                0.5,
                0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(DrivetrainConstants.kRamseteB, 
                DrivetrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DrivetrainConstants.kS,
                DrivetrainConstants.kV,
                DrivetrainConstants.kA),
            DrivetrainConstants.kinematics,
            drivetrain::getCurrentSpeeds,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::driveVolts,
            drivetrain);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.runOnce(() -> drivetrain.resetOdometry(exampleTrajectory.getInitialPose()))
        .andThen(ramseteCommand)
        .andThen(Commands.runOnce(() -> drivetrain.driveVolts(0, 0)));
  }
}