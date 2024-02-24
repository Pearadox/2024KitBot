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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

  //private final SendableChooser<SendableChooser<Command>> chooserChooser = new SendableChooser<SendableChooser<Command>>();

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
    
    //SmartDashboard.putData("Number of Controllers", controllerChoose);
    //controllerChoose.setDefaultOption("Driver + Operator", true);
    //controllerChoose.addOption("Driver Only", false);

    SmartDashboard.putData("manual or pathplanner", chooserChooser);
    chooserChooser.setDefaultOption("manual", true);
    chooserChooser.addOption("pathplanner", false);
    
    configureBindings();    
    
    pathPlannerChooser = AutoBuilder.buildAutoChooser("auto1");
    SmartDashboard.putData("path planner chooser", pathPlannerChooser);
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {
    // DONE: bind all the operator buttons also on the driver controller (no need for conditionals, just duplicate all the bindings)
    // if (controllerChoose.getSelected()) {
      // when operator holds roight bumper, run PrepareLaunch for 1 sec, then run LaunchNote
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
      
    // } else { 
      // must restart robot for changes to occur
      
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
    // }

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    // Using bumpers as a modifier and combining it with the buttons so that we can have both sets
    // of bindings at once

    // driverController
    //     .a()
    //     .and(driverController.rightBumper())
    //     .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController
    //     .b()
    //     .and(driverController.rightBumper())
    //     .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController
    //     .x()
    //     .and(driverController.rightBumper())
    //     .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController
    //     .y()
    //     .and(driverController.rightBumper())
    //     .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // driverController
    //     .a()
    //     .and(driverController.leftBumper())
    //     .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController
    //     .b()
    //     .and(driverController.leftBumper())
    //     .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController
    //     .x()
    //     .and(driverController.leftBumper())
    //     .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController
    //     .y()
    //     .and(driverController.leftBumper())
    //     .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    // returns either manualChooser or pathPlannerChooser depending on the value of the chooserChooser
    return chooserChooser.getSelected() ? manualChooser.getSelected() : pathPlannerChooser.getSelected();
  }
}