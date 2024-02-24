// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private final PearadoxSparkMax leftFront = new PearadoxSparkMax(DrivetrainConstants.leftFrontID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, true);
      
  private final PearadoxSparkMax rightFront = new PearadoxSparkMax(DrivetrainConstants.rightFrontID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, false);
  
  private final PearadoxSparkMax leftBack = new PearadoxSparkMax(DrivetrainConstants.leftBackID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, true, leftFront, 0);
  
  private final PearadoxSparkMax rightBack = new PearadoxSparkMax(DrivetrainConstants.rightBackID, 
    MotorType.kBrushless, PearadoxSparkMax.IdleMode.kBrake, DrivetrainConstants.limit, false, rightFront, 0);
  
  private DifferentialDriveOdometry odometry;
  private AHRS gyro = new AHRS(Port.kMXP);

  private final RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBack.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBack.getEncoder();

  // // remove below after sysid characterization
  // // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  // private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  // private final MutableMeasure<Distance> distance = mutable(Meters.of(0));
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  // private final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));

  /** Creates a new Drivetrain. */
  DifferentialDrive m_drivetrain;

  // TODO: Integrate foundational code necessary for trajectory/path following
  // Helpful references:
  // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/index.html <--
  // https://github.com/Pearadox/2023Everybot/blob/master/src/main/java/frc/robot/subsystems/DriveTrain.java
  // https://github.com/Pearadox/2023Everybot/blob/master/src/main/java/frc/robot/Constants.java

  public Drivetrain() {
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    // m_drivetrain.setSafetyEnabled(false); // need this for sysid characterization
    
    // 6 inch wheel, 10.71:1 gear ratio
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    leftBackEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    rightBackEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);

    leftFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    rightFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    leftBackEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    rightBackEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);

    // docs: https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
    // TODO: fix below; code crashes when path planner auto is selected & some of the commands are not even run
    
    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // Current ChassisSpeeds supplier
      this::drive, // Method that will drive the robot given ChassisSpeeds
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
        // boolean that controls which alliance the robot is on
        System.out.println("called replanning config method");
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) { // is working
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }



// // TODO: remove below after sysid characterization
// private final SysIdRoutine sysIdRoutine =
//   new SysIdRoutine(
//     // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//     new SysIdRoutine.Config(),
//     new SysIdRoutine.Mechanism(
//         // Tell SysId how to plumb the driving voltage to the motors.
//         (Measure<Voltage> volts) -> {
//           leftFront.setVoltage(volts.in(Volts));
//           rightFront.setVoltage(volts.in(Volts));
//         },
//         // Tell SysId how to record a frame of data for each motor on the mechanism being
//         // characterized.
//         log -> {
//           // Record a frame for the left motors.  Since these share an encoder, we consider
//               // the entire group to be one motor.
//               log.motor("drive-left")
//                   .voltage(
//                       appliedVoltage.mut_replace(
//                           leftFront.getAppliedOutput() * leftFront.getBusVoltage(), Volts))
//                   .linearPosition(distance.mut_replace(leftFrontEncoder.getPosition(), Meters))
//                   .linearVelocity(velocity.mut_replace(leftFrontEncoder.getVelocity(), MetersPerSecond));
//               // Record a frame for the right motors.  Since these share an encoder, we consider
//               // the entire group to be one motor.
//               log.motor("drive-right")
//                   .voltage(
//                       appliedVoltage.mut_replace(
//                           rightFront.getAppliedOutput() * rightFront.getBusVoltage(), Volts))
//                   .linearPosition(distance.mut_replace(rightFrontEncoder.getPosition(), Meters))
//                   .linearVelocity(velocity.mut_replace(rightFrontEncoder.getVelocity(), MetersPerSecond));
//         },
//         // Tell SysId to make generated commands require this subsystem, suffix test state in
//         // WPILog with this subsystem's name ("drive")
//         this));

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.dynamic(direction);
  // }


  
  public void driveVolts(double leftVolts, double rightVolts) {
    System.out.println("called drive volts");
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    m_drivetrain.feed(); // motor safety thing
  }
  
  public void arcadeDrive(double throttle, double twist) {
    m_drivetrain.arcadeDrive(throttle, twist);
  }

  // for PathPlanner
  public void drive(ChassisSpeeds chassisSpeeds) { 
    // done: convert meters per second to percentage (-1 to 1) or voltage
    System.out.println("called drive method");
    SmartDashboard.putString("status1", "driving"); //NO RETURN
    DifferentialDriveWheelSpeeds speeds = DrivetrainConstants.kinematics.toWheelSpeeds(chassisSpeeds);
    double left = speeds.leftMetersPerSecond  / DrivetrainConstants.maxSpeed;
    double right = speeds.rightMetersPerSecond / DrivetrainConstants.maxSpeed;
    m_drivetrain.tankDrive((Math.abs(left) <= 1) ? left : 0, (Math.abs(right) <= 1) ? right : 0);
    // leftFront.setVoltage(speeds.leftMetersPerSecond);
  }

  public RelativeEncoder getEncoder() {
    return leftFront.getEncoder();
  }

  // for PathPlanner
  public Pose2d getPose(){ 
    System.out.println("called get pose method");
    SmartDashboard.putString("status2", "got pose"); // NOT WORKING
    return odometry.getPoseMeters();
  }

  // for PathPlanner
public ChassisSpeeds getRobotRelativeSpeeds() { 
  System.out.println("called get robot relative speeds method");
  SmartDashboard.putString("status3", "got robot relative speeds"); // NOT WORKING
  DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
    leftFrontEncoder.getVelocity(), - rightFrontEncoder.getVelocity());
    return DrivetrainConstants.kinematics.toChassisSpeeds(wheelSpeeds);
  }
  
  // for PathPlanner
public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
  System.out.println("called get current speeds methods");
  return new DifferentialDriveWheelSpeeds(leftFrontEncoder.getVelocity(),
    - rightFrontEncoder.getVelocity());
  }
  
  // for PathPlanner
public void resetOdometry(Pose2d pose){
  System.out.println("called reset odometry method");
  SmartDashboard.putString("status4", "reset odometry"); // IS WORKING
  resetEncoders();
  odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()), 
    leftFrontEncoder.getPosition(), -rightFrontEncoder.getPosition(), pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Front", leftFront.getOutputCurrent());
    SmartDashboard.putNumber("Right Front", rightFront.getOutputCurrent());
    SmartDashboard.putNumber("Left Back", leftBack.getOutputCurrent());
    SmartDashboard.putNumber("Right Back", rightBack.getOutputCurrent());

    //double sensorPosition = Encoder.get() * DrivetrainConstants.encoderConversionFactor;
    odometry.update(
        gyro.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
  }

  public double getDistance() {
    return (leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition() + 
      leftBackEncoder.getPosition() + rightBackEncoder.getPosition()) / 4.0;
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }
}
