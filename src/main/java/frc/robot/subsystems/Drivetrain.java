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

  /** Creates a new Drivetrain. */
  DifferentialDrive m_drivetrain;

  // TODO: Integrate foundational code necessary for trajectory/path following
  // Helpful references:
  // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/index.html <--
  // https://github.com/Pearadox/2023Everybot/blob/master/src/main/java/frc/robot/subsystems/DriveTrain.java
  // https://github.com/Pearadox/2023Everybot/blob/master/src/main/java/frc/robot/Constants.java

  public Drivetrain() {
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
    
    // 6 inch wheel, 10.71:1 gear ratio
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    leftBackEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    rightBackEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);

    leftFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    rightFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    leftBackEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    rightBackEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 
      0.0, 0.0, 
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    // docs: https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
    // TODO: fix below; code crashes when path planner auto is selected & some of the commands are not even run
    
    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // Current ChassisSpeeds supplier
      this::chassisSpeedDrive, // Method that will drive the robot given ChassisSpeeds
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

  public void arcadeDrive(double throttle, double twist) {
    m_drivetrain.arcadeDrive(throttle, twist);
  }

  // for PathPlanner
  public void chassisSpeedDrive(ChassisSpeeds chassisSpeeds) { 
    // done: convert meters per second to percentage (-1 to 1) or voltage
    DifferentialDriveWheelSpeeds speeds = DrivetrainConstants.kinematics.toWheelSpeeds(chassisSpeeds);
    double left = speeds.leftMetersPerSecond  / DrivetrainConstants.maxSpeed;
    double right = speeds.rightMetersPerSecond / DrivetrainConstants.maxSpeed;
    m_drivetrain.tankDrive(left, right);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    m_drivetrain.feed(); // motor safety thing
  }

  // for PathPlanner
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // for PathPlanner
  public ChassisSpeeds getRobotRelativeSpeeds() { 
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
      leftFrontEncoder.getVelocity(), rightFrontEncoder.getVelocity());
      return DrivetrainConstants.kinematics.toChassisSpeeds(wheelSpeeds);
  }
    
  // for PathPlanner
  public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFrontEncoder.getVelocity(),
      rightFrontEncoder.getVelocity());
  }
    
  // for PathPlanner
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()), 
      leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition(), pose);
    m_drivetrain.feed();
  }
    
  public RelativeEncoder getEncoder() {
    return leftFront.getEncoder();
  }
  
  public double getDistance() {
  return (leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition() + 
    leftBackEncoder.getPosition() + rightBackEncoder.getPosition()) / 4.0;
  }    
  
  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }
  
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Left Front", leftFront.getOutputCurrent());
    // SmartDashboard.putNumber("Right Front", rightFront.getOutputCurrent());
    // SmartDashboard.putNumber("Left Back", leftBack.getOutputCurrent());
    // SmartDashboard.putNumber("Right Back", rightBack.getOutputCurrent());

    SmartDashboard.putNumber("pose x", getPose().getX());
    SmartDashboard.putNumber("pose y", getPose().getY());
    SmartDashboard.putNumber("pose theta", getPose().getRotation().getDegrees());

    System.out.println(getPose().getX() + ", " + 
      getPose().getY() + ", " + getPose().getRotation().getDegrees());

    //double sensorPosition = Encoder.get() * DrivetrainConstants.encoderConversionFactor;
    odometry.update(
        gyro.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
  }
}