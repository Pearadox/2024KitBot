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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  private Pose2d pose2d;
  private AHRS gyro = new AHRS(Port.kMXP);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
  
  private final PIDController leftPIDController = new PIDController(0.01, 0, 0);
  private final PIDController rightPIDController = new PIDController(0.01, 0, 0);

  // private final SlewRateLimiter speedLimiter = new SlewRateLimiter(DrivetrainConstants.maxSpeed);
  // private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularSpeed / 5.0);

  private final RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBack.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBack.getEncoder();

  /** Creates a new Drivetrain. */
  DifferentialDrive drivetrain;
  // TODO: Integrate foundational code necessary for trajectory/path following
  // Helpful references:
  // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/index.html <--
  // https://github.com/Pearadox/2023Everybot/blob/master/src/main/java/frc/robot/subsystems/DriveTrain.java
  // https://github.com/Pearadox/2023Everybot/blob/master/src/main/java/frc/robot/Constants.java

  public Drivetrain() {
    drivetrain = new DifferentialDrive(leftFront, rightFront);

    drivetrain.setSafetyEnabled(false); // TODO: test??
    
    // 6 inch wheel, 10.71:1 gear ratio
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    leftBackEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);
    rightBackEncoder.setPositionConversionFactor(DrivetrainConstants.encoderConversionFactor);

    leftFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    rightFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    leftBackEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);
    rightBackEncoder.setVelocityConversionFactor(DrivetrainConstants.encoderConversionFactor / 60.0);

    odometry = new DifferentialDriveOdometry(new Rotation2d(-gyro.getAngle()),
      0.0, 0.0, 
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    // docs: https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project    
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
    // m_drivetrain.arcadeDrive(throttle, twist);  
    // throttle = speedLimiter.calculate(throttle) * DrivetrainConstants.maxSpeed;
    // twist = rotLimiter.calculate(twist) * DrivetrainConstants.maxAngularSpeed;  
    throttle *= DrivetrainConstants.maxSpeed;
    twist *= DrivetrainConstants.maxAngularSpeed;  
    var wheelSpeeds = new ChassisSpeeds(throttle, 0.0, twist);
    SmartDashboard.putNumber("throttle", throttle);
    SmartDashboard.putNumber("twist", twist);
    
    chassisSpeedDrive(wheelSpeeds);
  }

  public void chassisSpeedDrive(ChassisSpeeds chassisSpeeds) { 
    // done: convert meters per second to percentage (-1 to 1) or voltage
    DifferentialDriveWheelSpeeds speeds = DrivetrainConstants.kinematics.toWheelSpeeds(chassisSpeeds);
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = 
      leftPIDController.calculate(leftFrontEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput = 
      rightPIDController.calculate(rightFrontEncoder.getVelocity(), speeds.rightMetersPerSecond);
    
    // double left = speeds.leftMetersPerSecond  / DrivetrainConstants.maxSpeed;
    // double right = speeds.rightMetersPerSecond / DrivetrainConstants.maxSpeed;
    driveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
    System.out.println("left: " + (leftOutput + leftFeedforward) + " right: " + (rightOutput + rightFeedforward));
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    assert leftVolts < leftFront.getBusVoltage();
    assert rightVolts < rightFront.getBusVoltage();
    SmartDashboard.putNumber("leftVolts", leftVolts);
    SmartDashboard.putNumber("rightVolts", rightVolts);
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    // leftBack.setVoltage(leftVolts);
    // rightBack.setVoltage(rightVolts);
    drivetrain.feed(); // motor safety thing
  }

  public Pose2d getPose() {
    drivetrain.feed();
    return pose2d;
    //return odometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() { 
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
      leftFrontEncoder.getVelocity(), rightFrontEncoder.getVelocity());
    return DrivetrainConstants.kinematics.toChassisSpeeds(wheelSpeeds);
  }
    
  public DifferentialDriveWheelSpeeds getCurrentSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFrontEncoder.getVelocity() / 60,
      rightFrontEncoder.getVelocity() / 60);
  }
    
  // for PathPlanner
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), 
      leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition(), pose);
    drivetrain.feed();
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

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    pose2d = odometry.update(
        new Rotation2d(-gyro.getAngle()), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Front", leftFront.getOutputCurrent());
    SmartDashboard.putNumber("Left Back", leftBack.getOutputCurrent());
    SmartDashboard.putNumber("Right Front", rightFront.getOutputCurrent());
    SmartDashboard.putNumber("Right Back", rightBack.getOutputCurrent());
    SmartDashboard.putNumber("left encoder", leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("right encoder", rightFrontEncoder.getPosition());

    SmartDashboard.putNumber("pose x", getPose().getX());
    SmartDashboard.putNumber("pose y", getPose().getY());
    SmartDashboard.putNumber("pose theta", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("angle", gyro.getAngle());
    SmartDashboard.putNumber("is gyro working?", gyro.getVelocityX());
    SmartDashboard.putNumber("yaw", gyro.getYaw());

    System.out.println(getPose().getX() + ", " + 
      getPose().getY() + ", " + getPose().getRotation().getDegrees());

    //double sensorPosition = Encoder.get() * DrivetrainConstants.encoderConversionFactor;
  }
}