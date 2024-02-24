// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  
  public static class DrivetrainConstants {
    public static final int leftFrontID = 10;
    public static final int rightFrontID = 13;
    public static final int leftBackID = 11; 
    public static final int rightBackID = 12;
    
    public static final double deadBan = 0.1;

    public static final int limit = 45;

    public static final double encoderConversionFactor = Units.inchesToMeters(6*Math.PI) / 10.71;
    public static final double maxSpeed = 5 / 1.54; // tested max speed in m/s

    public static final DifferentialDriveKinematics kinematics = 
      new DifferentialDriveKinematics(Units.inchesToMeters(MechanicalConstants.trackWidth));

    // change below after characterization!
    public static final double kS = 0.2136; // volts
    public static final double kV = 5.05; // volts * seconds / meters
    public static final double kA = 0.66565; // volts * seconds^2 / meters

    public static final double kP = 6.5103; // additional output per m/s of velocity error

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class MechanicalConstants {
    public static final double trackWidth = 22.0;

  }
}