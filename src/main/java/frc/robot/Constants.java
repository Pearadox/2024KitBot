// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int leftFrontID = 10; //4 on 2023 everybot before change
    public static final int rightFrontID = 13; //3
    public static final int leftBackID = 11; //2
    public static final int rightBackID = 12; //1
    
    public static final double deadBan = 0.1;

    public static final int limit = 45;
  }

  public static class LauncherConstants {
    public static final int feederID = 5;
    public static final int launcherID = 6;

    public static final int launcherCurrentLimit = 80; // change?
    public static final int feederCurrentLimit = 80; // change?

    public static final double launcherSpeed = 1;
    public static final double launchFeederSpeed = 1;
    public static final double intakeLauncherSpeed = -1;
    public static final double intakeFeederSpeed = -0.2;

    public static final double launcherDelay = 1;
  }  
}
