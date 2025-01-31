// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

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
  }

  public static class AdvantageKitConstants{
    public enum RobotType{
      SIM,
      REAL,
      REPLAY
    }

    public static final RobotType CURRENT_MODE = RobotType.REAL;
  }

  public enum ElevatorLevels
  {
    HOME,
    ONE,
    TWO,
    THREE,
    FOUR;
  }

  public static DigitalInput getLimitSwitchForLevel(ElevatorLevels levels) {
    switch (levels) {
      case HOME: return new DigitalInput(0);
      case ONE: return null;
      case TWO: return null;
      case THREE: return null;
      case FOUR: return null;
      default: throw new IllegalArgumentException("Unknown level: " + levels);
    }
  }

  public static double getEncoderValueForLevel(ElevatorLevels level)
  {
    switch(level)
    {
      case HOME: return 0.0;
      case ONE: return 0.0;
      case TWO: return 0.0;
      case THREE: return 0.0;
      case FOUR: return 0.0;
      default: throw new IllegalArgumentException("Unknown level: " + level);
    }
  }

  public static class ElevatorMotors {
    public static final int LEFT = 0;
    public static final int RIGHT = 0;
    public static final int MIDDLE = 0;
  }

  public static class ElevatorLimitSwitches {
    public static final int MAIN = 0;
    public static final int MIDDLE = 0;
  }
}
