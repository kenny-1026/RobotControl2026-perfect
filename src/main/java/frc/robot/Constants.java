// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

  // ===== 遙測節流 =====
  public static final int kTelemetryDivider = 10;

  public static class OperatorConstants {
    public static final int kSwerveControllerPort = 0;
  }

  // ===== 手動駕駛 (ManualDrive) 常數 =====
  public static final class ManualDriveConstants {
    public static final double kXMultiplier = 1.0;
    public static final double kYMultiplier = 1.0;
    public static final double kZMultiplier = -0.4;

    public static final double kXDeadzone = 0.05;
    public static final double kYDeadzone = 0.05;
    public static final double kZDeadzone = 0.05;
  }

  // Swerve constants
  public static final class SwerveConstants {
    /** DRIVETRAIN CAN bus 物件 */
    public static final CANBus kDrivetrainCANBus = new CANBus("DRIVETRAIN");

    public static final int kPigeonID = 10;
    public static final int kPDPID = 0;

    // Rotor IDs (Turn motors)
    public static final int kLeftFrontRotorID = 3;
    public static final int kRightFrontRotorID = 5;
    public static final int kLeftRearRotorID = 1;
    public static final int kRightRearRotorID = 7;

    // Throttle IDs (Drive motors)
    public static final int kLeftFrontThrottleID = 4;
    public static final int kRightFrontThrottleID = 6;
    public static final int kLeftRearThrottleID = 2;
    public static final int kRightRearThrottleID = 8;

    // Rotor encoder IDs (CANCoders)
    public static final int kLeftFrontCANCoderID = 12;
    public static final int kRightFrontCANCoderID = 13;
    public static final int kLeftRearCANCoderID = 11;
    public static final int kRightRearCANCoderID = 14;

    // Rotor encoder & motor inversion
    public static final boolean kRotorEncoderDirection = false;

    public static final boolean kLeftFrontRotorInverted = true;
    public static final boolean kRightFrontRotorInverted = true;
    public static final boolean kLeftRearRotorInverted = true;
    public static final boolean kRightRearRotorInverted = true;

    public static final boolean kLeftFrontThrottleInverted = false;
    public static final boolean kRightFrontThrottleInverted = false;
    public static final boolean kLeftRearThrottleInverted = false;
    public static final boolean kRightRearThrottleInverted = false;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.62865000;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.62865000;

    // Swerve kinematics (order: left front, right front, left rear, right rear)
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Rotor PID constants
    public static final double kRotor_kP = 0.005;
    public static final double kRotor_kI = 0.001;
    public static final double kRotor_kD = 0.0001;
    public static final double kRotor_maxVelocity = 360;
    public static final double kRotor_maxAcceleration = kRotor_maxVelocity * 5;

    // Wheel diameter
    public static final double kWheelDiameterMeters = 0.1;

    // Throttle gear ratio
    public static final double kThrottleGearRatio = 6.12; 

    // Throttle velocity conversion constant
    public static final double kThrottleVelocityConversionFactor = (1 / kThrottleGearRatio)
        * kWheelDiameterMeters * Math.PI / 60;

    // Trottle position conversion constant
    public static final double kThrottlePositionConversionFactor = (1 / kThrottleGearRatio)
        * kWheelDiameterMeters * Math.PI;

    // Rotor position conversion constant
    public static final double kRotorPositionConversionFactor = (1 / (150.0 / 7.0)) * 360.0;

    // Max physical speed
    public static final double kMaxPhysicalSpeedMps = 
        (6000.0 / 60.0 / kThrottleGearRatio) * kWheelDiameterMeters * Math.PI;
  }

  // Voltage compensation
  public static final double kVoltageCompensation = 12.0;
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = SwerveConstants.kMaxPhysicalSpeedMps;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/10;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/5;
    
    public static final double kTranslationController_kP = 1.3;
    public static final double kTranslationController_kI = 0.001;
    public static final double kTranslationController_kD = 0.005;

    public static final double kRotationController_kP = 0.2;
    public static final double kRotationController_kI = 0.005;
    public static final double kRotationController_kD = 0.001;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
}
