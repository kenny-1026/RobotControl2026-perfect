  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kLimelightName = "limelight";

  // ===== Limelight 設定 =====
  public static final class LimelightConstants {
    // ── 安裝位置（機器人座標系）──
    // ⚠️ 請用捲尺實際量測後修改！
    public static final double kForwardMeters = -0.24;   // 鏡頭距機器人中心往前 (m)
    public static final double kSideMeters    = 0.21;   // 鏡頭距機器人中心往左 (m)，右為負
    public static final double kUpMeters      = 0.60;   // 鏡頭距地面高度 (m)
    public static final double kRollDegrees   = 0.0;    // 繞前後軸旋轉 (deg)
    public static final double kPitchDegrees  = 20.0;   // 鏡頭仰角 (deg)，向上為正
    public static final double kYawDegrees    = 0.0;    // 鏡頭水平旋轉 (deg)，向左為正

    // ── 視覺融合過濾閾值 ──
    public static final double kMaxAmbiguity       = 0.7;    // 單 Tag ambiguity 超過此值就丟棄
    public static final double kMaxTagDistMeters   = 4.0;    // 單 Tag 距離超過此值就丟棄 (m)
    public static final double kMaxGyroRateDps     = 720.0;  // 旋轉速度超過此值時不融合 (deg/s)

    // ── 視覺融合信任度（標準差）──
    public static final double kMultiTagXYStdDev   = 0.3;    // 多 Tag 交叉定位 XY 標準差 (m)
    public static final double kSingleTagBaseStdDev = 0.5;   // 單 Tag 基礎 XY 標準差 (m)
    public static final double kSingleTagDistScale = 0.15;   // 單 Tag 距離比例係數（越遠越不信）
    public static final double kAngleStdDev        = 999999; // 角度不融合，完全信任 IMU
  }

  // ===== 遙測節流 =====
  // Dashboard 更新頻率 = 50Hz / kTelemetryDivider
  // 5 → 每 5 個週期更新一次 = 10Hz (100ms)，人眼足夠
  // 比賽時可改更大值（如 10 → 5Hz）進一步減輕網路負擔
  public static final int kTelemetryDivider = 5;

  // ===== 自動瞄準射擊相關常數 =====
  // 2026 REBUILT: 射入 Hub 得分（Fuel 遊戲物件）
  public static final class AutoAimConstants {
    // ── 場地尺寸 ──
    public static final double kFieldLengthMeters = 16.541; // 場地總長度 (m)（官方 2026 REBUILT）
    public static final double kFieldWidthMeters = 8.069;   // 場地總寬度 (m)
    public static final double kFieldMidX = kFieldLengthMeters / 2.0; // 中場線 X ≈ 8.27m

    // ── Hub 目標座標（藍方原點座標系，單一來源） ──
    // 2026 REBUILT: Hub 在場地中央區域，不在牆壁邊！
    // 座標由官方 AprilTag JSON (2026-rebuilt-welded.json) 計算 Hub 中心位置
    // 藍方 Hub: AprilTag 18-21, 24-27 中心
    // 紅方 Hub 由 kFieldLengthMeters - kHubX 自動推導（對稱場地）
    // ⚠️ 這些值是從 AprilTag 座標估算的 Hub 中心，只需維護藍方座標，紅方自動鏡像！
    public static final double kHubX = 4.626;   // 藍方 Hub X 座標 (m)（場地左半部）
    public static final double kHubY = 4.035;   // Hub Y 座標 (m)（場地中央，紅藍相同）

    // ── 中立區回傳角度（藍方基準） ──
    // 藍方聯盟區在場地左邊(-X) → 朝 180° 射
    // 紅方由 flipAngle() 自動計算（π → 0）
    public static final double kReturnAngleRad = Math.PI; // 藍方基準：朝場地正左 (180°)

    /**
     * 取得己方 Hub 的場地座標（自動處理紅藍鏡像）
     * WPILib 座標系始終以藍方為原點，紅方 Hub = (fieldLength - blueHubX, blueHubY)
     * @param isRed 是否為紅方聯盟
     * @return Hub 中心的 Translation2d
     */
    public static Translation2d getHubPosition(boolean isRed) {
      return isRed
          ? new Translation2d(kFieldLengthMeters - kHubX, kHubY)
          : new Translation2d(kHubX, kHubY);
    }

    /**
     * 取得中立區回傳角度（自動處理紅藍鏡像）
     * 藍方朝 π（場地正左），紅方朝 0（場地正右）
     * @param isRed 是否為紅方聯盟
     * @return 回傳方向角度 (rad)
     */
    public static double getReturnAngleRad(boolean isRed) {
      // 紅方：將藍方角度水平翻轉 → π - θ（mod 2π 後 π - π = 0）
      return isRed ? MathUtil.angleModulus(Math.PI - kReturnAngleRad) : kReturnAngleRad;
    }

    /**
     * 判斷機器人是否在己方場域
     * 藍方己方在 X < kFieldMidX，紅方己方在 X > kFieldMidX
     * @param robotX 機器人當前 X 座標 (m)
     * @param isRed 是否為紅方聯盟
     * @return true = 在己方場域
     */
    public static boolean isInOwnZone(double robotX, boolean isRed) {
      return isRed ? (robotX > kFieldMidX) : (robotX < kFieldMidX);
    }

    // 旋轉 PID（控制底盤面向目標）
    public static final double kRotation_kP = 5.5;
    public static final double kRotation_kI = 0.0;
    public static final double kRotation_kD = 0.1;
    public static final double kRotationToleranceDeg = 2.0; // 角度容許誤差 (度)

    // ── 距離 → 射手 RPS 多項式曲線擬合 ──
    // 由實際測量 8 個數據點 (1m~5m) 做 2 次多項式迴歸得出：
    //   RPS(d) = kRpsA * d² + kRpsB * d + kRpsC
    // 最大擬合誤差 ≈ 1.3 RPS（在容許範圍內）
    //
    // 原始測量數據（保留作為校驗參考）：
    //   1.0m→35, 1.5m→40, 2.0m→45, 2.5m→50,
    //   3.0m→55, 3.5m→60, 4.0m→65, 5.0m→70
    //
    // ⚠️ 如果更換射手機構或重新測量，請用 fit_rps.py 重新擬合係數！
    public static final double kRpsA = -0.686275; // d² 係數
    public static final double kRpsB =  13.186275; // d  係數
    public static final double kRpsC = 21.911765; // 常數項

    // 安全限制：超出測量範圍時 clamp 到邊界值
    public static final double kRpsMinDistance = 1.0;  // 最近測量距離 (m)
    public static final double kRpsMaxDistance = 5.0;  // 最遠測量距離 (m)
    public static final double kRpsMin = 35.0;         // 最低 RPS（對應最近距離）
    public static final double kRpsMax = 70.0;         // 最高 RPS（對應最遠距離）

    // 射手速度容許誤差 (RPS)
    public static final double kShooterToleranceRps = 10.0;

    // 中立區回傳球固定射手 RPS
    // 因為距離遠且不需要精準進 Hub，用固定高速射回即可
    // ⚠️ 請在實際場地測試後調整！
    public static final double kMidFieldReturnRps = 70.0;

    // 送球遲滯角度 (Hysteresis)：
    // 首次觸發送球需 ≤ kRotationToleranceDeg (2°)
    // 一旦開始送球，放寬到 kFeedingHysteresisDeg，避免移動中微小偏差中斷連射
    public static final double kFeedingHysteresisDeg = 5.0;

    // 射擊模式下底盤平移速度倍率
    // 降低平移速度避免因慣性導致球射偏（1.0 = 全速，0.3 = 30% 速度）
    public static final double kShootingModeSpeedMultiplier = 0.3;

    // ── 射手安裝方向偏移 ──
    // 如果射手出口在機器人正前方（+X 方向），設為 0
    // 如果射手出口在機器人正後方（-X 方向），設為 Math.PI
    // 如果射手出口在其他角度，填入相應弧度值
    // ⚠️ 請根據實際機器人射手安裝方向設定！
    public static final double kShooterAngleOffsetRad = 0; // TODO: 確認射手方向（假設射手在背面）
  }

  public static class OperatorConstants {
    public static final int kSwerveControllerPort = 0;
  }

  // ===== 射手 (Shooter) 常數 =====
  public static final class ShooterConstants {
    // ── 硬體 CAN ID ──
    public static final int kLeaderMotorID = 22;
    public static final int kFollowerMotorID = 21;

    // ── PID 初始值 (Slot 0, VelocityVoltage) ──
    // ⚠ 透過 Shuffleboard TunableNumber 可即時調參，這裡是開機預設值
    public static final double kDefaultKV = 0.16;
    public static final double kDefaultKP = 0.6;
    public static final double kDefaultKI = 0.0;
    public static final double kDefaultKD = 0.0;
    public static final double kDefaultKS = 0.0;

    // ── 其他 ──
    public static final double kTriggerDeadband = 0.05;
  }

  // ===== 輸送帶 (Transport) 常數 =====
  public static final class TransportConstants {
    // ── 硬體 CAN ID ──
    public static final int kUpToShootMotorID = 26;
    public static final int kTransportMotorID = 30;

    // ── 速度目標 (RPS) ──
    public static final double kTransportRps = -35.0;       // 輸送帶正常速度
    public static final double kUpToShootRps = -80.0;       // 上膛推球速度
    public static final double kSlowTransportRps = 40.0;   // 慢速輸送帶 (Intake 時)

    // ── PID 初始值 (Slot 0, VelocityVoltage) ──
    public static final double kDefaultKV = 0.12;
    public static final double kDefaultKP = 0.2;
    public static final double kDefaultKI = 0.0;
    public static final double kDefaultKD = 0.0;

    // ── 電流限制 ──
    public static final double kStatorCurrentLimit = 60.0;
    public static final double kSupplyCurrentLimit = 40.0;
  }

  // ===== Intake 滾輪 (IntakeRoller) 常數 =====
  public static final class IntakeRollerConstants {
    // ── 硬體 CAN ID ──
    public static final int kLeaderMotorID = 29;
    public static final int kFollowerMotorID = 35;

    // ── 速度目標 (RPS) ──
    public static final double kIntakeTargetRps = 40.0;
    public static final double kOuttakeTargetRps = -30.0;

    // ── PID 初始值 (Slot 0, VelocityVoltage) ──
    public static final double kDefaultKV = 0.12;
    public static final double kDefaultKP = 0.2;
    public static final double kDefaultKI = 0.01;
    public static final double kDefaultKD = 0.0;

    // ── 電流限制 ──
    public static final double kStatorCurrentLimit = 60.0;
    public static final double kSupplyCurrentLimit = 40.0;
  }

  // ===== Intake 手臂 (IntakeArm) 常數 =====
  public static final class IntakeArmConstants {
    // ── 硬體 CAN ID ──
    // Tuner X: right_intake_hand = 32, left_intake_hand = 33
    public static final int kLeaderMotorID = 32;
    public static final int kFollowerMotorID = 33;

    // ── 機構參數 ──
    public static final double kGearRatio = 20.0;

    // ── PID 初始值 (Slot 0, PositionVoltage) ──
    public static final double kDefaultKP = 2.0;
    public static final double kDefaultKI = 0.0;
    public static final double kDefaultKD = 0.1;
    public static final double kDefaultKG = 0.1;  // 重力補償
  }

  // ===== 手動駕駛 (ManualDrive) 常數 =====
  public static final class ManualDriveConstants {
    // ── 搖桿倍率 ──
    public static final double kXMultiplier = 1.0;
    public static final double kYMultiplier = 1.0;
    public static final double kZMultiplier = -0.4;

    // ── 搖桿死區 ──
    public static final double kXDeadzone = 0.05;
    public static final double kYDeadzone = 0.05;
    public static final double kZDeadzone = 0.05;
  }

  // ===== AprilTag 對位 (Drive2Tag) 常數 =====
  public static final class Drive2TagConstants {
    // ── PID ──
    public static final double kX_kP = 0.8;
    public static final double kX_kI = 0.001;
    public static final double kX_kD = 0.0;

    public static final double kY_kP = 0.25;
    public static final double kY_kI = 0.001;
    public static final double kY_kD = 0.0;

    public static final double kTheta_kP = 0.008;
    public static final double kTheta_kI = 0.0;
    public static final double kTheta_kD = 0.0;

    // ── 容許誤差 ──
    public static final double kXTolerance = 0.01;     // 公尺
    public static final double kYTolerance = 0.01;     // 公尺
    public static final double kThetaTolerance = 2.0;  // 度

    // ── 輸出限制 ──
    public static final double kSpeedMultiplier = 6.0;
    public static final double kMaxTranslationSpeed = 1.8;  // m/s
    public static final double kMaxRotationSpeed = 1.5;     // rad/s
  }

  public static final class RoboArmConstants {
    public static final int kShooterLeftMotorID = 21;
    
    public static final boolean kShoulderRightMotorInverted = false;
  }

  // Swerve constants
  public static final class SwerveConstants {
    /** DRIVETRAIN CAN bus 物件（Phoenix 6 v26+ 要求使用 CANBus 而非 String） */
    public static final CANBus kDrivetrainCANBus = new CANBus("DRIVETRAIN");

    public static final int kPigeonID = 0;

    // Rotor IDs
    public static final int kLeftFrontRotorID = 3;
    public static final int kRightFrontRotorID = 5;
    public static final int kLeftRearRotorID = 1;
    public static final int kRightRearRotorID = 7;

    // Throttle IDs
    public static final int kLeftFrontThrottleID = 4;
    public static final int kRightFrontThrottleID = 6;
    public static final int kLeftRearThrottleID = 2;
    public static final int kRightRearThrottleID = 8;

    // Rotor encoder IDs
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
    // Swerve kinematics（順序：左前，右前，左後，右後）
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Camera centered kinematics
    public static final SwerveDriveKinematics kSwerveKinematicsCamera = new SwerveDriveKinematics(
      new Translation2d(0, kTrackWidth / 2),
      new Translation2d(0, -kTrackWidth / 2),
      new Translation2d(-kWheelBase, kTrackWidth / 2),
      new Translation2d(-kWheelBase, -kTrackWidth / 2));

    // Rotor PID constants
    // Ref: https://www.ni.com/zh-tw/shop/labview/pid-theory-explained.html#section-366173388
    public static final double kRotor_kP = 0.005; // critical damping: 0.024 ~ 0.025 
    public static final double kRotor_kI = 0.001; // period: 0.169s
    public static final double kRotor_kD = 0.0001;
    public static final double kRotor_maxVelocity = 360; //control max motor output
    public static final double kRotor_maxAcceleration = kRotor_maxVelocity * 5;

    // // Rotor PID constants soft
    // public static final double kRotor_kP = 0.002;
    // public static final double kRotor_kI = 0.005;
    // public static final double kRotor_kD = 0.00001;

    // Wheel diameter
    // 輪徑
    // OD(outer diameter) 4 inches = 0.1016 meters
    // Ref: https://www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore
    // 要包含胎皮受到重量的厚度，建議重新量測並再次確認
    public static final double kWheelDiameterMeters = 0.1;

    // Throttle gear ratio
    // (number of turns it takes the motor to rotate the rotor one revolution)
    // Throttle 齒輪比率（馬達轉動輪子一圈所需的圈數）
    // MK4i底盤: 8.14 for L1 - Standard, 6.75 for L2 - Fast, 6.12 for L3 - Very Fast
    // Ref: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    public static final double kThrottleGearRatio = 6.12; 

    // Throttle velocity conversion constant
    // Throttle 速度轉換 Constant
    // 轉換Enocoder速度RPM -> m/s
    public static final double kThrottleVelocityConversionFactor = (1 / kThrottleGearRatio)
        * kWheelDiameterMeters * Math.PI / 60;

    // Trottle position conversion constant
    // Throttle 位置轉換 Constant
    // 轉換Enocoder位置 圈數 -> m
    public static final double kThrottlePositionConversionFactor = (1 / kThrottleGearRatio)
        * kWheelDiameterMeters * Math.PI;

    // Rotor position conversion constant
    // Rotor 位置轉換 Constant
    // 轉換Enocoder位置 圈數 -> 角度
    // The steering gear ratio of the MK4i is 150/7:1
    // Ref: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    public static final double kRotorPositionConversionFactor = (1 / (150.0 / 7.0)) * 360.0;

    // MK4i L3 + Kraken X60 理論最大速度
    // = (6000 RPM / 60 / kThrottleGearRatio) * kWheelDiameterMeters * PI
    // ≈ 5.13 m/s (用 0.1m 輪徑)
    // SDS 官方建議值約 5.21 m/s (用 4 inch = 0.1016m)
    // 這裡取計算值，所有速度相關的地方都引用這個常數
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
