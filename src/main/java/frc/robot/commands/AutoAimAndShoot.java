package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.util.TunableNumber;

/**
 * 自動瞄準並射擊的 Command (2026 REBUILT)
 * 
 * 功能：
 * 1. 根據機器人目前座標，計算面向目標（Hub）所需的旋轉角度
 * 2. 用 PID 控制底盤旋轉到正確角度（同時駕駛員仍可控制平移）
 * 3. 根據距離自動調整射手轉速
 * 4. 當角度對齊且射手達速時，自動啟動 Transport 送球發射
 * 5. 中場區域自動切換為回傳模式（朝己方聯盟區射球）
 * 
 * 使用方式：綁定到按鈕的 whileTrue()，放開即停止
 */
public class AutoAimAndShoot extends Command {

    private final Swerve m_swerve;
    private final ShooterSubsystem m_shooter;
    private final TransportSubsystem m_transport;
    private final ManualDrive m_manualDrive; // 用於控制射擊模式

    // 旋轉 PID 控制器
    private final PIDController m_rotationPID;

    // ── 即時調參 ──
    private final TunableNumber tunableRotKP;
    private final TunableNumber tunableRotKI;
    private final TunableNumber tunableRotKD;

    // ── Shuffleboard 遙測 ──
    private GenericEntry distanceEntry, targetAngleEntry, currentAngleEntry;
    private GenericEntry angleErrorEntry, rotOutputEntry, targetRpsEntry, currentRpsEntry;
    private GenericEntry isAlignedEntry, isAtSpeedEntry, isFeedingEntry;

    // 目標位置（每週期動態更新）
    private Translation2d m_targetPosition;

    // 狀態追蹤
    private boolean m_isFeeding = false;
    private boolean m_isInOwnZone = true; // 追蹤是否在己方場域
    private int telemetryCounter = 0;

    public AutoAimAndShoot(Swerve swerve, ShooterSubsystem shooter, TransportSubsystem transport, ManualDrive manualDrive) {
        this(swerve, shooter, transport, manualDrive, null);
    }

    public AutoAimAndShoot(Swerve swerve, ShooterSubsystem shooter, TransportSubsystem transport, ManualDrive manualDrive, ShuffleboardTab tab) {
        m_swerve = swerve;
        m_shooter = shooter;
        m_transport = transport;
        m_manualDrive = manualDrive;

        // ── 調參：使用 Shuffleboard Tab 或 SmartDashboard ──
        if (tab != null) {
            tunableRotKP = new TunableNumber(tab, "Rotation kP", AutoAimConstants.kRotation_kP);
            tunableRotKI = new TunableNumber(tab, "Rotation kI", AutoAimConstants.kRotation_kI);
            tunableRotKD = new TunableNumber(tab, "Rotation kD", AutoAimConstants.kRotation_kD);

            distanceEntry    = tab.add("Distance", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(0, 2).getEntry();
            targetAngleEntry = tab.add("Target Angle", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(2, 2).getEntry();
            currentAngleEntry= tab.add("Current Angle", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(4, 2).getEntry();
            angleErrorEntry  = tab.add("Angle Error", 0).withWidget(BuiltInWidgets.kGraph).withSize(3, 2).withPosition(0, 3).getEntry();
            rotOutputEntry   = tab.add("Rot Output", 0).withWidget(BuiltInWidgets.kGraph).withSize(3, 2).withPosition(3, 3).getEntry();
            targetRpsEntry   = tab.add("Target RPS", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(6, 2).getEntry();
            currentRpsEntry  = tab.add("Current RPS", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(6, 3).getEntry();
            isAlignedEntry   = tab.add("Aligned?", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1).withPosition(8, 2).getEntry();
            isAtSpeedEntry   = tab.add("At Speed?", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1).withPosition(8, 3).getEntry();
            isFeedingEntry   = tab.add("Feeding?", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1).withPosition(8, 4).getEntry();
        } else {
            tunableRotKP = new TunableNumber("AutoAim/Rotation kP", AutoAimConstants.kRotation_kP);
            tunableRotKI = new TunableNumber("AutoAim/Rotation kI", AutoAimConstants.kRotation_kI);
            tunableRotKD = new TunableNumber("AutoAim/Rotation kD", AutoAimConstants.kRotation_kD);
        }

        m_rotationPID = new PIDController(
            AutoAimConstants.kRotation_kP,
            AutoAimConstants.kRotation_kI,
            AutoAimConstants.kRotation_kD
        );
        // 角度是連續的 (-π ~ π)，避免在 ±180° 附近震盪
        m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        m_rotationPID.setTolerance(Math.toRadians(AutoAimConstants.kRotationToleranceDeg));

        // 需要控制 swerve (透過 setAimSpeed) 和 shooter 和 transport
        addRequirements(m_shooter, m_transport);
        // 注意：不 addRequirements(m_swerve) 因為我們透過 setAimSpeed 疊加，
        // ManualDrive 仍然需要控制 swerve 的平移
    }

    @Override
    public void initialize() {
        m_rotationPID.reset();
        m_isFeeding = false;
        m_isInOwnZone = true;

        // 啟用射擊模式：鎖定操作者旋轉、降低平移速度
        if (m_manualDrive != null) {
            m_manualDrive.setShootingMode(true);
        }
        // 目標位置在 execute() 中每週期動態判斷（依機器人所在場域切換）
    }

    @Override
    public void execute() {
        // ── 即時 PID 調參 ──
        if (tunableRotKP.hasChanged() || tunableRotKI.hasChanged() || tunableRotKD.hasChanged()) {
            m_rotationPID.setPID(tunableRotKP.get(), tunableRotKI.get(), tunableRotKD.get());
        }

        // 1. 取得機器人目前位置
        Pose2d robotPose = m_swerve.getPose();
        Translation2d robotPosition = robotPose.getTranslation();

        // 2. 動態判斷機器人所在區域，決定目標
        //    己方場域 → 瞄準 Hub 得分 (2026 REBUILT: Fuel 射入 Hub)
        //    中立區 / 對方場域 → 朝固定角度射回己方聯盟區
        boolean isRed = m_swerve.isAllianceRed();
        double robotX = robotPosition.getX();

        // 目標角度（場地座標系）
        double targetAngleRad;

        // 統一邏輯：使用 Constants helper 自動處理紅藍鏡像
        m_isInOwnZone = AutoAimConstants.isInOwnZone(robotX, isRed);
        if (m_isInOwnZone) {
            // 己方區域：瞄準己方 Hub
            m_targetPosition = AutoAimConstants.getHubPosition(isRed);
            Translation2d toTarget = m_targetPosition.minus(robotPosition);
            // atan2 計算「機器人→Hub」的場地角度，再加上射手安裝偏移
            // 如果射手在背面（offset=π），機器人需要背對 Hub 才能射中
            targetAngleRad = Math.atan2(toTarget.getY(), toTarget.getX()) 
                + AutoAimConstants.kShooterAngleOffsetRad;
        } else {
            // 中立區：朝固定角度射回己方聯盟區
            m_targetPosition = null; // 不需要目標點
            targetAngleRad = AutoAimConstants.getReturnAngleRad(isRed) 
                ;
        }

        // 3. 計算到目標的距離（己方區域用實際距離，中立區用估算）
        double distanceToTarget;
        if (m_isInOwnZone && m_targetPosition != null) {
            distanceToTarget = m_targetPosition.minus(robotPosition).getNorm();
        } else {
            // 中立區沒有特定目標點，用到 Hub 的距離估算（僅用於 debug 顯示）
            distanceToTarget = AutoAimConstants.getHubPosition(isRed).minus(robotPosition).getNorm();
        }

        // 4. 目前機器人朝向
        double currentAngleRad = robotPose.getRotation().getRadians();

        // 正規化目標角度到 [-π, π]（加上射手偏移後可能超出範圍）
        targetAngleRad = MathUtil.angleModulus(targetAngleRad);

        // 6. PID 計算旋轉速度（不限制最大值，讓底盤全速旋轉對齊）
        double rotationOutput = m_rotationPID.calculate(currentAngleRad, targetAngleRad);

        // 7. 透過 setAimSpeed 疊加旋轉（不影響駕駛員的平移控制）
        m_swerve.setAimSpeed(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, rotationOutput));

        // 8. 根據區域決定射手 RPS
        //    己方場域 → 根據距離查表（精準射擊得分）
        //    中立區  → 固定高速（只需把球射回己方區域）
        double targetRps;
        if (m_isInOwnZone) {
            targetRps = interpolateRps(distanceToTarget);
        } else {
            targetRps = AutoAimConstants.kMidFieldReturnRps;
        }

        // 9. 設定射手速度（按下按鈕就開始轉，不等對齊）
        m_shooter.setTargetVelocity(targetRps);

        // 10. 判斷角度對齊（使用遲滯邏輯防止連射中斷）
        double angleErrorRad = MathUtil.angleModulus(targetAngleRad - currentAngleRad);
        double angleErrorDeg = Math.abs(Math.toDegrees(angleErrorRad));
        
        // 遲滯 (Hysteresis)：
        //   - 尚未送球 → 需嚴格對齊 (≤ kRotationToleranceDeg, 預設 2°) 才開始
        //   - 已在送球 → 使用寬鬆閾值 (≤ kFeedingHysteresisDeg, 預設 5°) 保持連射
        //   這樣移動過程中的些許角度偏差不會中斷送球
        boolean isAligned;
        if (m_isFeeding) {
            isAligned = angleErrorDeg <= AutoAimConstants.kFeedingHysteresisDeg;
        } else {
            isAligned = angleErrorDeg <= AutoAimConstants.kRotationToleranceDeg;
        }
        
        boolean isAtSpeed = m_shooter.isAtSpeed(targetRps, AutoAimConstants.kShooterToleranceRps);

        // 11~12. 角度對齊 + 射手達速 → 同時啟動 transport + up_to_shoot 送球
        //        兩顆馬達同時啟動、同時停止，確保送球一致性
        if (isAligned && isAtSpeed) {
            m_transport.runTransport();     // 同時啟動 transport + up_to_shoot
            m_isFeeding = true;
        } else {
            // 超出遲滯範圍或射手失速 → 同時停止兩顆馬達
            m_transport.stopTransport();
            m_isFeeding = false;
        }

        // Debug 資訊（節流輸出）
        if (++telemetryCounter >= Constants.kTelemetryDivider) {
            telemetryCounter = 0;
            if (distanceEntry != null) {
                distanceEntry.setDouble(distanceToTarget);
                targetAngleEntry.setDouble(Math.toDegrees(targetAngleRad));
                currentAngleEntry.setDouble(Math.toDegrees(currentAngleRad));
                angleErrorEntry.setDouble(Math.toDegrees(angleErrorRad));
                rotOutputEntry.setDouble(rotationOutput);
                targetRpsEntry.setDouble(targetRps);
                currentRpsEntry.setDouble(m_shooter.getCurrentRps());
                isAlignedEntry.setBoolean(isAligned);
                isAtSpeedEntry.setBoolean(isAtSpeed);
                isFeedingEntry.setBoolean(m_isFeeding);
            } else {
                SmartDashboard.putNumber("AutoAim/Distance", distanceToTarget);
                SmartDashboard.putNumber("AutoAim/TargetAngle", Math.toDegrees(targetAngleRad));
                SmartDashboard.putNumber("AutoAim/CurrentAngle", Math.toDegrees(currentAngleRad));
                SmartDashboard.putNumber("AutoAim/AngleError", Math.toDegrees(angleErrorRad));
                SmartDashboard.putNumber("AutoAim/RotationOutput", rotationOutput);
                SmartDashboard.putNumber("AutoAim/TargetRPS", targetRps);
                SmartDashboard.putNumber("AutoAim/CurrentRPS", m_shooter.getCurrentRps());
                SmartDashboard.putBoolean("AutoAim/IsAligned", isAligned);
                SmartDashboard.putBoolean("AutoAim/IsAtSpeed", isAtSpeed);
                SmartDashboard.putBoolean("AutoAim/IsFeeding", m_isFeeding);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // 停止所有動作
        m_swerve.setAimSpeed(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0));
        m_shooter.stopShooter();
        m_transport.stopTransport();
        m_isFeeding = false;

        // 解除射擊模式：恢復正常操控
        if (m_manualDrive != null) {
            m_manualDrive.setShootingMode(false);
        }
    }

    @Override
    public boolean isFinished() {
        // 此 Command 持續執行，靠 whileTrue 放開按鈕來結束
        return false;
    }

    /**
     * 根據距離線性內插查表取得目標 RPS
     * 委託給 ShooterSubsystem.interpolateRps（共用邏輯）
     * @param distance 到目標的距離 (m)
     * @return 目標射手 RPS
     */
    private double interpolateRps(double distance) {
        return ShooterSubsystem.interpolateRps(distance);
    }
}
