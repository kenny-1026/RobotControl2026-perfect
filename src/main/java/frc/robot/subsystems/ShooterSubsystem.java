package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private int telemetryCounter = 0;

    // ── 快取 Status Signal（避免每週期重複查找 + 控制 CAN 更新頻率）──
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;

    // ── Shuffleboard 即時調參 ──
    private TunableNumber tunableKV;
    private TunableNumber tunableKP;
    private TunableNumber tunableKI;
    private TunableNumber tunableKD;
    private TunableNumber tunableKS;

    // ── Shuffleboard 遙測 ──
    private GenericEntry currentRpsEntry;
    private GenericEntry targetRpsEntry;
    private GenericEntry errorRpsEntry;
    private GenericEntry outputVoltageEntry;
    private GenericEntry statorCurrentEntry;

    public ShooterSubsystem() {
        this(null);
    }

    public ShooterSubsystem(ShuffleboardTab tab) {
        leaderMotor = new TalonFX(ShooterConstants.kLeaderMotorID);
        followerMotor = new TalonFX(ShooterConstants.kFollowerMotorID);

        // ── 初始化可調參數 ──
        if (tab != null) {
            tunableKV = new TunableNumber(tab, "kV", ShooterConstants.kDefaultKV);
            tunableKP = new TunableNumber(tab, "kP", ShooterConstants.kDefaultKP);
            tunableKI = new TunableNumber(tab, "kI", ShooterConstants.kDefaultKI);
            tunableKD = new TunableNumber(tab, "kD", ShooterConstants.kDefaultKD);
            tunableKS = new TunableNumber(tab, "kS", ShooterConstants.kDefaultKS);

            // ── 遙測示波圖 ──
            currentRpsEntry = tab.add("Current RPS", 0)
                .withWidget(BuiltInWidgets.kGraph)
                .withSize(3, 2).withPosition(0, 2).getEntry();
            targetRpsEntry = tab.add("Target RPS", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(1, 1).withPosition(3, 2).getEntry();
            errorRpsEntry = tab.add("Error RPS", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(1, 1).withPosition(4, 2).getEntry();
            outputVoltageEntry = tab.add("Output V", 0)
                .withWidget(BuiltInWidgets.kGraph)
                .withSize(3, 2).withPosition(5, 2).getEntry();
            statorCurrentEntry = tab.add("Stator A", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(1, 1).withPosition(3, 3).getEntry();
        } else {
            tunableKV = new TunableNumber("Shooter/kV", ShooterConstants.kDefaultKV);
            tunableKP = new TunableNumber("Shooter/kP", ShooterConstants.kDefaultKP);
            tunableKI = new TunableNumber("Shooter/kI", ShooterConstants.kDefaultKI);
            tunableKD = new TunableNumber("Shooter/kD", ShooterConstants.kDefaultKD);
            tunableKS = new TunableNumber("Shooter/kS", ShooterConstants.kDefaultKS);
        }

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.Slot0.kV = tunableKV.get(); 
        config.Slot0.kP = tunableKP.get();
        config.Slot0.kI = tunableKI.get();
        config.Slot0.kD = tunableKD.get();
        config.Slot0.kS = tunableKS.get();

        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // ── 快取 Status Signal 並設定 CAN 更新頻率 ──
        // velocity: 控制迴路用（isAtSpeed 判斷），50Hz 足夠（預設 250Hz 太浪費）
        // voltage/current: 僅 debug 遙測用，10Hz 即可
        velocitySignal = leaderMotor.getVelocity();
        motorVoltageSignal = leaderMotor.getMotorVoltage();
        statorCurrentSignal = leaderMotor.getStatorCurrent();

        velocitySignal.setUpdateFrequency(50);       // 50Hz = 20ms
        motorVoltageSignal.setUpdateFrequency(10);    // 10Hz = 100ms
        statorCurrentSignal.setUpdateFrequency(10);   // 10Hz = 100ms

        // 其餘未使用的 Signal 降到最低，節省 CAN 頻寬
        leaderMotor.optimizeBusUtilization();
    }

    private void setVelocity(double rps){  
        leaderMotor.setControl(velocityRequest.withVelocity(rps));
    }

    private void stop() {
        leaderMotor.stopMotor();
        followerMotor.stopMotor();
    }

    /**
     * 設定射手馬達目標速度 (public，供外部 Command 呼叫)
     * @param rps 目標轉速 (rotations per second)
     */
    public void setTargetVelocity(double rps) {
        setVelocity(rps);
    }

    /**
     * 停止射手馬達 (public，供外部 Command 呼叫)
     */
    public void stopShooter() {
        stop();
    }

    /**
     * 取得目前實際轉速（使用快取 Signal，不重複查找）
     * @return 目前 RPS
     */
    public double getCurrentRps() {
        return velocitySignal.refresh().getValueAsDouble();
    }

    /**
     * 建立一個固定轉速射擊的 Command（一鍵達速模式）。
     * 按住按鈕期間持續以指定 RPS 運轉，放開後自動停止。
     *
     * @param targetRps 目標轉速 (rotations per second)
     * @return 控制 Shooter 的 Command
     */
    public Command sys_manualShoot(double targetRps) {
        return this.runEnd(
            () -> setVelocity(targetRps),
            () -> stop()
        );
    }

    /**
     * 檢查目前轉速是否達到目標 (允許一點點誤差)
     * @param targetRps 目標轉速
     * @param tolerance 容許誤差
     * @return true 代表速度夠了，可以發射
     */
    public boolean isAtSpeed(double targetRps, double tolerance) {
        double currentRps = velocitySignal.refresh().getValueAsDouble();
        return Math.abs(currentRps - targetRps) < tolerance;
    }
    /**
     * 回傳目前是否達到指定速度 (使用預設誤差 5 RPS)
     * 這是為了讓 Command 寫起來簡潔一點
     */
    public boolean isAtSpeed(int targetRps) {
        return isAtSpeed(targetRps, 5.0); // 預設容許誤差 5 RPS
    }

    /**
     * 根據距離計算目標 RPS（2 次多項式曲線擬合）。
     * <p>
     * 公式: {@code RPS = kRpsA * d² + kRpsB * d + kRpsC}
     * <p>
     * 超出測量範圍 (1m~5m) 時 clamp 到邊界值，避免外推爆掉。
     *
     * @param distance 到目標的距離 (m)
     * @return 目標射手 RPS
     */
    public static double interpolateRps(double distance) {
        // 邊界安全：超出測量範圍直接回傳限制值
        if (distance <= AutoAimConstants.kRpsMinDistance) {
            return AutoAimConstants.kRpsMin;
        }
        if (distance >= AutoAimConstants.kRpsMaxDistance) {
            return AutoAimConstants.kRpsMax;
        }

        // 2 次多項式: RPS = A*d² + B*d + C
        double rps = AutoAimConstants.kRpsA * distance * distance
                   + AutoAimConstants.kRpsB * distance 
                   + AutoAimConstants.kRpsC;

        // 安全 clamp，防止曲線在邊界附近超出合理範圍
        return MathUtil.clamp(rps, AutoAimConstants.kRpsMin, AutoAimConstants.kRpsMax);
    }

    @Override
    public void periodic() {
        // ── 即時 PID 調參：偵測 Shuffleboard 上的數值變更，自動套用到馬達 ──
        if (tunableKV.hasChanged() || tunableKP.hasChanged() 
            || tunableKI.hasChanged() || tunableKD.hasChanged()
            || tunableKS.hasChanged()) {
            var newSlot0 = new Slot0Configs();
            newSlot0.kV = tunableKV.get();
            newSlot0.kP = tunableKP.get();
            newSlot0.kI = tunableKI.get();
            newSlot0.kD = tunableKD.get();
            newSlot0.kS = tunableKS.get();
            leaderMotor.getConfigurator().apply(newSlot0);
        }

        // ── 遙測數據（節流：每 kTelemetryDivider 週期更新一次）──
        if (++telemetryCounter >= Constants.kTelemetryDivider) {
            telemetryCounter = 0;

            double currentRps = velocitySignal.refresh().getValueAsDouble();
            double targetRps = velocityRequest.Velocity;
            double errorRps = targetRps - currentRps;
            double outputV = motorVoltageSignal.refresh().getValueAsDouble();
            double statorA = statorCurrentSignal.refresh().getValueAsDouble();

            if (currentRpsEntry != null) {
                // Shuffleboard 模式
                currentRpsEntry.setDouble(currentRps);
                targetRpsEntry.setDouble(targetRps);
                errorRpsEntry.setDouble(errorRps);
                outputVoltageEntry.setDouble(outputV);
                statorCurrentEntry.setDouble(statorA);
            } else {
                // SmartDashboard 後備
                SmartDashboard.putNumber("Shooter/Current RPS", currentRps);
                SmartDashboard.putNumber("Shooter/Target RPS", targetRps);
                SmartDashboard.putNumber("Shooter/Error RPS", errorRps);
                SmartDashboard.putNumber("Shooter/Output Voltage", outputV);
                SmartDashboard.putNumber("Shooter/Stator Current", statorA);
            }
        }
    }
}