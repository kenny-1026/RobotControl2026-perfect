package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleKraken extends SwerveModule {

    private TalonFX rotorMotor;
    private TalonFX throttleMotor;

    // ── 快取 Status Signal（里程計關鍵路徑，100Hz 更新）──
    private StatusSignal<Angle> throttlePositionSignal;
    private StatusSignal<AngularVelocity> throttleVelocitySignal;

    // 模擬用：追蹤模擬的位置和速度
    private double simThrottlePosition = 0;
    private double simThrottleVelocity = 0;
    private double simRotorPosition = 0;

    public SwerveModuleKraken(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted, String location) {
        super(throttleID, rotorID, throttleInverted, rotorInverted, location);
    }

    public SwerveModuleKraken(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted, int rotorEncoderID, String location) {
        super(throttleID, rotorID, throttleInverted, rotorInverted, rotorEncoderID, location);
    }

    @Override
    protected void initMotors(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted) {
        var throttleConfig = new MotorOutputConfigs()
                                .withInverted(throttleInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake);
        var throttleFeedbeackConfig = new FeedbackConfigs().withSensorToMechanismRatio(1/SwerveConstants.kThrottlePositionConversionFactor);
                                
        throttleMotor = new TalonFX(throttleID, SwerveConstants.kDrivetrainCANBus);
        StatusCode status;
        status = throttleMotor.getConfigurator().apply(throttleConfig);
        if (status != StatusCode.OK) {
            DriverStation.reportWarning(mLocation + " Throttle Config Error: " + status, false);
        }
        status = throttleMotor.getConfigurator().apply(throttleFeedbeackConfig);
        if (status != StatusCode.OK) {
            DriverStation.reportWarning(mLocation + " Throttle Feedback Config Error: " + status, false);
        }
        var rotorConfig = new MotorOutputConfigs()
                                .withInverted(rotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast);
        var rotorFeedbeackConfig = new FeedbackConfigs()
                                        .withSensorToMechanismRatio(1/SwerveConstants.kRotorPositionConversionFactor);
        rotorMotor = new TalonFX(rotorID, SwerveConstants.kDrivetrainCANBus);
        status = rotorMotor.getConfigurator().apply(rotorConfig);
        if (status != StatusCode.OK) {
            DriverStation.reportWarning(mLocation + " Rotor Config Error: " + status, false);
        }
        status = rotorMotor.getConfigurator().apply(rotorFeedbeackConfig);
        if (status != StatusCode.OK) {
            DriverStation.reportWarning(mLocation + " Rotor Feedback Config Error: " + status, false);
        }

        // ── 快取 Status Signal 並設定 CAN 更新頻率 ──
        // 里程計 (Odometry) 用：position + velocity 需要高頻率
        throttlePositionSignal = throttleMotor.getPosition();
        throttleVelocitySignal = throttleMotor.getVelocity();

        throttlePositionSignal.setUpdateFrequency(100);  // 100Hz（里程計關鍵路徑）
        throttleVelocitySignal.setUpdateFrequency(100);  // 100Hz

        // Rotor 馬達不需要讀取 Status Signal（用 CANcoder 讀角度），降低頻率
        throttleMotor.optimizeBusUtilization();
        rotorMotor.optimizeBusUtilization();
    }

    @Override
    public void setThrottleSpeed(double speed) {
        double maxPhysicalSpeed = SwerveConstants.kMaxPhysicalSpeedMps;
        double percentOutput = speed / maxPhysicalSpeed;
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        throttleMotor.set(percentOutput);

        // 模擬環境：直接用百分比輸出推算速度與位置
        if (RobotBase.isSimulation()) {
            simThrottleVelocity = percentOutput * maxPhysicalSpeed; // m/s
            simThrottlePosition += simThrottleVelocity * 0.02;     // 每個週期 20ms
        }
    }

    @Override
    public void setRotorSpeed(double speed) {
        rotorMotor.set(speed);

        // 模擬環境：用輸出推算轉向角度變化
        if (RobotBase.isSimulation()) {
            // speed 是 PID 輸出 (-1~1)，映射成度/秒 (假設最大 720 deg/s)
            double rotorDegreesPerSecond = speed * 720.0;
            simRotorPosition += rotorDegreesPerSecond * 0.02;
            // 保持在 -180 ~ 180
            simRotorPosition = ((simRotorPosition + 180) % 360 + 360) % 360 - 180;
        }
    }

    @Override
    public double getThrottlePosition() {
        if (RobotBase.isSimulation()) {
            return simThrottlePosition;
        }
        return throttlePositionSignal.refresh().getValueAsDouble();
    }

    @Override
    public double getThrottleVelocity() {
        if (RobotBase.isSimulation()) {
            return simThrottleVelocity;
        }
        return throttleVelocitySignal.refresh().getValueAsDouble();
    }

    @Override
    public double getRotorPosition() {
        if (RobotBase.isSimulation()) {
            return simRotorPosition;
        }
        return super.getRotorPosition();
    }
}