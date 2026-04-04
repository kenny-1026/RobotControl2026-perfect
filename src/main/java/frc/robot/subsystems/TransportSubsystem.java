package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
    // 宣告兩顆馬達
    private final TalonFX up_to_shoot;
    private final TalonFX transport;

    // 建立獨立的 VelocityVoltage 閉環控制請求物件，避免兩個馬達共用同一個參照導致互相覆蓋
    private final VelocityVoltage upToShootRequest = new VelocityVoltage(0);
    private final VelocityVoltage transportRequest = new VelocityVoltage(0);

    public TransportSubsystem() {
        up_to_shoot = new TalonFX(TransportConstants.kUpToShootMotorID); 
        transport = new TalonFX(TransportConstants.kTransportMotorID);

        // ==========================================
        // up_to_shoot 馬達設定
        // ==========================================
        TalonFXConfiguration shootConfig = new TalonFXConfiguration();
        shootConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shootConfig.Slot0.kV = TransportConstants.kDefaultKV;
        shootConfig.Slot0.kP = TransportConstants.kDefaultKP;
        shootConfig.Slot0.kI = TransportConstants.kDefaultKI;
        shootConfig.Slot0.kD = TransportConstants.kDefaultKD;
        shootConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shootConfig.CurrentLimits.StatorCurrentLimit = TransportConstants.kStatorCurrentLimit;
        shootConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shootConfig.CurrentLimits.SupplyCurrentLimit = TransportConstants.kSupplyCurrentLimit;
        shootConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        up_to_shoot.getConfigurator().apply(shootConfig);

        // ==========================================
        // transport 輸送帶馬達設定
        // ==========================================
        TalonFXConfiguration transportConfig = new TalonFXConfiguration();
        transportConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        transportConfig.Slot0.kV = TransportConstants.kDefaultKV;
        transportConfig.Slot0.kP = TransportConstants.kDefaultKP;
        transportConfig.Slot0.kI = TransportConstants.kDefaultKI;
        transportConfig.Slot0.kD = TransportConstants.kDefaultKD;
        transportConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        transportConfig.CurrentLimits.StatorCurrentLimit = TransportConstants.kStatorCurrentLimit;
        transportConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        transportConfig.CurrentLimits.SupplyCurrentLimit = TransportConstants.kSupplyCurrentLimit;
        transportConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        transport.getConfigurator().apply(transportConfig);
    }

    /**
     * 同時設定兩顆馬達的目標轉速 (RPS)
     * @param transportRps transport 輸送帶目標轉速
     * @param shootRps     up_to_shoot 上膛推球目標轉速
     */
    public void setSpeed(double transportRps, double shootRps) {
        up_to_shoot.setControl(upToShootRequest.withVelocity(shootRps));
        transport.setControl(transportRequest.withVelocity(transportRps));
    }

    /**
     * 只啟動 transport 輸送帶 (閉環目標轉速)
     */
    public void setSpeed_transport() {
        transport.setControl(transportRequest.withVelocity(TransportConstants.kTransportRps));
    }

    /**
     * 只啟動 up_to_shoot 上膛推球 (閉環目標轉速)
     */
    public void setSpeed_to_shoot() {
        up_to_shoot.setControl(upToShootRequest.withVelocity(TransportConstants.kUpToShootRps));
    }

    /**
     * 停止兩顆馬達（Coast 自然滑行，不瞬間煞車）
     */
    public void stop() {
        up_to_shoot.stopMotor();
        transport.stopMotor();
    }

    /**
     * 啟動 Transport 送球（供外部 Command 直接呼叫）
     * 同時啟動 transport（輸送帶）和 up_to_shoot（上膛推球）
     */
    public void runTransport() {
        setSpeed(TransportConstants.kTransportRps, TransportConstants.kUpToShootRps);
    }

    /**
     * 停止 Transport（供外部 Command 直接呼叫）
     */
    public void stopTransport() {
        stop();
    }

    /**
     * 建立 Command: 按住按鈕時運轉，放開停止
     */
    public Command sys_runTransport() {
        return this.runEnd(
            () -> {
                setSpeed(TransportConstants.kTransportRps, TransportConstants.kUpToShootRps);
            }, 
            () -> {
                stop();
            }
        );
    }

    public Command sys_slowRunTransport() {
        return this.runEnd(
            () -> {
                setSpeed(TransportConstants.kSlowTransportRps, TransportConstants.kSlowTransportRps);
            }, 
            () -> {
                stop();
            }
        );
    }
    
    /**
     * Intake 專用：transport 正轉吸球，up_to_shoot 反轉擋球（防止球直接衝進射手）
     * 在自動吸球時呼叫此方法取代 runTransport()
     */
    public void runIntakeMode() {
        transport.setControl(transportRequest.withVelocity(-30));
        up_to_shoot.setControl(upToShootRequest.withVelocity(0));
    }

    /**
     * 只啟動 up_to_shoot 推球到射手（transport 不動）
     * 用於準備好球後將球推入射手的階段
     */
    public void runUpToShootOnly() {
        up_to_shoot.setControl(upToShootRequest.withVelocity(TransportConstants.kUpToShootRps));
    }

    /**
     * Auto 專用：up_to_shoot 正轉推球 N 秒後自動結束，transport 同步正轉
     * 射手應已在 sys_idle 或更高速度待命
     *
     * @param seconds 推球持續時間 (s)
     * @return Command，結束後 transport 與 up_to_shoot 同時停止
     */
    public Command sys_upToShootForSeconds(double seconds) {
        Timer timer = new Timer();
        return this.run(() -> {
                runTransport(); // transport + up_to_shoot 同時正轉送球
            })
            .beforeStarting(() -> timer.restart())
            .until(() -> timer.hasElapsed(seconds))
            .finallyDo(() -> stopTransport())
            .withName("UpToShoot " + seconds + "s");
        
    }

    /**
     * (選配) 反轉吐球的 Command
     */
    public Command sys_reverseTransport() {
        return this.runEnd(
            () -> setSpeed(-TransportConstants.kTransportRps, -TransportConstants.kUpToShootRps), 
            this::stop
        );
    }

    public Command sys_reverseroller() {
        return this.runEnd(
            () -> up_to_shoot.setControl(upToShootRequest.withVelocity(-TransportConstants.kUpToShootRps)), 
            this::stop
        );
    }
}