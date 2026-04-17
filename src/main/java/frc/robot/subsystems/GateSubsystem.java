package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ✨ 注意：這裡現在「只」匯入 GateConstants
import frc.robot.Constants.GateConstants; 

public class GateSubsystem extends SubsystemBase {
    private final TalonFX gateMotor;
    private final VelocityVoltage gateRequest = new VelocityVoltage(0);

    public GateSubsystem() {
        gateMotor = new TalonFX(GateConstants.kGateMotorID); 

        TalonFXConfiguration config = new TalonFXConfiguration();
       // ✨ 修改 1：改成 Brake 模式，才不會被重力拉回去
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // ✨ 使用專屬的 PID 參數
        config.Slot0.kV = GateConstants.kGateKv;
        config.Slot0.kP = GateConstants.kGateKp;
        config.Slot0.kI = GateConstants.kGateKi;
        config.Slot0.kD = GateConstants.kGateKd;
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // ✨ 修改 2：設定軟體極限 (Soft Limits)
        // 開啟正轉極限，並設定最大圈數
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = GateConstants.kGateMaxTurns;
        
        // 開啟反轉極限，並設定最小圈數 (通常是 0)
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = GateConstants.kGateMinTurns;

        gateMotor.getConfigurator().apply(config);

        // ✨ 修改 3：開機時，強制把馬達現在的位置當作「0圈」
        // ⚠️ 警告：這代表你每次開機時，閘門都必須在「完全放下/關閉」的狀態！
        gateMotor.setPosition(0.0);
    }

    public void setSpeed(double rps) {
        gateMotor.setControl(gateRequest.withVelocity(rps));
    }

    public void stop() {
        gateMotor.stopMotor();
    }

    // ✨ 使用專屬的轉速 (kGateSpeedRps)
    public Command sys_runGate() {
        return this.runEnd(
            () -> setSpeed(GateConstants.kGateSpeedRps), 
            this::stop
        );
    }
}