package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue; 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    
    private final VelocityVoltage storageRequest = new VelocityVoltage(0);

    public StorageSubsystem() {
        leaderMotor = new TalonFX(StorageConstants.kLeaderID);
        followerMotor = new TalonFX(StorageConstants.kFollowerID);

        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Storage 建議用煞車
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        config.Slot0.kV = StorageConstants.kKv;
        config.Slot0.kP = StorageConstants.kKp;

        // 設定圈數限制
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = StorageConstants.kMaxTurns;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = StorageConstants.kMinTurns;

        // 兩顆馬達套用完全一樣的基礎設定
        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        // 兩顆馬達皆歸零
        // leaderMotor.setPosition(0.0);
        // followerMotor.setPosition(0.0);

        // =========================================
        // 綁定跟隨：使用你們成功的魔法指令
        // =========================================
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setSpeed(double rps) {
        leaderMotor.setControl(storageRequest.withVelocity(rps));
    }

    public void stop() {
        leaderMotor.stopMotor();
    }

    // ====================== Commands ======================

    public Command sys_runStorage() {
        return this.runEnd(
            () -> setSpeed(StorageConstants.kStorageSpeedRps),
            this::stop
        );
    }

    public Command sys_reverseStorage() {
        return this.runEnd(
            () -> setSpeed(-StorageConstants.kStorageSpeedRps),
            this::stop
        );
    }
}