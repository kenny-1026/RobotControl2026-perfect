package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.MotorAlignmentValue; 
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {
    private final TalonFX leaderMotor;
    // private final TalonFX followerMotor;
    
    private final VelocityVoltage storageRequest = new VelocityVoltage(0);

    // ✨ 新增：位置控制 (轉到特定點)
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public boolean isAtMax() {
        return isAtMax;
    }

    // ✨ 新增這行：用來記住目前是不是在 Max 的位置 (預設為 false，代表一開始在 0)
    private boolean isAtMax = false;

    public StorageSubsystem() {
        leaderMotor = new TalonFX(StorageConstants.kLeaderID);
        // followerMotor = new TalonFX(StorageConstants.kFollowerID);

        
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
        // followerMotor.getConfigurator().apply(config);

        
        // leaderMotor.setPosition(0.0);
        // followerMotor.setPosition(0.0);

        // =========================================
        // 綁定跟隨：使用你們成功的魔法指令
        // =========================================
        // followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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
   
    //   ✨ 位置控制：轉到特定的圈數 (Target Turns)
    //  例如：傳入 5.0，馬達就會轉到 5.0 圈的位置然後停住並鎖死。
     
    public void goToPosition(double targetTurns) {
        leaderMotor.setControl(positionRequest.withPosition(targetTurns));
    }

    public Command sys_togglePosition() {
        // 使用 runOnce，代表這個指令只要被觸發「一次」就會執行並結束
        // CTRE 馬達控制器會自己在背景幫你把馬達推到目標點並鎖死
        return this.runOnce(
            () -> {
                if (isAtMax) {
                    // 如果現在的狀態是 Max，就叫它去 Min
                    goToPosition(StorageConstants.kMinTurns);
                    isAtMax = false; // 把記憶更新為 Min
                } else {
                    // 如果現在的狀態是 Min (或剛開機)，就叫它去 Max
                    goToPosition(StorageConstants.kMaxTurns);
                    isAtMax = true; // 把記憶更新為 Max
                }
            }
        );
    }
}