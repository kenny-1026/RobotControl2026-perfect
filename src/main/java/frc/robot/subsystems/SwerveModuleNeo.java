package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleNeo extends SwerveModule {
    private SparkMax rotorMotor;
    private SparkMax throttleMotor;
    private RelativeEncoder mThrottleEncoder;
    
    public SwerveModuleNeo(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted, String location) {
        super(throttleID, rotorID, throttleInverted, rotorInverted, location);
    }
    public SwerveModuleNeo(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted, int rotorEncoderID, String location) {
        super(throttleID, rotorID, throttleInverted, rotorInverted, rotorEncoderID, location);
    }
    
    @Override
    protected void initMotors(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted) {
        SparkBaseConfig throttleConfig = new SparkMaxConfig()
            .voltageCompensation(Constants.kVoltageCompensation)
            .idleMode(IdleMode.kBrake)
            .inverted(throttleInverted);
        // 設定Encoder的轉換係數
        // Throttle速度: rpm -> m/s
        throttleConfig.encoder.velocityConversionFactor(SwerveConstants.kThrottleVelocityConversionFactor)
        // Throttle位置: 圈數 -> m
            .positionConversionFactor(SwerveConstants.kThrottlePositionConversionFactor);
        throttleMotor = new SparkMax(throttleID, SparkMax.MotorType.kBrushless);
        if (throttleMotor.configure(throttleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
            throw new RuntimeException(mLocation + " Throttle Config Error");
        }
        mThrottleEncoder = throttleMotor.getEncoder();
        
        SparkBaseConfig rotorConfig = new SparkMaxConfig()
            .voltageCompensation(Constants.kVoltageCompensation)
            .idleMode(IdleMode.kCoast)
            .inverted(rotorInverted);
        // 設定Encoder的轉換係數
        // Rotor位置: 圈數 -> 度
        rotorConfig.encoder.positionConversionFactor(SwerveConstants.kRotorPositionConversionFactor);
        rotorMotor = new SparkMax(rotorID, SparkMax.MotorType.kBrushless);
        if (rotorMotor.configure(rotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
            throw new RuntimeException(mLocation + " Rotor Config Error");
        }
    }

    @Override
    public void setThrottleSpeed(double speed) {
        // SmartDashboard.putNumber("Throttle Output " + location, speed);
        throttleMotor.set(ENABLE_OUTPUT ? speed : 0);
    }

    @Override
    public void setRotorSpeed(double speed) {
        // SmartDashboard.putNumber("Rotor Output " + location, speed);
        rotorMotor.set(ENABLE_OUTPUT ? speed : 0);
    }

    @Override
    public double getThrottlePosition() {
        return mThrottleEncoder.getPosition();
    }

    @Override 
    public double getThrottleVelocity() {
        return mThrottleEncoder.getVelocity();
    }
}
