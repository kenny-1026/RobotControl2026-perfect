package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.TunableNumber;

public abstract class SwerveModule {
    protected static final boolean ENABLE_OUTPUT = true;

    protected String mLocation;

    // Initialize rotor encoder
    // 初始化 rotor encoder
    private CANcoder mRotorEncoder;
    private StatusSignal<Angle> mRotorAbsolutePositionSignal; // 快取 CANcoder signal

    // Initialize rotor PID controller
    // 初始化 rotor PID controller
    private ProfiledPIDController mRotorPID;
    private RelativeEncoder mRotorRelativeEncoder;

    private boolean mUseAbsoluteEncoder = false;

    // ── Glass 即時調參（靜態，所有 SwerveModule 共用同一組 PID 參數） ──
    private static final TunableNumber tunableRotorKP = new TunableNumber("Swerve/Rotor kP", SwerveConstants.kRotor_kP);
    private static final TunableNumber tunableRotorKI = new TunableNumber("Swerve/Rotor kI", SwerveConstants.kRotor_kI);
    private static final TunableNumber tunableRotorKD = new TunableNumber("Swerve/Rotor kD", SwerveConstants.kRotor_kD);

    // Construct swerve module with relative encoder
    public SwerveModule(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted, String location) {
        this.mLocation = location;
        mUseAbsoluteEncoder = false;

        initMotors(throttleID, rotorID, throttleInverted, rotorInverted);
        initPID();
    }
    /**
     * Construct SwerveModule with absolute encoder
     *
     * @param throttleID          CAN ID of throttle 馬達
     * @param rotorID             CAN ID of rotor 馬達
     * @param rotorEncoderID      CAN ID of rotor encoder (not used for now)
     */
    public SwerveModule(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted, int rotorEncoderID, String location) {
        mLocation = location;
        mUseAbsoluteEncoder = true;
        
        initMotors(throttleID, rotorID, throttleInverted, rotorInverted);
        initPID();

        mRotorEncoder = new CANcoder(rotorEncoderID, SwerveConstants.kDrivetrainCANBus);
         // Configure absolute encoder

        // 快取 CANcoder Signal 並設定更新頻率（里程計關鍵路徑）
        mRotorAbsolutePositionSignal = mRotorEncoder.getAbsolutePosition();
        mRotorAbsolutePositionSignal.setUpdateFrequency(100); // 100Hz
        mRotorEncoder.optimizeBusUtilization();
    }

    protected abstract void initMotors(int throttleID, int rotorID, boolean throttleInverted, boolean rotorInverted);


    private void initPID() {
        mRotorPID = new ProfiledPIDController(
                SwerveConstants.kRotor_kP,
                SwerveConstants.kRotor_kI,
                SwerveConstants.kRotor_kD,
                new TrapezoidProfile.Constraints(
                    SwerveConstants.kRotor_maxVelocity,
                    SwerveConstants.kRotor_maxAcceleration));
        mRotorPID.setIntegratorRange(-0.5, 0.5);

        // ContinuousInput 認為 min 和 max 是同一點並且自動計算到設定點的最短路線
        mRotorPID.enableContinuousInput(-180, 180);
        mRotorPID.setTolerance(1);
    }

    /**
     * Get current position of rotor
     * 
     * @return rotor position in degrees (-180 ~ 180)
     */
    public double getRotorPosition() {
        if (mUseAbsoluteEncoder) {
            // 絕對位置Encoder（使用快取 Signal）
            return mRotorAbsolutePositionSignal.refresh().getValueAsDouble()*360; // 度
        }
        else {
            // 相對位置Encoder
            double rotorPosition = mRotorRelativeEncoder.getPosition(); // 度
            rotorPosition = ((rotorPosition + 180) % 360 + 360) % 360 - 180; // 限制在 -180 ~ 180 度
            return rotorPosition;
        }
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getThrottleVelocity(),
                Rotation2d.fromDegrees(getRotorPosition()));
    }

    /**
     * Return current position of module
     * 
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getThrottlePosition(),
                Rotation2d.fromDegrees(getRotorPosition()));
    }

    /**
     * Set module state
     * 
     * @param state module state
     */
    public void setState(SwerveModuleState state) {
        // SmartDashboard.putNumber("Rotor Target " + location, state.angle.getDegrees());
        // SmartDashboard.putNumber("Throttle Target " + location, state.speedMetersPerSecond);

        // ── 檢查 Glass 即時調參 ──
        checkTunableUpdates();

        // 只讀一次 CAN，快取角度值，避免重複 CAN 通訊
        Rotation2d currentAngle = getState().angle;

        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        state.optimize(currentAngle);
        state.cosineScale(currentAngle);

        // SmartDashboard.putNumber("Rotor Target Optimized " + location, optimizedState.angle.getDegrees());
        // SmartDashboard.putNumber("Throttle Target Optimized " + location, optimizedState.speedMetersPerSecond);

        runRotorPID(currentAngle, state.angle.getDegrees());
        setThrottleSpeed(state.speedMetersPerSecond);
    }

    public void setRotorAngle(double angle) {
        runRotorPID(getState().angle, angle);
    }
    
    public abstract void setThrottleSpeed(double speed);
    public abstract void setRotorSpeed(double speed);

    private void runRotorPID(Rotation2d currentAngle, double targetAngle) {
        // 通過比較目前角度與目標角度來用 PID 控制器計算轉向馬達所需的輸出
        double rotorOutput = mRotorPID.calculate(currentAngle.getDegrees(), targetAngle);
        setRotorSpeed(mRotorPID.atSetpoint() ? 0 : rotorOutput);
    }

    public void updateSmartDashboard() {
        // SmartDashboard.putNumber("Rotor Angle " + mLocation, getState().angle.getDegrees());
    }

    /**
     * 檢查 Glass 上的 Rotor PID 參數是否有變更，有就即時套用。
     * 由於四個 SwerveModule 共用同一組 TunableNumber（靜態欄位），
     * 任何一個模組偵測到變更後都會更新自己的 PIDController。
     */
    public void checkTunableUpdates() {
        if (tunableRotorKP.hasChanged() || tunableRotorKI.hasChanged() || tunableRotorKD.hasChanged()) {
            mRotorPID.setPID(tunableRotorKP.get(), tunableRotorKI.get(), tunableRotorKD.get());
        }
    }

    public void resetPID() {
        mRotorPID.reset(getState().angle.getDegrees());
    }

    public abstract double getThrottlePosition();
    public abstract double getThrottleVelocity();
}
