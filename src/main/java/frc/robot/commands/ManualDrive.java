package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.ManualDriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveMode;

public class ManualDrive extends Command {
    
    private final Swerve mSwerve;
    private final CommandXboxController mJoystick;
    private boolean isFieldOriented = true;

    // ═══════════════ 射擊模式 ═══════════════
    // 啟用時：鎖定旋轉控制（z=0），降低平移速度
    // AutoAimAndShoot 的 PID 旋轉會透過 setAimSpeed 疊加，不受影響
    private volatile boolean shootingMode = false;
    private int telemetryCounter = 0;
    
    public ManualDrive(Swerve drive, CommandXboxController joystick) {
        mSwerve = drive;
        mJoystick = joystick;

        // Adds the Swerve subsystem as a requirement to the command
        // 加入 swerve 為這條命令的必要條件
        addRequirements(mSwerve);
    }

    @Override
    public void initialize() {
        mSwerve.setSwerveMode(SwerveMode.ROBOT_CENTRIC);
    }

    @Override
    public void execute() {
        // Xbox 搖桿慣例：Y 軸往前推 = -1，X 軸往左推 = -1
        // WPILib 慣例：前進 = +X，左移 = +Y
        // 在源頭反轉，後續所有邏輯都用 WPILib 標準正負號
        double xCtl = -mJoystick.getLeftY();  // 往前推 → +X (前進)
        double yCtl = -mJoystick.getLeftX();  // 往左推 → +Y (左移)
        double zCtl = mJoystick.getRightX();  // 右搖桿不反轉（順時針=正）

        // SmartDashboard.putNumber("xCtl", xCtl);
        // SmartDashboard.putNumber("yCtl", yCtl);
        // SmartDashboard.putNumber("zCtl", zCtl);
        
        // DEBUG: 原始手把輸入（節流輸出）
        boolean telemetryThisCycle = (++telemetryCounter >= Constants.kTelemetryDivider);
        if (telemetryThisCycle) {
            telemetryCounter = 0;
            SmartDashboard.putNumber("Raw/LeftY", mJoystick.getLeftY());
            SmartDashboard.putNumber("Raw/LeftX", mJoystick.getLeftX());
        }
        
        double boostTranslation = mJoystick.rightBumper().getAsBoolean()?1:0.7;
        // double boostTranslation = 1;
        
        xCtl = calculateNullZone(xCtl, ManualDriveConstants.kXDeadzone);
        xCtl *= ManualDriveConstants.kXMultiplier;
        xCtl *= boostTranslation;
        yCtl = calculateNullZone(yCtl, ManualDriveConstants.kYDeadzone);
        yCtl *= ManualDriveConstants.kYMultiplier;
        yCtl *= boostTranslation;
        zCtl = calculateNullZone(zCtl, ManualDriveConstants.kZDeadzone);
        zCtl *= ManualDriveConstants.kZMultiplier;

        // ═══════════════ 射擊模式 ═══════════════
        // 鎖定旋轉（AutoAim PID 會透過 setAimSpeed 疊加控制旋轉）
        // 降低平移速度，避免移動慣性導致球射偏目標
        if (shootingMode) {
            zCtl = 0; // 旋轉歸零，完全交給 AutoAim
            double shootingMultiplier = AutoAimConstants.kShootingModeSpeedMultiplier;
            xCtl *= shootingMultiplier;
            yCtl *= shootingMultiplier;
        }

        // 搖桿滿推 + Boost = kMaxPhysicalSpeedMps (MK4i L3 最大速度)
        // 搖桿滿推 不按Boost = kMaxPhysicalSpeedMps * 0.5
        xCtl *= SwerveConstants.kMaxPhysicalSpeedMps;
        yCtl *= SwerveConstants.kMaxPhysicalSpeedMps;
        zCtl *= SwerveConstants.kMaxPhysicalSpeedMps;

        mSwerve.setSpeed(xCtl, yCtl, zCtl, isFieldOriented);
        if (telemetryThisCycle) {
            SmartDashboard.putNumber("Drive/xSpeed", xCtl);
            SmartDashboard.putNumber("Drive/ySpeed", yCtl);
            SmartDashboard.putNumber("Drive/zSpeed", zCtl);
            SmartDashboard.putBoolean("Drive/fieldOriented", isFieldOriented);
            SmartDashboard.putBoolean("Drive/shootingMode", shootingMode);
        }
    }

    private double calculateNullZone(double input, double nullZone) {
        if (Math.abs(input) < nullZone) return 0;
        else {
            input += (input > 0 ? -nullZone : nullZone);
            input *= 1/(1-nullZone);
            return input;
        }
    }

    public void setIsFieldOriented(boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
    }

    public boolean getIsFieldOriented() {
        return isFieldOriented;
    }

    /**
     * 設定射擊模式。啟用時：
     * - 右搖桿旋轉輸入歸零（旋轉完全由 AutoAim PID 控制）
     * - 左搖桿平移速度降低為 kShootingModeSpeedMultiplier（預設 30%），避免慣性導致球射偏
     */
    public void setShootingMode(boolean enabled) {
        this.shootingMode = enabled;
    }

    public boolean isShootingMode() {
        return shootingMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Field Oriented", this::getIsFieldOriented, this::setIsFieldOriented);
    }
}   
