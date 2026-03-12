package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive2TagConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class Drive2Tag extends Command {
    private final Swerve m_swerve;
    private final String m_limelightName;

    // 定義 PID Controller
    private final PIDController xController = new PIDController(
        Drive2TagConstants.kX_kP, Drive2TagConstants.kX_kI, Drive2TagConstants.kX_kD);
    private final PIDController yController = new PIDController(
        Drive2TagConstants.kY_kP, Drive2TagConstants.kY_kI, Drive2TagConstants.kY_kD);
    private final PIDController thetaController = new PIDController(
        Drive2TagConstants.kTheta_kP, Drive2TagConstants.kTheta_kI, Drive2TagConstants.kTheta_kD);

    // 新增變數：用來存儲我們想要的目標位置
    private final double m_targetX;
    private final double m_targetY;
    private final double m_targetYaw;

    /**
     * 建構子
     * @param swerve 底盤
     * @param limelightName Limelight 名稱
     * @param targetX 目標前後距離 (公尺) - 例如 -1.15 代表 Tag 在機器人前方 1.15m（Limelight tx, forward）
     * @param targetY 目標左右偏移 (公尺) - 0 = 正對, 正數=左偏, 負數=右偏（Limelight ty, left）
     * @param targetYaw 目標角度 (度) - 通常是 0（正對 Tag）
     */
    public Drive2Tag(Swerve swerve, String limelightName, double targetX, double targetY, double targetYaw) {
        m_swerve = swerve;
        m_limelightName = limelightName;

        // 儲存傳進來的目標參數
        m_targetX = targetX;
        m_targetY = targetY;
        m_targetYaw = targetYaw;

        addRequirements(m_swerve);

        // 設定容許誤差
        xController.setTolerance(Drive2TagConstants.kXTolerance);
        yController.setTolerance(Drive2TagConstants.kYTolerance);
        thetaController.setTolerance(Drive2TagConstants.kThetaTolerance);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(m_limelightName, 1);
        // 清除可能殘留的 AutoAim 旋轉疊加，避免兩個 Command 打架
        m_swerve.setAimSpeed(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public void execute() {
        // SmartDashboard.putBoolean("Drive2Tag/IsRunning", true);

        boolean hasTarget = LimelightHelpers.getTV(m_limelightName);
        // SmartDashboard.putBoolean("Drive2Tag/HasTarget", hasTarget);

        if (hasTarget) {
            double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(m_limelightName);

            if (targetPose != null && targetPose.length >= 6) {
                // Limelight targetpose_robotspace 陣列格式:
                // [0]=tx(前後/forward), [1]=ty(左右/left), [2]=tz(上下/up),
                // [3]=pitch, [4]=yaw, [5]=roll
                // WPILib ChassisSpeeds: vx=前後, vy=左右
                // → currentX(前後) 對應 targetPose[0], currentY(左右) 對應 targetPose[1]
                double currentX = targetPose[0];  // 前後距離 (forward)
                double currentY = targetPose[1];  // 左右偏移 (left)
                double currentYaw = targetPose[4]; // 偏航角 (yaw)

                // SmartDashboard.putNumber("Drive2Tag/Dist_X", currentX);
                // SmartDashboard.putNumber("Drive2Tag/Dist_Y", currentY);
                // SmartDashboard.putNumber("Drive2Tag/Angle_Yaw", currentYaw);

                // --- 關鍵修改：使用建構子傳進來的目標變數 ---
                double xSpeed = xController.calculate(currentX, m_targetX);
                double ySpeed = yController.calculate(currentY, m_targetY); // 這裡不再是寫死的 0
                double thetaSpeed = thetaController.calculate(currentYaw, m_targetYaw); // 這裡不再是寫死的 -15

                // SmartDashboard.putBoolean("Drive2Tag/AtSetpoint", xController.atSetpoint());

                xSpeed *= Drive2TagConstants.kSpeedMultiplier;
                ySpeed *= Drive2TagConstants.kSpeedMultiplier;
                thetaSpeed *= Drive2TagConstants.kSpeedMultiplier;

                // 限制輸出速度 (Clamp)
                xSpeed = Math.max(-Drive2TagConstants.kMaxTranslationSpeed, Math.min(Drive2TagConstants.kMaxTranslationSpeed, xSpeed));
                ySpeed = Math.max(-Drive2TagConstants.kMaxTranslationSpeed, Math.min(Drive2TagConstants.kMaxTranslationSpeed, ySpeed));
                thetaSpeed = Math.max(-Drive2TagConstants.kMaxRotationSpeed, Math.min(Drive2TagConstants.kMaxRotationSpeed, thetaSpeed));


                // xSpeed = 0;
                // ySpeed = 0;
                // thetaSpeed = 0;

                m_swerve.setSpeed(xSpeed, ySpeed, thetaSpeed, false);
            } else {
                // SmartDashboard.putString("Drive2Tag/Error", "Data Invalid");
                m_swerve.setSpeed(0, 0, 0, false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.setSpeed(0, 0, 0, false);
        m_swerve.setAimSpeed(new ChassisSpeeds(0, 0, 0));
        LimelightHelpers.setPipelineIndex(m_limelightName, 0);
    }
}