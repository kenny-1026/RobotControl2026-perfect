package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {
    public enum SwerveMode {
        ROBOT_CENTRIC,
        CAMERA_CENTRIC
    }
    private boolean isUseLimelight = false;
    private String limelightName;
    // Swerve Modules
    public SwerveModule mLeftFrontModule, mRightFrontModule, mLeftRearModule, mRightRearModule;
    private SwerveDriveOdometry mOdometry;
    private SwerveDrivePoseEstimator poseEstimator;
    // IMU
    private Pigeon2 mPigeonIMU;
    private StatusSignal<AngularVelocity> gyroAngularVelocitySignal; // 快取角速度 Signal
    // For Acceleration Constraints
    private ChassisSpeeds mTargetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    private ChassisSpeeds mAimChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    // SlewRateLimiter: 限制速度變化率（加速度），單位 m/s per second
    // 數值越大 = 加速越快，例如 20 表示每秒速度最多變化 20 m/s
    private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(20); // X 平移加速度限制
    private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(20); // Y 平移加速度限制
    private SlewRateLimiter zSpeedLimiter = new SlewRateLimiter(15); // 旋轉加速度限制

    public static final String kCANBusName = "DRIVETRAIN"; 

    private SwerveMode mSwerveMode = SwerveMode.ROBOT_CENTRIC;
    private boolean autoUpdateIMU = false;

    // 模擬用：追蹤陀螺儀角度
    private double simGyroAngleDeg = 0;

    private final Field2d m_field = new Field2d();

    // ═══════════════ Shuffleboard (Swerve Tab) ═══════════════
    private ShuffleboardTab swerveTab;
    private GenericEntry chassisVxEntry, chassisVyEntry, chassisOmegaEntry;
    private GenericEntry gyroAngleEntry;
    private GenericEntry mod0SpeedEntry, mod0AngleEntry;
    // Vision 遙測
    private GenericEntry visionTagCountEntry, visionRejectedEntry;
    private GenericEntry visionPoseXEntry, visionPoseYEntry;

    // ── 遙測節流 ──
    private int telemetryCounter = 0;
    private boolean telemetryThisCycle = false;

    public Swerve() {
        initFields();
        mOdometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveKinematics, 
            getGyroAngle(), 
            getModulePositions()
        );
    }

    public Swerve(String limelightName) {
        this.limelightName = limelightName;
        isUseLimelight = true;
        initFields();

        // 設定 Limelight 安裝位置（從 Constants 統一管理）
        LimelightHelpers.setCameraPose_RobotSpace(limelightName,
            Constants.LimelightConstants.kForwardMeters,
            Constants.LimelightConstants.kSideMeters,
            Constants.LimelightConstants.kUpMeters,
            Constants.LimelightConstants.kRollDegrees,
            Constants.LimelightConstants.kPitchDegrees,
            Constants.LimelightConstants.kYawDegrees);

        // 模擬環境：將初始位置設在場地中央，避免因紅方速度反轉跑出場地邊界看不見
        Pose2d initialPose;
        if (RobotBase.isSimulation()) {
            initialPose = new Pose2d(
                Constants.AutoAimConstants.kFieldLengthMeters / 2.0,
                Constants.AutoAimConstants.kFieldWidthMeters / 2.0,
                Rotation2d.fromDegrees(0));
            simGyroAngleDeg = 0;
        } else {
            initialPose = new Pose2d();
        }

        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.kSwerveKinematics,
                getGyroAngle(),
                getModulePositions(),
                initialPose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    /**
     * 設定 Shuffleboard Swerve 分頁（由 RobotContainer 在建構後呼叫）。
     */
    public void setupShuffleboardTab(ShuffleboardTab tab) {
        this.swerveTab = tab;

        chassisVxEntry = tab.add("Chassis vx", 0)
                .withWidget(BuiltInWidgets.kGraph).withSize(3, 2).withPosition(0, 0).getEntry();
        chassisVyEntry = tab.add("Chassis vy", 0)
                .withWidget(BuiltInWidgets.kGraph).withSize(3, 2).withPosition(3, 0).getEntry();
        chassisOmegaEntry = tab.add("Chassis omega", 0)
                .withWidget(BuiltInWidgets.kGraph).withSize(3, 2).withPosition(6, 0).getEntry();
        gyroAngleEntry = tab.add("Gyro Angle", 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(0, 2).getEntry();
        mod0SpeedEntry = tab.add("Mod0 Speed", 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(2, 2).getEntry();
        mod0AngleEntry = tab.add("Mod0 Angle", 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(4, 2).getEntry();

        // ── Vision 遙測 ──
        visionTagCountEntry = tab.add("Vision Tags", 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(6, 2).getEntry();
        visionRejectedEntry = tab.add("Vision Rejected", false)
                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1).withPosition(8, 2).getEntry();
        visionPoseXEntry = tab.add("Vision X", 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(6, 3).getEntry();
        visionPoseYEntry = tab.add("Vision Y", 0)
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(8, 3).getEntry();
    }

    /**
     * 取得 Field2d 物件（供 ShuffleboardManager.setupMainTab 使用）。
     */
    public Field2d getField2d() {
        return m_field;
    }

    private void initFields() {
        mPigeonIMU = new Pigeon2(SwerveConstants.kPigeonID);

        // ── 快取 Pigeon2 Signal 並設定 CAN 更新頻率 ──
        // Yaw: getRotation2d() 內部會讀取，設定 100Hz（里程計關鍵路徑）
        mPigeonIMU.getYaw().setUpdateFrequency(100);
        // 角速度: 用於判斷是否融合 Limelight 視覺資料
        gyroAngularVelocitySignal = mPigeonIMU.getAngularVelocityZDevice();
        gyroAngularVelocitySignal.setUpdateFrequency(50); // 50Hz 足夠
        mPigeonIMU.optimizeBusUtilization();


        mLeftFrontModule = new SwerveModuleKraken(
            SwerveConstants.kLeftFrontThrottleID, 
            SwerveConstants.kLeftFrontRotorID, 
            SwerveConstants.kLeftFrontThrottleInverted,
            SwerveConstants.kLeftFrontRotorInverted,
            SwerveConstants.kLeftFrontCANCoderID, 
            "left front"
        );

        mRightFrontModule = new SwerveModuleKraken(
            SwerveConstants.kRightFrontThrottleID, 
            SwerveConstants.kRightFrontRotorID, 
            SwerveConstants.kRightFrontThrottleInverted,
            SwerveConstants.kRightFrontRotorInverted,
            SwerveConstants.kRightFrontCANCoderID, 
            "right front"
        );

        mLeftRearModule = new SwerveModuleKraken(
            SwerveConstants.kLeftRearThrottleID, 
            SwerveConstants.kLeftRearRotorID, 
            SwerveConstants.kLeftRearThrottleInverted,
            SwerveConstants.kLeftRearRotorInverted,
            SwerveConstants.kLeftRearCANCoderID, 
            "left rear"
        );

        mRightRearModule = new SwerveModuleKraken(
            SwerveConstants.kRightRearThrottleID, 
            SwerveConstants.kRightRearRotorID, 
            SwerveConstants.kRightRearThrottleInverted,
            SwerveConstants.kRightRearRotorInverted,
            SwerveConstants.kRightRearCANCoderID, 
            "right rear"
        );

        setSpeed(0, 0, 0, false);

        // ⚠️ 不要在這裡呼叫 SmartDashboard.putData("Field", m_field)！
        // Field2d 會在 ShuffleboardManager.setupMainTab() 中透過 Shuffleboard API 發佈。
        // 同一個 Sendable 物件只能綁定到一個 NT 路徑，雙重發佈會導致其中一個不更新。
    }

    @Override
    public void periodic() {
        // ── 遙測節流：每 kTelemetryDivider 個週期才輸出一次 ──
        telemetryThisCycle = (++telemetryCounter >= Constants.kTelemetryDivider);
        if (telemetryThisCycle) {
            telemetryCounter = 0;
        }

        // 取得基本感測器數據
        Rotation2d gyroAngle = getGyroAngle();
        SwerveModulePosition[] modulePositions = getModulePositions();

        if (isUseLimelight && poseEstimator != null) {
            // ① 底盤推算更新（主要座標來源）
            poseEstimator.update(gyroAngle, modulePositions);

            // ② Limelight 視覺融合（輔助校正，防止打滑導致座標偏移）
            if (limelightName != null) {
                // 告訴 Limelight 目前 IMU 角度，讓 MegaTag2 演算法更精準
                LimelightHelpers.SetRobotOrientation_NoFlush(limelightName, 
                    gyroAngle.getDegrees(), getGyroRateDps(), 0, 0, 0, 0);
                
                // 統一使用藍方原點座標系（WPILib 標準）
                // PathPlanner 的 isAllianceRed() 會自動鏡射路徑，不需要手動切換紅方座標
                LimelightHelpers.PoseEstimate mt2 = 
                    LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                
                boolean doRejectUpdate = false;

                if (mt2 == null || mt2.tagCount == 0 
                    || mt2.rawFiducials == null || mt2.rawFiducials.length == 0) {
                    // 沒看到任何 AprilTag → 不更新
                    doRejectUpdate = true;
                } else {
                    // ── 過濾不可靠的視覺資料 ──
                    if (mt2.tagCount == 1) {
                        // 只看到 1 個 Tag：檢查可靠度
                        if (mt2.rawFiducials[0].ambiguity > Constants.LimelightConstants.kMaxAmbiguity) doRejectUpdate = true;
                        if (mt2.rawFiducials[0].distToCamera > Constants.LimelightConstants.kMaxTagDistMeters) doRejectUpdate = true;
                    }
                    // 多個 Tag（tagCount >= 2）：交叉定位精度高，跳過 ambiguity/distance 檢查

                    // 機器人高速旋轉時影像模糊，不融合
                    if (Math.abs(getGyroRateDps()) > Constants.LimelightConstants.kMaxGyroRateDps) doRejectUpdate = true;
                }

                if (!doRejectUpdate) {
                    // ── 動態信任度：看到越多 Tag → 越信任 Limelight ──
                    double xyStdDev;
                    if (mt2.tagCount >= 2) {
                        xyStdDev = Constants.LimelightConstants.kMultiTagXYStdDev;
                    } else {
                        // 單 Tag：根據距離動態調整（越遠越不信任）
                        double dist = mt2.rawFiducials[0].distToCamera;
                        xyStdDev = Constants.LimelightConstants.kSingleTagBaseStdDev
                                 + (dist * Constants.LimelightConstants.kSingleTagDistScale);
                    }

                    // 角度永遠不融合，完全信任 IMU
                    poseEstimator.setVisionMeasurementStdDevs(
                        VecBuilder.fill(xyStdDev, xyStdDev, Constants.LimelightConstants.kAngleStdDev));
                    poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                }

                // ── Vision 遙測輸出到 Shuffleboard ──
                if (telemetryThisCycle && visionTagCountEntry != null) {
                    visionTagCountEntry.setDouble(mt2 != null ? mt2.tagCount : 0);
                    visionRejectedEntry.setBoolean(doRejectUpdate);
                    if (mt2 != null && mt2.tagCount > 0) {
                        visionPoseXEntry.setDouble(mt2.pose.getX());
                        visionPoseYEntry.setDouble(mt2.pose.getY());
                    }
                }
            }
        } else if (mOdometry != null) {
            mOdometry.update(gyroAngle, modulePositions);
        }

        // 更新 Field2d
        m_field.setRobotPose(getPose());

        // 更新 Swerve Tab 陀螺儀角度
        if (telemetryThisCycle && gyroAngleEntry != null) {
            gyroAngleEntry.setDouble(gyroAngle.getDegrees());
        }

        // 每個週期都執行馬達控制（原本靠獨立的 setStateCommand 排程，容易漏掉）
        if (edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
            runSetStates();
        }
    }

    public boolean isAllianceRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false; //預設藍色
    }

    private void runSetStates() {
        var sumChassisSpeeds = new ChassisSpeeds(
            mTargetChassisSpeeds.vxMetersPerSecond + mAimChassisSpeeds.vxMetersPerSecond,
            mTargetChassisSpeeds.vyMetersPerSecond + mAimChassisSpeeds.vyMetersPerSecond,
            mTargetChassisSpeeds.omegaRadiansPerSecond + mAimChassisSpeeds.omegaRadiansPerSecond
        );

        // SlewRateLimiter: 直接限制速度變化率（加速度），不會提前減速
        double xSpeed = xSpeedLimiter.calculate(sumChassisSpeeds.vxMetersPerSecond);
        double ySpeed = ySpeedLimiter.calculate(sumChassisSpeeds.vyMetersPerSecond);
        double zSpeed = zSpeedLimiter.calculate(sumChassisSpeeds.omegaRadiansPerSecond);

        var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);

        if (telemetryThisCycle) {
            if (chassisVxEntry != null) {
                chassisVxEntry.setDouble(chassisSpeeds.vxMetersPerSecond);
                chassisVyEntry.setDouble(chassisSpeeds.vyMetersPerSecond);
                chassisOmegaEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond);
            } else {
                SmartDashboard.putNumber("Chassis/vx", chassisSpeeds.vxMetersPerSecond);
                SmartDashboard.putNumber("Chassis/vy", chassisSpeeds.vyMetersPerSecond);
                SmartDashboard.putNumber("Chassis/omega", chassisSpeeds.omegaRadiansPerSecond);
            }
        }

        SwerveModuleState[] mStates;
        if (mSwerveMode == SwerveMode.ROBOT_CENTRIC) 
            mStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
        else
            mStates = SwerveConstants.kSwerveKinematicsCamera.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(mStates);

        // 模擬環境：用 omega 積分更新模擬陀螺儀角度
        if (RobotBase.isSimulation()) {
            simGyroAngleDeg += Math.toDegrees(zSpeed) * 0.02;
        }
    }
 
    public void run() {
        // runSetStates() 已移至 periodic() 中，不再需要獨立排程
        resetPID();
    }

    public SwerveMode getSwerveMode() {
        return mSwerveMode;
    }

    public void setSwerveMode(SwerveMode mode) {
        if (mode != mSwerveMode){
            mSwerveMode = mode;
        }
    }

    private Rotation2d getGyroAngle() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromDegrees(simGyroAngleDeg);
        }
        return mPigeonIMU.getRotation2d();
        // return mPigeonIMU.getRotation2d().plus(Rotation2d.fromDegrees(180)); //260220
        // return mPigeonIMU.getRotation2d().unaryMinus().plus(Rotation2d.fromDegrees(180));
    }

    private double getGyroRateDps() {
        if (RobotBase.isSimulation()) {
            return 0; // 模擬中不需要精確的角速度
        }
        return gyroAngularVelocitySignal.refresh().getValueAsDouble();
    }

    /***
     * 
     * @return gyro degrees between 180 and -180
     */
    private double getGyroAngleDegrees() {
        return MathUtil.inputModulus(getGyroAngle().getDegrees(), -180, 180);
    }

    /**
     * 設定底盤移動速度。
     * 呼叫端應傳入 WPILib 標準正負號：+X=前進, +Y=左移, +Z=逆時針旋轉
     * 
     * @param xSpeed 前後速度 (m/s)，正 = 前進
     * @param ySpeed 左右速度 (m/s)，正 = 左移
     * @param zSpeed 旋轉速度 (rad/s)
     * @param fieldOriented true = 場地導向, false = 機器人導向
     */

    public void setSpeed(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        if (fieldOriented) {
            // 如果我們在紅方，駕駛員的「前進」和「向左」相對於場地座標是 180° 旋轉的
            if (isAllianceRed()) {
                xSpeed = -xSpeed;
                ySpeed = -ySpeed;
            }
            mTargetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getGyroAngle());
        } else {
            mTargetChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        }
    }

    public void setAimSpeed(ChassisSpeeds speeds) {
        mAimChassisSpeeds = speeds;
    }
    
    /**
     * Sets the target chassis speeds for autonomous control
     *
     * @param speeds The desired chassis speeds, represented as a ChassisSpeeds object.
     *               - vxMetersPerSecond: The forward velocity in meters per second.
     *               - vyMetersPerSecond: The sideways velocity in meters per second.
     *               - omegaRadiansPerSecond: The angular velocity in radians per second.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        mTargetChassisSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond*0.185, speeds.vyMetersPerSecond*0.185, speeds.omegaRadiansPerSecond*0.21);
    }

    public void zeroRotor() {
        mLeftFrontModule.setRotorAngle(0);
        mLeftFrontModule.setThrottleSpeed(0);
        mRightFrontModule.setRotorAngle(0);
        mRightFrontModule.setThrottleSpeed(0);
        mLeftRearModule.setRotorAngle(0);
        mLeftRearModule.setThrottleSpeed(0);
        mRightRearModule.setRotorAngle(0);
        mRightRearModule.setThrottleSpeed(0);
    }
    /**
     * Get current swerve module states
     * 輸出 4 個 Swerve Module 的當前狀態 modules
     * 
     * @return swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        };
    }

    /**
     * Get current swerve module positions
     * 
     * @return swerve module positions 
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            mLeftFrontModule.getPosition(), 
            mRightFrontModule.getPosition(), 
            mLeftRearModule.getPosition(), 
            mRightRearModule.getPosition()
        };
    }

    /**
     * Sets swerve module states
     * 設置 4 個 Swerve module 的狀態。
     * 
     * @param desiredStates array of desired states, order: [leftFront, leftRear, rightFront, rightRear]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getMaxVelocity());

        if (telemetryThisCycle) {
            if (mod0SpeedEntry != null) {
                mod0SpeedEntry.setDouble(desiredStates[0].speedMetersPerSecond);
                mod0AngleEntry.setDouble(desiredStates[0].angle.getDegrees());
            } else {
                SmartDashboard.putNumber("Module0/speed", desiredStates[0].speedMetersPerSecond);
                SmartDashboard.putNumber("Module0/angle", desiredStates[0].angle.getDegrees());
            }
        }

        mLeftFrontModule.setState(desiredStates[0]);
        mRightFrontModule.setState(desiredStates[1]);
        mLeftRearModule.setState(desiredStates[2]);
        mRightRearModule.setState(desiredStates[3]);
    }

    /**
     * Get predicted pose
     * 獲取機器人的當前位置
     * 
     * @return pose
     */
    public Pose2d getPose() {
        if (isUseLimelight) {
            return poseEstimator.getEstimatedPosition();
        }
        return mOdometry.getPoseMeters();
    }

    /**
     * Set robot pose — 同時同步 IMU yaw。
     * 將測程法（odometry）位置設置為給與的 x、y、位置和角度，
     * 並把 Pigeon2 的 yaw 設為 pose 的旋轉角度，確保 field-oriented 驅動正確。
     * 
     * @param pose robot pose（藍方原點座標系）
     */
    public void setPose(Pose2d pose) {
        // ── 同步 IMU yaw，讓 field-oriented 驅動的角度基準與位姿一致 ──
        double headingDeg = pose.getRotation().getDegrees();
        mPigeonIMU.setYaw(headingDeg);
        if (RobotBase.isSimulation()) {
            simGyroAngleDeg = headingDeg;
        }

        Rotation2d newGyroAngle = getGyroAngle();
        if (isUseLimelight) {
            poseEstimator.resetPosition(newGyroAngle, getModulePositions(), pose);
        } else {
            mOdometry.resetPosition(newGyroAngle, getModulePositions(), pose);
        }
    }

    /**
     * PathPlanner 在 auto 開始時呼叫的方法（resetOdom: true）。
     * 現在直接委派給 setPose()，因為 setPose() 已經會同步 IMU yaw。
     */
    public void resetPose(Pose2d pose) {
        setPose(pose);
    }
    public double getMaxVelocity() {
        // 使用 Constants 中根據齒輪比和輪徑計算出的物理最大速度
        return SwerveConstants.kMaxPhysicalSpeedMps;
    }

    /**
     * 專門給 PathPlanner 使用的驅動方法
     * 接收機器人相對速度 (Robot Relative ChassisSpeeds)
     */
    public void drive(ChassisSpeeds speeds) {        
        SwerveModuleState[] swerveModuleStates = 
            SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds);

        // 進行速度飽和限制 (Desaturate)
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getMaxVelocity());

        // 設定模組狀態
        setModuleStates(swerveModuleStates);
    }

    /**
     * 用 Limelight 校正 XY 位置，角度保持使用 IMU。
     * 統一使用藍方原點座標系（WPILib 標準）。
     * 在 teleopInit() 呼叫，消除 Auto 累積的位移誤差。
     */
    public void resetPoseToLimelight() {
        LimelightHelpers.PoseEstimate mt2 = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (mt2 != null && mt2.tagCount > 0) {
            // 只使用 Limelight 的 XY 位置，角度保持 IMU 的值
            Pose2d correctedPose = new Pose2d(
                mt2.pose.getTranslation(),  // XY 來自 Limelight
                getGyroAngle()              // 角度來自 IMU
            );
            this.setPose(correctedPose);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        );
    }

    /**
     * 緊急重設 IMU（綁定在按鈕 8）。
     * 自動偵測聯盟色：藍方 → 0°，紅方 → 180°（WPILib 藍方原點座標系）。
     */
    public void resetIMU() {
        resetIMU(isAllianceRed() ? 180.0 : 0.0);
    }

    public void resetIMU(double angle) {
        // 保留位置 XY，只將航向重設為指定角度
        // setPose() 內部會同步 IMU yaw + poseEstimator/odometry
        Pose2d currentPose = getPose();
        Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(angle));
        setPose(correctedPose);
    }

    public void updateSmartDashboard() {
        // mLeftFrontModule.updateSmartDashboard();
        // mRightFrontModule.updateSmartDashboard();
        // mLeftRearModule.updateSmartDashboard();
        // mRightRearModule.updateSmartDashboard();
    }

    public void resetPID() {
        System.out.println("Swerve.resetPID");
        mLeftFrontModule.resetPID();
        mRightFrontModule.resetPID();
        mLeftRearModule.resetPID();
        mRightRearModule.resetPID();
        xSpeedLimiter.reset(0);
        ySpeedLimiter.reset(0);
        zSpeedLimiter.reset(0);
    }

    public void disabledInit() {
        mLeftFrontModule.setRotorSpeed(0);
        mLeftFrontModule.setThrottleSpeed(0);
        mRightFrontModule.setRotorSpeed(0);
        mRightFrontModule.setThrottleSpeed(0);
        mLeftRearModule.setRotorSpeed(0);
        mLeftRearModule.setThrottleSpeed(0);
        mRightRearModule.setRotorSpeed(0);
        mRightRearModule.setThrottleSpeed(0);
    }

    public boolean isAutoUpdateIMU() {
        return autoUpdateIMU;
    }

    public void setAutoUpdateIMU(boolean autoUpdateIMU) {
        this.autoUpdateIMU = autoUpdateIMU;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("AutoUpdateIMU", this::isAutoUpdateIMU, this::setAutoUpdateIMU);
    }
}
