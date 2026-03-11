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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    public enum SwerveMode {
        ROBOT_CENTRIC,
        CAMERA_CENTRIC
    }
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
    // SlewRateLimiter
    private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(20);
    private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(20);
    private SlewRateLimiter zSpeedLimiter = new SlewRateLimiter(15);

    public static final String kCANBusName = "DRIVETRAIN"; 

    private SwerveMode mSwerveMode = SwerveMode.ROBOT_CENTRIC;
    private boolean autoUpdateIMU = false;

    // 模擬用
    private double simGyroAngleDeg = 0;

    private final Field2d m_field = new Field2d();

    // ═══════════════ Shuffleboard ═══════════════
    private ShuffleboardTab swerveTab;
    private GenericEntry chassisVxEntry, chassisVyEntry, chassisOmegaEntry;
    private GenericEntry gyroAngleEntry;
    private GenericEntry mod0SpeedEntry, mod0AngleEntry;

    // ── 遙測節流 ──
    private int telemetryCounter = 0;
    private boolean telemetryThisCycle = false;

    public Swerve() {
        initFields();

        Pose2d initialPose = new Pose2d();
        if (RobotBase.isSimulation()) {
            initialPose = new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(0));
            simGyroAngleDeg = 0;
        }

        mOdometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveKinematics, 
            getGyroAngle(), 
            getModulePositions(),
            initialPose
        );

        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.kSwerveKinematics,
                getGyroAngle(),
                getModulePositions(),
                initialPose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

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
    }

    public Field2d getField2d() {
        return m_field;
    }

    private void initFields() {
        mPigeonIMU = new Pigeon2(SwerveConstants.kPigeonID);
        mPigeonIMU.getYaw().setUpdateFrequency(100);
        gyroAngularVelocitySignal = mPigeonIMU.getAngularVelocityZDevice();
        gyroAngularVelocitySignal.setUpdateFrequency(50);
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
    }

    @Override
    public void periodic() {
        telemetryThisCycle = (++telemetryCounter >= Constants.kTelemetryDivider);
        if (telemetryThisCycle) {
            telemetryCounter = 0;
        }

        Rotation2d gyroAngle = getGyroAngle();
        SwerveModulePosition[] modulePositions = getModulePositions();

        poseEstimator.update(gyroAngle, modulePositions);
        mOdometry.update(gyroAngle, modulePositions);

        m_field.setRobotPose(getPose());

        if (telemetryThisCycle && gyroAngleEntry != null) {
            gyroAngleEntry.setDouble(gyroAngle.getDegrees());
        }

        if (edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
            runSetStates();
        }
    }

    public boolean isAllianceRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    private void runSetStates() {
        var sumChassisSpeeds = new ChassisSpeeds(
            mTargetChassisSpeeds.vxMetersPerSecond + mAimChassisSpeeds.vxMetersPerSecond,
            mTargetChassisSpeeds.vyMetersPerSecond + mAimChassisSpeeds.vyMetersPerSecond,
            mTargetChassisSpeeds.omegaRadiansPerSecond + mAimChassisSpeeds.omegaRadiansPerSecond
        );

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
            mStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds); // 暫時移除 Camera Centric
        setModuleStates(mStates);

        if (RobotBase.isSimulation()) {
            simGyroAngleDeg += Math.toDegrees(zSpeed) * 0.02;
        }
    }
 
    public void run() {
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
    }

    private double getGyroRateDps() {
        if (RobotBase.isSimulation()) {
            return 0;
        }
        return gyroAngularVelocitySignal.refresh().getValueAsDouble();
    }

    public void setSpeed(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        if (fieldOriented) {
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
    
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        mTargetChassisSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            mLeftFrontModule.getPosition(), 
            mRightFrontModule.getPosition(), 
            mLeftRearModule.getPosition(), 
            mRightRearModule.getPosition()
        };
    }

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

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        double headingDeg = pose.getRotation().getDegrees();
        mPigeonIMU.setYaw(headingDeg);
        if (RobotBase.isSimulation()) {
            simGyroAngleDeg = headingDeg;
        }

        Rotation2d newGyroAngle = getGyroAngle();
        poseEstimator.resetPosition(newGyroAngle, getModulePositions(), pose);
        mOdometry.resetPosition(newGyroAngle, getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        setPose(pose);
    }
    public double getMaxVelocity() {
        return SwerveConstants.kMaxPhysicalSpeedMps;
    }

    public void drive(ChassisSpeeds speeds) {        
        SwerveModuleState[] swerveModuleStates = 
            SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getMaxVelocity());
        setModuleStates(swerveModuleStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        );
    }

    public void resetIMU() {
        resetIMU(isAllianceRed() ? 180.0 : 0.0);
    }

    public void resetIMU(double angle) {
        Pose2d currentPose = getPose();
        Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(angle));
        setPose(correctedPose);
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
