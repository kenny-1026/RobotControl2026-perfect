// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.commands.Drive2Tag;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransportSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.*;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShuffleboardManager;

import java.util.logging.Logger;

import com.ctre.phoenix6.SignalLogger;

public class RobotContainer {

        // SignalLogger logger = new SignalLogger();
        // private final Timer timer = new Timer();
        private int printCounter = 0;
        private final Swerve swerve = new Swerve(Constants.kLimelightName);
        private final CommandXboxController driverController = new CommandXboxController(
                        OperatorConstants.kSwerveControllerPort);

        private final ManualDrive manualDriveCommand = new ManualDrive(swerve, driverController);
        private final SendableChooser<Command> autoChooser;

        // ═══════════════ Shuffleboard ═══════════════
        private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shuffleboardManager.getShooterTab());
        // private final IntakeArmSubsystem intakeArm = new
        // IntakeArmSubsystem(shuffleboardManager.getIntakeArmTab());
        private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem(
                        shuffleboardManager.getIntakeRollerTab());
        private final TransportSubsystem transport = new TransportSubsystem();
        private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();

        private Command autoCommand;

        // ── 距離自適應輔助方法 ──
        // 根據機器人目前位置計算到 Hub 的距離，查表取得目標 RPS
        private double getAdaptiveRps() {
                var robotPos = swerve.getPose().getTranslation();
                Translation2d hubPos = AutoAimConstants.getHubPosition(swerve.isAllianceRed());
                double distance = hubPos.minus(robotPos).getNorm();
                return ShooterSubsystem.interpolateRps(distance);
        }

        // ── 工廠方法：自適應自主射擊 ──
        // PathPlanner Auto 用：即時計算距離 → 動態調整 RPS → 達速後送球
        private Command createAutoShootCommand() {
                return Commands.parallel(
                                // 持續依距離設定射手速度
                                shooterSubsystem.run(() -> shooterSubsystem.setTargetVelocity(getAdaptiveRps()))
                                                .finallyDo(() -> shooterSubsystem.stopShooter()),
                                Commands.sequence(
                                                Commands.waitUntil(() -> shooterSubsystem.isAtSpeed(getAdaptiveRps(),
                                                                AutoAimConstants.kShooterToleranceRps))
                                                                .withTimeout(2.0),
                                                transport.sys_runTransport().withTimeout(4.0)))
                                .withTimeout(4.0);
        }

        private Command createAutoIntakeCommand() {
                return Commands.parallel(
                                intakeRoller.sys_intakeWithTrigger(),
                                transport.sys_slowRunTransport()).withTimeout(6.0);
        }

        private Command createShootCommand() {
                return Commands.sequence(
                                Commands.waitUntil(
                                                () -> shooterSubsystem.isAtSpeed(getAdaptiveRps(),
                                                                AutoAimConstants.kShooterToleranceRps)),
                                transport.sys_runTransport());
        }

        public RobotContainer() {
                SignalLogger.enableAutoLogging(false);

                // ═══════════════ Shuffleboard 初始化 ═══════════════
                swerve.setupShuffleboardTab(shuffleboardManager.getSwerveTab());

                // ═══════════════ PathPlanner NamedCommands ═══════════════

                // "Start Intake"：吸球 + up_to_shoot 反轉擋球（防止球衝入射手）
                // → parallel 使用：邊走邊吸球
                // → sequential 使用：停下吸球
                // NamedCommands.registerCommand("Start Intake",
                // intakeRoller.sys_intakeWithTrigger());
                NamedCommands.registerCommand("Start Intake",
                                Commands.parallel(
                                                // 1. 執行 Intake 吸球
                                                intakeRoller.sys_intakeWithTrigger(),

                                                // 2. 同時執行 Transport 反轉 (把球往外推 / 擋球)
                                                transport.sys_reverseTransport()));

                // "Stop Intake"：立即停止吸球與 up_to_shoot
                NamedCommands.registerCommand("Stop Intake",
                                intakeRoller.runOnce(() -> {
                                        intakeRoller.stop();
                                        transport.stopTransport();
                                }));

                // "UpToShoot 2s"：up_to_shoot + transport 正轉推球 2 秒後自動結束
                // 射手應已在比賽開始時啟動到待機轉速（sys_idle DefaultCommand）
                // 建議放在路徑結束後（sequential），讓機器人停下來射擊
                // ⚠ 如需不同秒數，複製並修改秒數後再加一行 registerCommand
                NamedCommands.registerCommand("UpToShoot 4.5s",
                                Commands.sequence(
                                                // 1. 先反轉 transport 跟 roller 1 秒 (把球微退)
                                                // (因為 sys_reverseTransport 裡面有寫 this::stop，所以結束時會自動停)
                                                transport.sys_reverseTransport().withTimeout(1.0),

                                                // 2. 接著正常正轉運作 4.5 秒 (推球進去射擊)
                                                // ⚠️ 改用你寫好的 sys_runTransport()，它裡面是用 runEnd 寫的，時間到一定會自動觸發 stop()！
                                                transport.sys_runTransport().withTimeout(4.5)

                                ).withTimeout(5.5));
                                
                NamedCommands.registerCommand("UpToShoot 2s",
                                transport.sys_upToShootForSeconds(4.0));

                try {
                        RobotConfig config = RobotConfig.fromGUISettings();

                        com.pathplanner.lib.util.PathPlannerLogging.setLogActivePathCallback(null);
                        com.pathplanner.lib.util.PathPlannerLogging.setLogTargetPoseCallback(null);
                        AutoBuilder.configure(
                                        swerve::getPose,
                                        swerve::resetPose,
                                        swerve::getChassisSpeeds,
                                        swerve::drive,

                                        new PPHolonomicDriveController(
                                                        new PIDConstants(3.5, 0.0, 0.0), // Translation PID
                                                        new PIDConstants(5.5, 0.0, 0.0) // Rotation PID
                                        ),

                                        config, // 機器人配置

                                        swerve::isAllianceRed, // 決定是否翻轉路徑
                                        swerve // Subsystem
                        );
                        // com.pathplanner.lib.util.PathPlannerLogging.setLogEstimatedPoseCallback(null);

                } catch (Exception e) {
                        e.printStackTrace();
                }

                autoChooser = AutoBuilder.buildAutoChooser();
                shuffleboardManager.setupMainTab(swerve.getField2d(), autoChooser);

                configureBindings();
                swerve.setDefaultCommand(manualDriveCommand);
                // shooterSubsystem.setDefaultCommand((shooterSubsystem.sys_idle()));
                driverController.button(8).onTrue(Commands.runOnce(swerve::resetIMU)); // menu button

                driverController.rightStick().onTrue(Commands.either(
                                Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(false)),
                                Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(true)),
                                manualDriveCommand::getIsFieldOriented));

        }

        private void configureBindings() {

                // 自動瞄準射擊：按住 rightTrigger 時自動旋轉面向目標 + 依距離調整射手速度 + 達速對準後自動發射
                // ⚠️ 與 Drive2Tag (A鍵) 互斥：
                // - Drive2Tag addRequirements(swerve) → 會中斷 ManualDrive
                // - AutoAimAndShoot 不佔 swerve（透過 setAimSpeed 疊加）
                // - 若同時按 A + LB，兩者會同時控制底盤打架
                // → 解法：Drive2Tag 綁定時額外 require shooter+transport，讓 scheduler 自動互斥

                // 手動射擊：按住左緩衝鍵 (Left Bumper) 時直接設定射手速度為 45 RPS，達速後啟動 Transport 送球；放開時強制停止射手
                driverController.leftBumper().whileTrue(
                                Commands.parallel(
                                                // 1. 讓 Shooter 馬達直接設定為 45 RPS (使用你寫好的 setTargetVelocity 方法)
                                                Commands.run(() -> shooterSubsystem.setTargetVelocity(60.0),
                                                                shooterSubsystem),

                                                // 2. 監控轉速，達速後啟動 Transport 馬達送球
                                                Commands.sequence(
                                                                // 優雅地使用你的 isAtSpeed 方法：等待轉速達到 45 (容許誤差 2.0 RPS)
                                                                Commands.waitUntil(() -> shooterSubsystem
                                                                                .isAtSpeed(60.0, 2.0)),
                                                                // 轉速到了，直接呼叫你寫好的 Transport Command 送球！
                                                                transport.sys_runTransport()))
                                                .finallyDo(() -> {
                                                        // 3. 安全防呆：只要放開左緩衝鍵，強制停止射手
                                                        shooterSubsystem.stopShooter();
                                                        // (註：transport.sys_runTransport() 放開時會自己停，所以這裡不用多寫)
                                                }));

                // Drive2Tag：按住 A 鍵自動對位 AprilTag
                // 額外 require shooter + transport → 若 AutoAimAndShoot 正在運行會被自動取消
                driverController.a().whileTrue(
                                transport.sys_reverseTransport());

                // AutoAimAndShoot：按住右板機自動瞄準 + 依距離調整射手速度 + 達速對準後自動發射

                // driverController.rightTrigger(0.1).whileTrue(
                // new AutoAimAndShoot(swerve, shooterSubsystem, transport, manualDriveCommand,
                // shuffleboardManager.getAutoAimTab()
                // )
                // );

                driverController.rightTrigger(0.1).whileTrue(
                                new AutoAimAndShoot(
                                                swerve,
                                                shooterSubsystem,
                                                transport,
                                                manualDriveCommand,
                                                shuffleboardManager.getAutoAimTab())
                                                .deadlineWith(
                                                                Commands.sequence(
                                                                                // 持續給 1.0 的速度，維持 0.4 秒
                                                                                intakeArm.run(() -> intakeArm
                                                                                                .setManualSpeed(-0.5))
                                                                                                .withTimeout(0.2),

                                                                                // 持續給 0.0 的速度，維持 0.4 秒
                                                                                intakeArm.run(() -> intakeArm
                                                                                                .setManualSpeed(0.0))
                                                                                                .withTimeout(0.5))
                                                                                .repeatedly() // 不斷循環
                                                )
                                                .finallyDo(() -> intakeArm.setManualSpeed(0)));

                // shooterSubsystem.sys_manualShoot(1.0);

                // ==========================================
                // 1. 手動測試模式 (Manual Mode)
                // ==========================================
                // 設定：使用 "左搖桿 Y 軸" 來控制 Intake 上下
                // 當你在測試的時候，一直推搖桿，看 Dashboard 的數值
                // intakeArm.setDefaultCommand(
                // intakeArm.sys_manualMove(() -> -operatorController.getLeftY()) // 注意 Y
                // 軸通常要加負號才會符合直覺 (上推=正)
                // );
                // ==========================================
                // 2. 自動按鈕 (Automation)
                // ==========================================
                // 假設你測試出來，Intake 放下的最佳位置是 0.25 圈 (90度)
                // 按下 A 鍵，Intake 自動跑 到 0.25 圈的位置
                // driverController.a().onTrue(
                // intakeArm.runOnce(() -> intakeArm.setTargetPosition(0.25))
                // );

                // // 按下 B 鍵，Intake 自動收回到 0 圈 (原點)
                driverController.b().whileTrue(
                                Commands.parallel(
                                                intakeRoller.sys_outtake(),
                                                transport.sys_reverseTransport()));

                // ==========================================
                // 設定：按住 "左板機 (Left Trigger)" 來控制 Intake 吸入
                // ==========================================

                // 當左板機按壓超過 0.1 時，啟動 sys_intakeWithTrigger 指令
                // 放開後自動停止
                driverController.leftTrigger(0.1).whileTrue(
                                intakeRoller.sys_intakeWithTrigger()
                // intakeRoller.sys_intakeWithTrigger(() ->
                // driverController.getLeftTriggerAxis())
                );

                // transport
                driverController.x().whileTrue(
                                transport.sys_runTransport());
                intakeArm.setDefaultCommand(
                                intakeArm.sys_manualMove(() -> -driverController.getRightY()));

                shooterSubsystem.setDefaultCommand(
                                shooterSubsystem.sys_manualShoot(55.0));

                // transport.setDefaultCommand(transport.sys_reverseroller());
        }

        public Command getAutonomousCommand() {
                // swerve.run();

                return autoChooser.getSelected();
        }

        public ShuffleboardManager getShuffleboardManager() {
                return shuffleboardManager;
        }

        public void teleopInit() {
                // 自動使用 Limelight 校正位姿（包含航向），免去手動按 resetIMU
                swerve.resetPoseToLimelight();

                swerve.run();
                // 進入 Teleop 時震動手把提示（必須 schedule 才會執行）
                CommandScheduler.getInstance().schedule(
                                Commands.sequence(
                                                Commands.runOnce(() -> driverController
                                                                .setRumble(RumbleType.kBothRumble, 1)),
                                                Commands.waitSeconds(0.3),
                                                Commands.runOnce(() -> driverController
                                                                .setRumble(RumbleType.kBothRumble, 0)),
                                                Commands.waitSeconds(0.1),
                                                Commands.runOnce(() -> driverController
                                                                .setRumble(RumbleType.kBothRumble, 1)),
                                                Commands.waitSeconds(0.3),
                                                Commands.runOnce(() -> driverController
                                                                .setRumble(RumbleType.kBothRumble, 0)))
                                                .finallyDo(() -> driverController.setRumble(RumbleType.kBothRumble,
                                                                0)));
        }

        public void disabledInit() {
                swerve.disabledInit();
        }
}
