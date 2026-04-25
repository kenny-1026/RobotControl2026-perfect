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
import frc.robot.subsystems.LightPollution;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.StorageSubsystem;
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
import edu.wpi.first.wpilibj.util.Color;
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
import frc.robot.subsystems.StorageSubsystem;
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

        // private final ShooterSubsystem shooterSubsystem = new
        // ShooterSubsystem(shuffleboardManager.getShooterTab());
        // private final IntakeArmSubsystem intakeArm = new
        // IntakeArmSubsystem(shuffleboardManager.getIntakeArmTab());
        private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem(
                        shuffleboardManager.getIntakeRollerTab());
        private final TransportSubsystem transport = new TransportSubsystem();
        private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();
        private final StorageSubsystem storage = new StorageSubsystem();
        private final LightPollution lightPollution = new LightPollution(9, 126);// 0: PWM號碼, 60: LED count
        private Command autoCommand;

        // ── 距離自適應輔助方法 ──
        // 根據機器人目前位置計算到 Hub 的距離，查表取得目標 RPS
        // private double getAdaptiveRps() {
        // var robotPos = swerve.getPose().getTranslation();
        // Translation2d hubPos =
        // AutoAimConstants.getHubPosition(swerve.isAllianceRed());
        // double distance = hubPos.minus(robotPos).getNorm();
        // return ShooterSubsystem.interpolateRps(distance);
        // }

        private Command createAutoIntakeCommand() {
                return Commands.parallel(
                                intakeRoller.sys_intakeWithTrigger(),
                                transport.sys_slowRunTransport()).withTimeout(3.0);
        }

        public RobotContainer() {
                SignalLogger.enableAutoLogging(false);

                // ═══════════════ Shuffleboard 初始化 ═══════════════
                swerve.setupShuffleboardTab(shuffleboardManager.getSwerveTab());
                NamedCommands.registerCommand("Auto Intake", createAutoIntakeCommand());

                NamedCommands.registerCommand("Start Intake",
                                intakeRoller.sys_intakeWithTrigger());

                NamedCommands.registerCommand("Stop Intake",
                                intakeRoller.runOnce(() -> intakeRoller.stop()));

                NamedCommands.registerCommand("AutoDropAndIntake",
                                // 放下手臂 0.4 秒
                                intakeArm.run(() -> intakeArm.setManualSpeed(0.35))
                                                .withTimeout(0.8)
                                                .finallyDo(() -> intakeArm.setManualSpeed(0.0)));

                // "UpToShoot 2s"：up_to_shoot + transport 正轉推球 2 秒後自動結束
                // 射手應已在比賽開始時啟動到待機轉速（sys_idle DefaultCommand）
                // 建議放在路徑結束後（sequential），讓機器人停下來射擊
                // ⚠ 如需不同秒數，複製並修改秒數後再加一行 registerCommand
                // NamedCommands.registerCommand("UpToShoot 4.5s",
                // transport.sys_runTransport().withTimeout(4.5));

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
                                                        new PIDConstants(3.0, 0.0, 0.0), // Translation PID
                                                        new PIDConstants(2.2, 0.0, 0.0) // Rotation PID
                                        ),

                                        config, // 機器人配置

                                        swerve::isAllianceRed, // 決定是否翻轉路徑
                                        swerve // Subsystem
                        );

                } catch (Exception e) {
                        e.printStackTrace();
                }

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                shuffleboardManager.setupMainTab(swerve.getField2d(), autoChooser);

                swerve.setDefaultCommand(manualDriveCommand);
                // lightPollution.setDefaultCommand();
                lightPollution.setModeRollingRainbow();
                driverController.button(8).onTrue(Commands.runOnce(swerve::resetIMU)); // menu button

                driverController.rightStick().onTrue(Commands.either(
                                Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(false)),
                                Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(true)),
                                manualDriveCommand::getIsFieldOriented));
        }

        private void configureBindings() {
                // 1. 右邊模式 (Right Bumper)
                // 參數: swerve, limelightName, TargetX(-0.8), TargetY(-0.5 往右), TargetYaw(-15)
                // driverController.rightBumper().whileTrue(
                // new Drive2Tag(swerve, Constants.kLimelightName, -1.15, 0.8, 15.0)
                // );

                // 2. 左邊模式 (Left Bumper)
                // 參數: swerve, limelightName, TargetX(-0.8), TargetY(0.5 往左), TargetYaw(-15)
                // driverController.leftBumper().whileTrue(
                // new Drive2Tag(swerve, Constants.kLimelightName, -1.15, -0.8, -15.0)
                // );

                // 自動瞄準射擊：按住 rightTrigger 時自動旋轉面向目標 + 依距離調整射手速度 + 達速對準後自動發射
                // ⚠️ 與 Drive2Tag (A鍵) 互斥：
                // - Drive2Tag addRequirements(swerve) → 會中斷 ManualDrive
                // - AutoAimAndShoot 不佔 swerve（透過 setAimSpeed 疊加）
                // - 若同時按 A + LB，兩者會同時控制底盤打架
                // → 解法：Drive2Tag 綁定時額外 require shooter+transport，讓 scheduler 自動互斥

                // 手動射擊：按住左緩衝鍵 (Left Bumper) 時直接設定射手速度為 45 RPS，達速後啟動 Transport 送球；放開時強制停止射手
                // driverController.leftBumper().whileTrue(
                // Commands.parallel(
                // // 1. 讓 Shooter 馬達直接設定為 45 RPS (使用你寫好的 setTargetVelocity 方法)
                // Commands.run(() -> shooterSubsystem.setTargetVelocity(60.0),
                // shooterSubsystem),

                // // 2. 監控轉速，達速後啟動 Transport 馬達送球
                // Commands.sequence(
                // // 優雅地使用你的 isAtSpeed 方法：等待轉速達到 45 (容許誤差 2.0 RPS)
                // Commands.waitUntil(() -> shooterSubsystem
                // .isAtSpeed(60.0, 2.0)),
                // // 轉速到了，直接呼叫你寫好的 Transport Command 送球！
                // transport.sys_runTransport()))
                // .finallyDo(() -> {
                // // 3. 安全防呆：只要放開左緩衝鍵，強制停止射手
                // shooterSubsystem.stopShooter();
                // // (註：transport.sys_runTransport() 放開時會自己停，所以這裡不用多寫)
                // }));

                // Drive2Tag：按住 A 鍵自動對位 AprilTag
                // 額外 require shooter + transport → 若 AutoAimAndShoot 正在運行會被自動取消
                // driverController.a().whileTrue(
                // transport.sys_reverseTransport());

                // AutoAimAndShoot：按住右板機自動瞄準 + 依距離調整射手速度 + 達速對準後自動發射

                // driverController.rightTrigger(0.1).whileTrue(
                // new AutoAimAndShoot(swerve, shooterSubsystem, transport, manualDriveCommand,
                // shuffleboardManager.getAutoAimTab()
                // )
                // );

                // Gate：按住Y鍵控制閘門開關
                // driverController.a().whileTrue(storage.sys_reverseStorage());
                // driverController.y().whileTrue(storage.sys_runStorage());

                // -------------- 原有按鍵 --------------

                // driverController.y().onTrue(storage.sys_togglePosition());

                // driverController.rightTrigger(0.1).whileTrue(
                //                 Commands.sequence(
                //                                 // 動作一：給 -0.3 的速度，維持 2.5 秒
                //                                 intakeArm.run(() -> intakeArm.setManualSpeed(-0.3))
                //                                                 .withTimeout(2.5),

                //                                 // 動作二：給 0.1 的速度，維持 1.5 秒
                //                                 intakeArm.run(() -> intakeArm.setManualSpeed(0.1))
                //                                                 .withTimeout(1.5))
                //                                 .repeatedly() // 只要按住右板機，就會不斷重複上述兩個動作
                //                                 .finallyDo(() -> intakeArm.setManualSpeed(0)) // 鬆開板機時，安全停止手臂
                // );

                // // 按下 B 鍵，Intake 吸球並啟動輸送帶
                // driverController.b().whileTrue(
                // Commands.parallel(
                // intakeRoller.sys_outtake(),
                // transport.sys_reverseTransport()));

                // // transport往閘門送球
                // driverController.x().whileTrue(
                // transport.sys_runTransport());
               
                // // 當左板機按壓超過 0.1 時，啟動 intake+transport 指令
                // // 放開後自動停止
                // driverController.leftTrigger(0.1).whileTrue(
                // Commands.parallel(
                // intakeRoller.sys_intakeWithTrigger(),
                // transport.sys_runTransport()));

                // -------------- 原有按鍵 --------------

                // -------------- 有加燈光特效的按鍵 --------------

                driverController.y()
                                .onTrue(Commands.parallel(
                                                storage.sys_togglePosition(), // 同時切換閘門
                                                Commands.runOnce(
                                                                () -> lightPollution.setModeSolidBlink(Color.kYellow),
                                                                lightPollution)))
                                .onFalse(Commands.runOnce(() -> lightPollution.setModeRollingRainbow(),
                                                lightPollution));

                driverController.x()
                                // 1. 保留原本功能：按住時 Transport 馬達持續轉動
                                .whileTrue(transport.sys_runTransport())
                                // 2. 新增燈光功能：按下的瞬間切換到紅色閃爍
                                .onTrue(Commands.runOnce(() -> lightPollution.setModeSolidBlink(Color.kFirstRed),
                                                lightPollution))
                                // 3. 恢復燈光功能：放開的瞬間切換回滾動彩虹
                                .onFalse(Commands.runOnce(() -> lightPollution.setModeRollingRainbow(),
                                                lightPollution));

                driverController.b()
                                // 1. 保留原本功能：按住時同時反轉吸球馬達與傳輸馬達
                                .whileTrue(Commands.parallel(
                                                intakeRoller.sys_outtake(),
                                                transport.sys_reverseTransport()))
                                // 2. 新增燈光功能：按下的瞬間切換到藍色閃爍
                                .onTrue(Commands.runOnce(() -> lightPollution.setModeSolidBlink(Color.kFirstBlue),
                                                lightPollution))
                                // 3. 恢復燈光功能：放開的瞬間切換回滾動彩虹
                                .onFalse(Commands.runOnce(() -> lightPollution.setModeRollingRainbow(),
                                                lightPollution));

                driverController.leftTrigger(0.1)
                                // 1. 保留原本功能：按住時啟動吸球與傳輸
                                .whileTrue(Commands.parallel(
                                                intakeRoller.sys_intakeWithTrigger(),
                                                transport.sys_runTransport()))
                                // 2. 新增燈光：按下的瞬間變成橘色閃爍
                                .onTrue(Commands.runOnce(() -> lightPollution.setModeSolidBlink(Color.kOrange),
                                                lightPollution))
                                // 3. 恢復燈光：放開的瞬間回到滾動彩虹
                                .onFalse(Commands.runOnce(() -> lightPollution.setModeRollingRainbow(),
                                                lightPollution));

                 driverController.rightTrigger(0.1).whileTrue(
                                Commands.sequence(
                                                // 動作一：給 -0.3 的速度，維持 2.5 秒
                                                intakeArm.run(() -> intakeArm.setManualSpeed(-0.3))
                                                                .withTimeout(2.5),

                                                // 動作二：給 0.1 的速度，維持 1.5 秒
                                                intakeArm.run(() -> intakeArm.setManualSpeed(0.1))
                                                                .withTimeout(1.5))
                                                .repeatedly() // 只要按住右板機，就會不斷重複上述兩個動作
                                                .finallyDo(() -> intakeArm.setManualSpeed(0)) // 鬆開板機時，安全停止手臂
                );

                 // 使用 "右搖桿 Y 軸" 來控制 Intake 上下                                
                intakeArm.setDefaultCommand(
                intakeArm.sys_manualMove(() -> -driverController.getRightY()));                                

                // -------------- 有加燈光特效的按鍵 --------------

                // driverController.rightTrigger(0.1).whileTrue(
                // new AutoAimAndShoot(
                // swerve,
                // shooterSubsystem,
                // transport,
                // manualDriveCommand,
                // shuffleboardManager.getAutoAimTab())
                // .deadlineWith(
                // Commands.sequence(
                // // 持續給 1.0 的速度，維持 0.4 秒
                // intakeArm.run(() -> intakeArm
                // .setManualSpeed(-0.3))
                // .withTimeout(2.5),

                // // 持續給 0.0 的速度，維持 0.4 秒
                // intakeArm.run(() -> intakeArm
                // .setManualSpeed(0.1))
                // .withTimeout(1.5))
                // .repeatedly() // 不斷循環
                // )
                // .finallyDo(() -> intakeArm.setManualSpeed(0)));

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

                // ==========================================
                // 設定：按住 "左板機 (Left Trigger)" 來控制 Intake 吸入
                // ==========================================

                // transport.setDefaultCommand(transport.sys_reverseroller());

        }

        public Command getAutonomousCommand() {
                Command selected = autoChooser.getSelected();
                if (selected != null) {
                        return selected;
                }
                return Commands.none();
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
