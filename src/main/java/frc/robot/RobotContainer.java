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
                                transport.sys_slowRunTransport()).withTimeout(3.0);
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

                NamedCommands.registerCommand("transport wait shoot", createShootCommand());
                // "shoot work"：僅啟動射手（依距離自適應 RPS），不含送球
                NamedCommands.registerCommand("shoot work",
                                shooterSubsystem.run(() -> shooterSubsystem.setTargetVelocity(getAdaptiveRps()))
                                                .finallyDo(() -> shooterSubsystem.stopShooter()));

                NamedCommands.registerCommand("Auto Shoot", createAutoShootCommand());
                // "Far Auto Shoot" 不再需要，統一用自適應 "Auto Shoot"
                NamedCommands.registerCommand("Far Auto Shoot", createAutoShootCommand());
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
                NamedCommands.registerCommand("UpToShoot 4.5s",
                                transport.sys_runTransport().withTimeout(4.5));

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
                                                        new PIDConstants(1.8, 0.0, 0.0) // Rotation PID
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

                // autoCommand = AutoBuilder.buildAuto("Simple Left Auto");

                configureBindings();
                swerve.setDefaultCommand(manualDriveCommand);
                // shooterSubsystem.setDefaultCommand((shooterSubsystem.sys_idle()));
                driverController.button(8).onTrue(Commands.runOnce(swerve::resetIMU)); // menu button

                driverController.rightStick().onTrue(Commands.either(
                                Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(false)),
                                Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(true)),
                                manualDriveCommand::getIsFieldOriented));

                // CommandScheduler.getInstance().schedule(
                // Commands.run(() -> {
                // // printCounter++;
                // // if (printCounter >= 10) { // 每 10 個週期 (約 0.2秒) 才執行一次
                // // // SmartDashboard.putString("Position", swerve.getPose().toString());
                // // // putLimeLight();
                // // // 這裡也可以呼叫 swerve.updateSmartDashboard();
                // // printCounter = 0; // 重置計數器
                // // }
                // // SmartDashboard.putString("Position", swerve.getPose().toString());
                // // putLimeLight();
                // // SmartDashboard.putNumber("Match Time", timer.getMatchTime());
                // // if (shooterSubsystem.isAtSpeed()) { // 假設目標是 80
                // // // 速度到了 -> 輕微震動左手把
                // // driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.1);
                // // } else {
                // // // 速度沒到 -> 關閉震動
                // // driverController.getHID().setRumble(RumbleType.kLeftRumble, 0);
                // // }
                // }).ignoringDisable(true)

                // );
        }

        public void putLimeLight() {
                LimelightHelpers.PoseEstimate mt2;
                // if (swerve.isAllianceRed()) {
                // // 如果是紅方，拿以紅方為基準的 MegaTag2 座標
                // mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.kLimelightName);
                // } else {
                // 預設拿藍方的
                // mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.kLimelightName);
                // // }
                // if (mt2 != null) {
                // // SmartDashboard.putNumber("TagCount", mt2.tagCount);
                // double[] fiducialIds = new double[mt2.rawFiducials.length];
                // for (int i = 0; i < mt2.rawFiducials.length; i++) {
                // fiducialIds[i] = mt2.rawFiducials[i].id;
                // }
                // // SmartDashboard.putNumberArray("Fiducials", fiducialIds);
                // }
        }

        private void configureBindings() {
                
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
                                                                                                .withTimeout(0.3),

                                                                                // 持續給 0.0 的速度，維持 0.4 秒
                                                                                intakeArm.run(() -> intakeArm
                                                                                                .setManualSpeed(0.4))
                                                                                                .withTimeout(0.4))
                                                                                .repeatedly() // 不斷循環
                                                )
                                                .finallyDo(() -> intakeArm.setManualSpeed(0)));

                intakeArm.setDefaultCommand(
                                intakeArm.sys_manualMove(() -> -driverController.getRightY()));

                shooterSubsystem.setDefaultCommand(
                                shooterSubsystem.sys_manualShoot(52.0)); // mid 52

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
