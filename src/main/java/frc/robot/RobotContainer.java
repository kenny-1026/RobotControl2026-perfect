// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.Constants.AutoAimConstants;
import frc.robot.commands.Drive2Tag;
import frc.robot.commands.ManualDrive;
// import frc.robot.commands.AutoAimAndShoot;
// import frc.robot.subsystems.IntakeArmSubsystem;
// import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.TransportSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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

// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShuffleboardManager;

import java.util.logging.Logger;

import com.ctre.phoenix6.SignalLogger;

public class RobotContainer {

    // SignalLogger logger = new SignalLogger();
    // private final Timer timer = new Timer();
    private int printCounter = 0;
    private final Swerve swerve = new Swerve(Constants.kLimelightName);
    private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.kSwerveControllerPort);

    private final ManualDrive manualDriveCommand = new ManualDrive(swerve, driverController);
    private final SendableChooser<Command> autoChooser;

    // ═══════════════ Shuffleboard ═══════════════
    private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

    // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shuffleboardManager.getShooterTab());
    // private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem(shuffleboardManager.getIntakeArmTab());
    // private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem(shuffleboardManager.getIntakeRollerTab());
    // private final TransportSubsystem transport = new TransportSubsystem();

    private Command autoCommand;

    // ── 距離自適應輔助方法 ──
    // 根據機器人目前位置計算到 Hub 的距離，查表取得目標 RPS
    /*
    private double getAdaptiveRps() {
        var robotPos = swerve.getPose().getTranslation();
        Translation2d hubPos = AutoAimConstants.getHubPosition(swerve.isAllianceRed());
        double distance = hubPos.minus(robotPos).getNorm();
        return ShooterSubsystem.interpolateRps(distance);
    }
    */

    // ── 工廠方法：自適應自主射擊 ──
    /*
    private Command createAutoShootCommand() {
        return Commands.parallel(
            shooterSubsystem.run(() -> shooterSubsystem.setTargetVelocity(getAdaptiveRps()))
                .finallyDo(() -> shooterSubsystem.stopShooter()),
            Commands.sequence(
                Commands.waitUntil(() -> shooterSubsystem.isAtSpeed(getAdaptiveRps(), AutoAimConstants.kShooterToleranceRps)).withTimeout(2.0),
                transport.sys_runTransport().withTimeout(4.0)
            )
        ).withTimeout(4.0);
    }

    private Command createAutoIntakeCommand() {
        return Commands.parallel(
            intakeRoller.sys_intakeWithTrigger(),
            transport.sys_slowRunTransport()
        ).withTimeout(2.5);
    }

    private Command createShootCommand() {
        return Commands.sequence(
            Commands.waitUntil(() -> shooterSubsystem.isAtSpeed(getAdaptiveRps(), AutoAimConstants.kShooterToleranceRps)),
            transport.sys_runTransport()
        );
    }
    */

    public RobotContainer() {
        SignalLogger.enableAutoLogging(false);

        // ═══════════════ Shuffleboard 初始化 ═══════════════
        swerve.setupShuffleboardTab(shuffleboardManager.getSwerveTab());
        

        /*
        NamedCommands.registerCommand("transport wait shoot", createShootCommand());
        NamedCommands.registerCommand("shoot work",
            shooterSubsystem.run(() -> shooterSubsystem.setTargetVelocity(getAdaptiveRps()))
                .finallyDo(() -> shooterSubsystem.stopShooter()));

        NamedCommands.registerCommand("Auto Shoot", createAutoShootCommand());
        NamedCommands.registerCommand("Far Auto Shoot", createAutoShootCommand());
        NamedCommands.registerCommand("Auto Intake", createAutoIntakeCommand());

        NamedCommands.registerCommand("Start Intake",
            intakeRoller.sys_intakeWithTrigger()
        );

        NamedCommands.registerCommand("Stop Intake",
            intakeRoller.runOnce(() -> intakeRoller.stop())
            );
        */


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
                    new PIDConstants(1.8, 0.0, 0.0)  // Rotation PID
                ),
                
                config, // 機器人配置
                
                swerve::isAllianceRed, // 決定是否翻轉路徑
                swerve // Subsystem
            );

        } catch (Exception e) {
            e.printStackTrace();
        }

        autoChooser = AutoBuilder.buildAutoChooser();
        shuffleboardManager.setupMainTab(swerve.getField2d(), autoChooser);
        
        configureBindings();
        swerve.setDefaultCommand(manualDriveCommand);
        driverController.button(8).onTrue(Commands.runOnce(swerve::resetIMU)); // menu button


        driverController.rightStick().onTrue(Commands.either(
            Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(false)),
            Commands.runOnce(() -> manualDriveCommand.setIsFieldOriented(true)),
            manualDriveCommand::getIsFieldOriented));
    }

    public void putLimeLight() {
        // LimelightHelpers.PoseEstimate mt2;
    }



    private void configureBindings() {
        /*
        driverController.leftBumper().whileTrue(
            Commands.parallel(
                Commands.run(() -> shooterSubsystem.setTargetVelocity(45.0), shooterSubsystem),

                Commands.sequence(
                    Commands.waitUntil(() -> shooterSubsystem.isAtSpeed(45.0, 2.0)),
                    transport.sys_runTransport()
                )
            ).finallyDo(() -> {
                shooterSubsystem.stopShooter();
            })
        );
        */

        driverController.a().whileTrue(
            new Drive2Tag(swerve, Constants.kLimelightName, -1.45, 0.0, 0.0)
            /*
                .alongWith(
                    Commands.runOnce(() -> {
                        shooterSubsystem.stopShooter();
                        transport.stopTransport();
                    }, shooterSubsystem, transport)
                )
            */
        );

       /*
       driverController.rightTrigger(0.1).whileTrue(
           new AutoAimAndShoot(swerve, shooterSubsystem, transport, manualDriveCommand, shuffleboardManager.getAutoAimTab()
            )
        );
       */

        /*
        driverController.b().whileTrue(
            Commands.parallel(
            intakeRoller.sys_outtake(),
            transport.sys_reverseTransport()
            )
        );

        driverController.leftTrigger(0.1).whileTrue(
            intakeRoller.sys_intakeWithTrigger()
        );

        driverController.x().whileTrue(
            transport.sys_runTransport()
        );
        */
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public ShuffleboardManager getShuffleboardManager() {
        return shuffleboardManager;
    }
    
    public void teleopInit() {
        swerve.resetPoseToLimelight();

        swerve.run();
        CommandScheduler.getInstance().schedule(
            Commands.sequence(
                Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 1)),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 0)),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 1)),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 0))
            ).finallyDo(() -> driverController.setRumble(RumbleType.kBothRumble, 0))
        );
    }

    public void disabledInit() {
        swerve.disabledInit();
    }
}
