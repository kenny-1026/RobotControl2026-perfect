// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.util.ShuffleboardManager;

import com.ctre.phoenix6.SignalLogger;

public class RobotContainer {

    private final Swerve swerve = new Swerve();
    private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.kSwerveControllerPort);

    private final ManualDrive manualDriveCommand = new ManualDrive(swerve, driverController);
    private final SendableChooser<Command> autoChooser;

    // ═══════════════ Shuffleboard ═══════════════
    private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

    public RobotContainer() {
        SignalLogger.enableAutoLogging(false);

        // ═══════════════ Shuffleboard 初始化 ═══════════════
        swerve.setupShuffleboardTab(shuffleboardManager.getSwerveTab());
        
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

    private void configureBindings() {
        // 這裡可以根據需要添加底盤相關的按鍵綁定
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public ShuffleboardManager getShuffleboardManager() {
        return shuffleboardManager;
    }
    
    public void teleopInit() {
        swerve.run();
    }

    public void disabledInit() {
        swerve.disabledInit();
    }
}
