package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ManualDriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.SwerveMode;

public class ManualDrive extends Command {
    
    private final Swerve mSwerve;
    private final CommandXboxController mJoystick;
    private boolean isFieldOriented = true;

    private int telemetryCounter = 0;
    
    public ManualDrive(Swerve drive, CommandXboxController joystick) {
        mSwerve = drive;
        mJoystick = joystick;

        addRequirements(mSwerve);
    }

    @Override
    public void initialize() {
        mSwerve.setSwerveMode(SwerveMode.ROBOT_CENTRIC);
    }

    @Override
    public void execute() {
        double xCtl = -mJoystick.getLeftY();
        double yCtl = -mJoystick.getLeftX();
        double zCtl = mJoystick.getRightX();

        boolean telemetryThisCycle = (++telemetryCounter >= Constants.kTelemetryDivider);
        if (telemetryThisCycle) {
            telemetryCounter = 0;
            SmartDashboard.putNumber("Raw/LeftY", mJoystick.getLeftY());
            SmartDashboard.putNumber("Raw/LeftX", mJoystick.getLeftX());
        }
        
        double boostTranslation = mJoystick.rightBumper().getAsBoolean()?1:0.5;
        
        xCtl = calculateNullZone(xCtl, ManualDriveConstants.kXDeadzone);
        xCtl *= ManualDriveConstants.kXMultiplier;
        xCtl *= boostTranslation;
        yCtl = calculateNullZone(yCtl, ManualDriveConstants.kYDeadzone);
        yCtl *= ManualDriveConstants.kYMultiplier;
        yCtl *= boostTranslation;
        zCtl = calculateNullZone(zCtl, ManualDriveConstants.kZDeadzone);
        zCtl *= ManualDriveConstants.kZMultiplier;

        xCtl *= SwerveConstants.kMaxPhysicalSpeedMps;
        yCtl *= SwerveConstants.kMaxPhysicalSpeedMps;
        zCtl *= SwerveConstants.kMaxPhysicalSpeedMps;

        mSwerve.setSpeed(xCtl, yCtl, zCtl, isFieldOriented);
        if (telemetryThisCycle) {
            SmartDashboard.putNumber("Drive/xSpeed", xCtl);
            SmartDashboard.putNumber("Drive/ySpeed", yCtl);
            SmartDashboard.putNumber("Drive/zSpeed", zCtl);
            SmartDashboard.putBoolean("Drive/fieldOriented", isFieldOriented);
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Field Oriented", this::getIsFieldOriented, this::setIsFieldOriented);
    }
}   
