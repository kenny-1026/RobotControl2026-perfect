package frc.robot.util;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Shuffleboard 集中管理器。
 * 
 * 在 RobotContainer 中建立一次即可。
 * 負責建立所有分頁 (Tab) 並提供各子系統取用。
 * 
 * ┌─────────────────────────────────────────────────────┐
 * │ Tab 規劃:                                           │
 * │                                                     │
 * │ 🏠 Main         → Field2d, Auto Chooser, Loop Time  │
 * │ 🔫 Shooter PID  → kV/kP/kI/kD + 示波圖              │
 * │ 🦾 IntakeArm PID→ kP/kI/kD/kG + 示波圖              │
 * │ 🔄 IntakeRoller  → kV/kP/kI/kD + 示波圖             │
 * │ 🚗 Swerve       → Rotor kP/kI/kD + 底盤速度         │
 * │ 🎯 AutoAim      → Rotation kP/kI/kD + 追蹤數據      │
 * └─────────────────────────────────────────────────────┘
 */
public class ShuffleboardManager {

    // ═══════════════ Tabs ═══════════════
    private final ShuffleboardTab mainTab;
    private final ShuffleboardTab shooterTab;
    private final ShuffleboardTab intakeArmTab;
    private final ShuffleboardTab intakeRollerTab;
    private final ShuffleboardTab swerveTab;
    private final ShuffleboardTab autoAimTab;

    // ═══════════════ Main Tab Entries ═══════════════
    private GenericEntry loopTimeEntry;
    private GenericEntry loopMaxEntry;
    private GenericEntry loopOverrunEntry;

    public ShuffleboardManager() {
        mainTab        = Shuffleboard.getTab("Main");
        shooterTab     = Shuffleboard.getTab("Shooter PID");
        intakeArmTab   = Shuffleboard.getTab("IntakeArm PID");
        intakeRollerTab= Shuffleboard.getTab("IntakeRoller PID");
        swerveTab      = Shuffleboard.getTab("Swerve");
        autoAimTab     = Shuffleboard.getTab("AutoAim");
    }

    // ─── Getters for subsystems to use ───
    public ShuffleboardTab getMainTab()          { return mainTab; }
    public ShuffleboardTab getShooterTab()       { return shooterTab; }
    public ShuffleboardTab getIntakeArmTab()     { return intakeArmTab; }
    public ShuffleboardTab getIntakeRollerTab()  { return intakeRollerTab; }
    public ShuffleboardTab getSwerveTab()        { return swerveTab; }
    public ShuffleboardTab getAutoAimTab()       { return autoAimTab; }

    /**
     * 初始化 Main Tab 上的共用 Widget。
     * @param field2d   場地圖（來自 Swerve）
     * @param autoChooser 自動模式選擇器
     */
    public void setupMainTab(Field2d field2d, SendableChooser<Command> autoChooser) {
        // 場地圖 — 只透過 Shuffleboard 發佈（不可同時用 SmartDashboard.putData，
        // 否則 Sendable 雙重綁定會導致其中一個路徑不更新）
        // Glass/SimGUI 中請打開 /Shuffleboard/Main/Field 來查看
        mainTab.add("Field", field2d)
               .withWidget(BuiltInWidgets.kField)
               .withSize(6, 4)
               .withPosition(0, 0);

        // Auto 選擇器
        mainTab.add("Auto Mode", autoChooser)
               .withWidget(BuiltInWidgets.kComboBoxChooser)
               .withSize(3, 1)
               .withPosition(6, 0);

        // Loop Time
        loopTimeEntry = mainTab.add("Loop ms", 0)
               .withWidget(BuiltInWidgets.kGraph)
               .withSize(3, 2)
               .withPosition(6, 1)
               .getEntry();

        loopMaxEntry = mainTab.add("Loop Max ms", 0)
               .withWidget(BuiltInWidgets.kTextView)
               .withSize(1, 1)
               .withPosition(9, 0)
               .getEntry();

        loopOverrunEntry = mainTab.add("Overrun?", false)
               .withWidget(BuiltInWidgets.kBooleanBox)
               .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "green"))
               .withSize(1, 1)
               .withPosition(9, 1)
               .getEntry();
    }

    /**
     * 更新 Loop 計時數據（在 Robot.robotPeriodic() 呼叫）。
     */
    public void updateLoopTime(double loopMs, double maxMs, boolean overrun) {
        if (loopTimeEntry != null) {
            loopTimeEntry.setDouble(loopMs);
            loopMaxEntry.setDouble(maxMs);
            loopOverrunEntry.setBoolean(overrun);
        }
    }
}
