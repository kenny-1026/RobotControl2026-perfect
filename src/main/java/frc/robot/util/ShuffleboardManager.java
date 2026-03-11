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
 */
public class ShuffleboardManager {

    // ═══════════════ Tabs ═══════════════
    private final ShuffleboardTab mainTab;
    private final ShuffleboardTab swerveTab;

    // ═══════════════ Main Tab Entries ═══════════════
    private GenericEntry loopTimeEntry;
    private GenericEntry loopMaxEntry;
    private GenericEntry loopOverrunEntry;

    public ShuffleboardManager() {
        mainTab        = Shuffleboard.getTab("Main");
        swerveTab      = Shuffleboard.getTab("Swerve");
    }

    // ─── Getters for subsystems to use ───
    public ShuffleboardTab getMainTab()          { return mainTab; }
    public ShuffleboardTab getSwerveTab()        { return swerveTab; }

    /**
     * 初始化 Main Tab 上的共用 Widget。
     * @param field2d   場地圖（來自 Swerve）
     * @param autoChooser 自動模式選擇器
     */
    public void setupMainTab(Field2d field2d, SendableChooser<Command> autoChooser) {
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
