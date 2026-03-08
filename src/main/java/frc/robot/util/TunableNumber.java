package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 可在 Shuffleboard / Glass 即時修改的數值。
 * 
 * 支援兩種模式：
 * 1. Shuffleboard Tab 模式：new TunableNumber(tab, "kP", 0.12)
 *    → 數值會出現在指定的 Shuffleboard 分頁，方便分類整理
 * 2. SmartDashboard 模式：new TunableNumber("Shooter/kP", 0.12)
 *    → 向下相容，數值會出現在 SmartDashboard 表
 * 
 * Shuffleboard 操作方式：
 *   1. 啟動 Shuffleboard（WPILib → Start Tool → Shuffleboard）
 *   2. 切換到對應的 Tab（例如 "Shooter PID"）
 *   3. 直接修改數值，程式會在下一個週期偵測到變更並自動套用到馬達
 */
public class TunableNumber {
    private final String key;
    private double lastValue;
    private final GenericEntry entry;

    /**
     * Shuffleboard Tab 模式建構子。
     * @param tab          Shuffleboard 分頁
     * @param key          參數名稱（顯示在 Widget 標題）
     * @param defaultValue 預設值
     */
    public TunableNumber(ShuffleboardTab tab, String key, double defaultValue) {
        this.key = key;
        this.lastValue = defaultValue;
        this.entry = tab.add(key, defaultValue)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
    }

    /**
     * SmartDashboard 模式建構子（向下相容）。
     * @param key          完整路徑名（例如 "Shooter/kP"）
     * @param defaultValue 預設值
     */
    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.lastValue = defaultValue;
        this.entry = null;
        SmartDashboard.putNumber(key, defaultValue);
    }

    /**
     * 取得目前在 NetworkTables 上的數值。
     */
    public double get() {
        if (entry != null) {
            return entry.getDouble(lastValue);
        }
        return SmartDashboard.getNumber(key, lastValue);
    }

    /**
     * 檢查數值是否被使用者修改過（與上次呼叫 hasChanged() 時相比）。
     * 如果有變更，內部會更新 lastValue。
     * @return true 如果數值有變更
     */
    public boolean hasChanged() {
        double current = get();
        if (current != lastValue) {
            lastValue = current;
            return true;
        }
        return false;
    }

    /**
     * 取得 NetworkTables key 名稱。
     */
    public String getKey() {
        return key;
    }
}
