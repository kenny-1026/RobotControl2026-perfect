# Loop Overrun 修復紀錄

**修復日期**：2026-02-26  
**專案**：RobotControl2026  
**問題**：機器人執行動作時出現延遲 / WPILib Loop Overrun 警告  
**最後更新**：2026-03-08

---

## 背景說明

FRC 機器人使用 `TimedRobot` 架構，預設每 **20ms** 執行一次 `robotPeriodic()`。  
當單次迴圈執行時間超過 20ms，Driver Station 就會顯示 **"Loop overrun"** 警告，  
導致控制訊號延遲、動作卡頓，甚至影響比賽安全。

本次共修復 **5 個初始問題 + 12 個後續優化**，涵蓋全部主要檔案。

---

## 修復清單

### 修復 1 ⭐ 最高優先 — `Swerve.java`

**問題**：`SetRobotOrientation()` 每次 periodic 都阻塞主迴圈

| 項目 | 內容 |
|------|------|
| **檔案** | `src/main/java/frc/robot/subsystems/Swerve.java` |
| **位置** | `periodic()` 方法內，呼叫 Limelight 的區塊 |
| **嚴重程度** | 🔴 高 — **最可能的 overrun 主因** |

**修改前**：
```java
LimelightHelpers.SetRobotOrientation(limelightName, 
    gyroAngle.getDegrees(), getGyroRateDps(), 0, 0, 0, 0);
```

**修改後**：
```java
LimelightHelpers.SetRobotOrientation_NoFlush(limelightName, 
    gyroAngle.getDegrees(), getGyroRateDps(), 0, 0, 0, 0);
```

**為什麼要這樣做**：

`SetRobotOrientation()` 內部會呼叫：
```
SetRobotOrientation_INTERNAL(..., flush = true)
  └─> NetworkTableInstance.getDefault().flush()  ← 阻塞！
```

`flush()` 是一個**同步阻塞（blocking）** 操作，它會讓 Java 執行緒停在那裡，  
等待所有 NetworkTables 資料完整發送到 Limelight 之後才繼續。  

這個操作在 `periodic()` 裡**每 20ms 執行一次**。  
當網路稍有延遲或 Limelight 忙碌時，flush 等待時間可能超過數毫秒，  
直接導致整個迴圈超過 20ms 的時間預算，觸發 **loop overrun**。

`_NoFlush` 版本會讓 NetworkTables 在背景自動發送資料，完全不阻塞主迴圈。

---

### 修復 2 — `SwerveModule.java`

**問題**：`setState()` 和 `runRotorPID()` 重複讀取 CAN Bus

| 項目 | 內容 |
|------|------|
| **檔案** | `src/main/java/frc/robot/subsystems/SwerveModule.java` |
| **位置** | `setState()` 和 `runRotorPID()` 方法 |
| **嚴重程度** | 🟠 中高 — 每個迴圈多 8 次不必要的 CAN 讀取 |

**修改前**：
```java
public void setState(SwerveModuleState state) {
    state.optimize(getState().angle);      // 第 1 次 CAN 讀取
    state.cosineScale(getState().angle);   // 第 2 次 CAN 讀取
    runRotorPID(state.angle.getDegrees()); // 進入 runRotorPID...
    setThrottleSpeed(state.speedMetersPerSecond);
}

private void runRotorPID(double angle) {
    double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), angle); // 第 3 次 CAN 讀取
    setRotorSpeed(mRotorPID.atSetpoint() ? 0 : rotorOutput);
}
```

**修改後**：
```java
public void setState(SwerveModuleState state) {
    Rotation2d currentAngle = getState().angle;  // 只讀 1 次，快取結果

    state.optimize(currentAngle);
    state.cosineScale(currentAngle);

    runRotorPID(currentAngle, state.angle.getDegrees()); // 傳入已讀的角度
    setThrottleSpeed(state.speedMetersPerSecond);
}

private void runRotorPID(Rotation2d currentAngle, double targetAngle) {
    double rotorOutput = mRotorPID.calculate(currentAngle.getDegrees(), targetAngle); // 使用傳入值
    setRotorSpeed(mRotorPID.atSetpoint() ? 0 : rotorOutput);
}
```

**為什麼要這樣做**：

`getState().angle` 每次呼叫都會透過 **CAN Bus** 向 CANcoder 發出讀取請求。  
CAN Bus 的頻寬有限（預設 1Mbps），同一時間發出越多請求，每個設備的回應就越慢。

| 情況 | 每個模組 CAN 讀取次數 | 4 個模組總計 |
|------|----------------------|-------------|
| 修改前 | 3 次 | **12 次** |
| 修改後 | 1 次 | **4 次** |

每個迴圈節省了 **8 次不必要的 CAN 讀取**，直接降低 CAN Bus 負載。

---

### 修復 3 — `LightPollution.java`

**問題**：LED 每 20ms 更新一次，佔用迴圈時間

| 項目 | 內容 |
|------|------|
| **檔案** | `src/main/java/frc/robot/subsystems/LightPollution.java` |
| **位置** | `periodic()` 方法 |
| **嚴重程度** | 🟡 中 — 疊加增加迴圈時間 |

**修改前**：
```java
@Override
public void periodic() {
    ledPattern.applyTo(buffer);  // 每 20ms 計算一次動態效果
    led.setData(buffer);         // 每 20ms 序列化傳輸所有 LED 資料
    super.periodic();
}
```

**修改後**：
```java
private int ledUpdateCounter = 0;

@Override
public void periodic() {
    // 每 3 個迴圈 (約 60ms) 才更新一次 LED，減少 periodic 迴圈負擔
    if (++ledUpdateCounter >= 3) {
        ledPattern.applyTo(buffer);
        led.setData(buffer);
        ledUpdateCounter = 0;
    }
}
```

**為什麼要這樣做**：

`led.setData(buffer)` 需要透過 PWM/DIO 將每一顆 LED 的 R、G、B 值序列化傳出去，  
LED 數量越多，傳輸時間越長。`ledPattern.applyTo(buffer)` 對彩虹滾動等動態效果  
還需要做數學計算（時間插值、顏色混合等）。

這兩個操作對機器人動作控制**毫無幫助**，卻每 20ms 都佔用寶貴的迴圈時間。  
人眼對 60ms (約 16fps) 的 LED 更新完全感覺不到差異，  
但迴圈負擔降低了 **2/3**（從每 20ms 更新變為每 60ms 更新）。

---

### 修復 4 — `DriveSubsystem.java`

**問題**：`AutoBuilder.configure()` 被呼叫了兩次，設定互相衝突

| 項目 | 內容 |
|------|------|
| **檔案** | `src/main/java/frc/robot/subsystems/DriveSubsystem.java` |
| **位置** | 建構子 |
| **嚴重程度** | 🟡 中 — 造成自動路徑行為不可預測 |

**修改前**：
```java
// DriveSubsystem.java — 第一次呼叫（使用 Constants.AutoConstants 的 PID）
public DriveSubsystem(Swerve swerve) {
    AutoBuilder.configure(
        swerve::getPose, swerve::setPose, swerve::getChassisSpeeds,
        (speeds, feedforwards) -> swerve.setChassisSpeeds(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(kTranslationController_kP, kI, kD),
            new PIDConstants(kRotationController_kP, kI, kD)
        ),
        config,
        () -> { ... },
        this  // subsystem = DriveSubsystem
    );
}

// RobotContainer.java — 第二次呼叫（覆蓋前者，使用不同 PID）
AutoBuilder.configure(
    swerve::getPose, swerve::resetPose, swerve::getChassisSpeeds, swerve::drive,
    new PPHolonomicDriveController(
        new PIDConstants(3.0, 0.0, 0.0),
        new PIDConstants(1.8, 0.0, 0.0)
    ),
    config,
    swerve::isAllianceRed,
    swerve  // subsystem = Swerve
);
```

**修改後**：
```java
// DriveSubsystem.java — 清空建構子，不再重複配置
public class DriveSubsystem extends SubsystemBase {
    public DriveSubsystem(Swerve swerve) {
        // AutoBuilder.configure() 已移至 RobotContainer 統一管理
    }
}
```

**為什麼要這樣做**：

`AutoBuilder.configure()` 是一個**全域的靜態設定**，整個程式只能有一份有效配置。  
第二次呼叫會直接覆蓋第一次的設定。  
兩次設定的差異：

| 項目 | DriveSubsystem（被覆蓋） | RobotContainer（實際生效） |
|------|------------------------|--------------------------|
| 平移 PID kP | `kTranslationController_kP = 1.3` | `3.0` |
| 旋轉 PID kP | `kRotationController_kP = 0.2` | `1.8` |
| drive 方法 | `setChassisSpeeds()` (有 0.185 縮放) | `drive()` (無縮放) |
| subsystem | `DriveSubsystem` | `Swerve` |

保留 `RobotContainer` 中的配置（它直接使用 `swerve::drive`，路徑跟蹤更準確），  
移除 `DriveSubsystem` 中多餘的呼叫，避免混淆。

---

### 修復 5 — `RobotContainer.java`

**問題**：Command 實例共用，可能導致排程衝突

| 項目 | 內容 |
|------|------|
| **檔案** | `src/main/java/frc/robot/RobotContainer.java` |
| **位置** | 欄位宣告與 `configureBindings()` |
| **嚴重程度** | 🟡 中 — 造成 Command 執行異常或意外取消 |

**修改前**：
```java
// 欄位：建立一個 Command 實例
private Command shoot_command = Commands.sequence(
    Commands.waitUntil(() -> shooterSubsystem.isAtSpeed(50)),
    transport.sys_runTransport()
);

// 同一個實例被兩處綁定
NamedCommands.registerCommand("transport wait shoot", shoot_command); // Auto 用
driverController.rightTrigger(0.1).whileTrue(shoot_command);         // Teleop 用
```

**修改後**：
```java
// 工廠方法：每次呼叫建立全新的 Command 實例
private Command createShootCommand() {
    return Commands.sequence(
        Commands.waitUntil(() -> shooterSubsystem.isAtSpeed(50)),
        transport.sys_runTransport()
    );
}

// 各自拿到獨立的實例
NamedCommands.registerCommand("transport wait shoot", createShootCommand()); // Auto 用
driverController.rightTrigger(0.1).whileTrue(createShootCommand());          // Teleop 用
```

同樣的修改套用到 `auto_shoot_command`、`far_auto_shoot_command`、`auto_intake_command`。

**為什麼要這樣做**：

WPILib 的 `Command` 是**有狀態的物件**，每個實例在 `CommandScheduler` 中只能有一個排程狀態（未啟動 / 執行中 / 已結束）。  

當同一個實例同時被 `NamedCommands`（自動模式）和 `Trigger`（手動模式）持有時：

```
問題情境：
1. 自動模式啟動 shoot_command（實例狀態 → 執行中）
2. teleopInit 切換到手動，Trigger 系統嘗試再次啟動同一個 shoot_command
3. CommandScheduler 發現它已在執行，對其發出 interrupt
4. Command 被意外取消 → 球沒有發射出去
```

改用工廠方法後，Auto 和 Teleop 各自持有**完全獨立的實例**，互不影響。

---

## 修復效果總結

| # | 問題 | 修改檔案 | 預期改善 |
|---|------|----------|----------|
| 1 | `flush()` 阻塞主迴圈 | `Swerve.java` | **消除最主要的 overrun 來源** |
| 2 | 重複 CAN 讀取 ×8 次/迴圈 | `SwerveModule.java` | 降低 CAN Bus 負載 |
| 3 | LED 每 20ms 更新 | `LightPollution.java` | 減少 2/3 的 LED 傳輸開銷 |
| 4 | `AutoBuilder` 重複設定 | `DriveSubsystem.java` | 自動路徑行為穩定 |
| 5 | Command 實例共用 | `RobotContainer.java` | 消除排程衝突 |

**建置結果**：`BUILD SUCCESSFUL` ✅（無新增錯誤，僅原有的 deprecation warnings）

---

## 補充：Loop Overrun 診斷方式

若未來再次出現 overrun，可透過以下方式定位：

1. **Driver Station Log Viewer**：開啟 `.dslog` 檔，找 "loop overrun" 時間點，對照執行的 Command
2. **SmartDashboard / Shuffleboard**：將可疑的 subsystem `putData()` 到儀表板，觀察哪個 `periodic()` 花最多時間
3. **Phoenix Tuner X**：檢查 CAN Bus 利用率，若超過 50% 就需要優化讀取頻率
4. **WPILib Profiling**：使用 `Tracer` 類別在各段程式碼前後記錄時間戳

```java
// 例：在 Swerve.periodic() 中加入追蹤
edu.wpi.first.wpilibj.Tracer.resetTimer();
// ... 你的程式碼 ...
edu.wpi.first.wpilibj.Tracer.printEpochs(); // 印出各段耗時
```

---

## 後續優化與功能新增（Overrun 修復後）

以下修改是在 Overrun 問題修復之後，進一步針對速度、功能、穩定性所做的優化。

---

### 優化 6 — 速度控制鏈重構

**問題**：機器人最大速度只有 2 m/s，遠低於 MK4i L3 的物理極限 5.13 m/s

| 項目 | 內容 |
|------|------|
| **影響檔案** | `Constants.java`, `ManualDrive.java`, `Swerve.java`, `SwerveModuleKraken.java` |
| **修復類型** | 性能優化 |

**修改內容**：

1. **Constants.java** — 新增物理最大速度常數：
   ```java
   public static final double kMaxPhysicalSpeedMps = 
       (6000.0 / 60.0 / kThrottleGearRatio) * kWheelDiameterMeters * Math.PI;
   // ≈ 5.13 m/s
   ```

2. **ManualDrive.java** — 搖桿輸出改用物理最大速度：
   - `xCtl *= SwerveConstants.kMaxPhysicalSpeedMps`（取代舊的 `×6.0`）
   - `yCtl *= SwerveConstants.kMaxPhysicalSpeedMps`
   - `zCtl *= SwerveConstants.kMaxPhysicalSpeedMps`

3. **Swerve.java** — `getMaxVelocity()` 改用 `kMaxPhysicalSpeedMps`（取代硬編碼 2）

4. **SwerveModuleKraken.java** — `setThrottleSpeed()` 使用 `kMaxPhysicalSpeedMps` 計算百分比

---

### 優化 7 — TrapezoidProfile → SlewRateLimiter

**問題**：TrapezoidProfile 被誤用 — 將速度值當作「位置」輸入，目標速度設為 0，導致中途減速

| 項目 | 內容 |
|------|------|
| **檔案** | `Swerve.java` |
| **修復類型** | Bug 修復 + 性能優化 |

**修改前**：
```java
TrapezoidProfile xProfile = new TrapezoidProfile(new Constraints(10, 10));
// 每次呼叫: xProfile.calculate(0.02, new State(targetSpeed, 0), new State(0, 0))
// 問題：把 targetSpeed 當「位置」，目標位置=0，所以永遠在減速到停
```

**修改後**：
```java
SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(20); // 20 m/s² 加速度限制
SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(20);
SlewRateLimiter zSpeedLimiter = new SlewRateLimiter(15); // 旋轉 15 rad/s²
```

**效果**：加速度響應正確，不再提前減速，vx 可達 5.13 m/s。

---

### 優化 8 — 模擬器支援

| 項目 | 內容 |
|------|------|
| **影響檔案** | `build.gradle`, `SwerveModuleKraken.java`, `Swerve.java`, `Robot.java` |
| **修復類型** | 開發工具 |

**修改內容**：
- `build.gradle`: `includeDesktopSupport = true`
- `SwerveModuleKraken.java`: 新增 sim 物理追蹤（simThrottlePosition/Velocity）
- `Swerve.java`: sim gyro 角度積分、Field2d 顯示、`setStateCommand` 移至 `periodic()`
- `Robot.java`: Loop 計時監控（CurrentMs、MaxMs、Overrun 指示器）
- `SwerveModuleKraken.java`: `RuntimeException` → `DriverStation.reportWarning`

---

### 優化 9 — IMU 與位姿同步

**問題**：自動模式結束後需手動按 Button 8 重新校正 IMU

| 項目 | 內容 |
|------|------|
| **檔案** | `Swerve.java`, `RobotContainer.java` |
| **修復類型** | Bug 修復 |

**根因**：
1. `resetPose()` 只重設 poseEstimator，沒有同步 Pigeon2 yaw
2. `resetIMU()` 只重設 Pigeon2，沒有同步 poseEstimator
3. 視覺融合的角度 stddev 為 999999（完全不修正角度）

**修改內容**：

1. **`resetPose(Pose2d)`** — PathPlanner auto 開始時呼叫，現在會**同步設定 IMU yaw** 為目標位姿角度
2. **`resetIMU(double)`** — 手動重設 IMU 後，同步**更新 poseEstimator 航向**
3. **`resetPoseToLimelight()`** — 改為只校正 XY 位置，**角度保持 IMU**（因使用者不信任 Limelight 角度）
4. **`teleopInit()`** — 進入 teleop 時自動呼叫 `resetPoseToLimelight()` 校正 XY

---

### 新功能 10 — AutoAimAndShoot 指令

| 項目 | 內容 |
|------|------|
| **新增檔案** | `commands/AutoAimAndShoot.java` |
| **修改檔案** | `Constants.java`, `ShooterSubsystem.java`, `TransportSubsystem.java`, `RobotContainer.java` |

**功能說明**：
按住 Left Bumper 時：
1. 自動旋轉底盤面向 Speaker（PID 控制，kP=5.0）
2. 依距離線性內插查表調整射手 RPS（1m=40RPS ~ 5m=80RPS）
3. 角度對齊 (±2°) + 射手達速 → 自動啟動 Transport 送球
4. 透過 `setAimSpeed()` 疊加旋轉，不影響手動平移控制

**新增 Constants (`AutoAimConstants`)**：
- 藍方 Speaker 座標: `(0.0, 5.55)`
- 紅方 Speaker 座標: `(16.54, 5.55)`
- 距離-RPS 對照表：5 組數據點

**新增 Subsystem 方法**：
- `ShooterSubsystem`: `setTargetVelocity(rps)`, `stopShooter()`, `getCurrentRps()`
- `TransportSubsystem`: `runTransport()`, `stopTransport()`

---

### 優化 11 — Intake 改為閉環轉速控制

**問題**：Intake 使用 DutyCycleOut(0.4)，只給 40% 電壓，遇阻力時力量不足

| 項目 | 內容 |
|------|------|
| **檔案** | `IntakeRollerSubsystem.java` |
| **修復類型** | 性能優化 |

**修改前**：
```java
// DutyCycleOut 開環百分比控制
private final DutyCycleOut voltageRequest = new DutyCycleOut(0);
private final double MAX_OUTPUT = 0.4; // 只給 40% 電壓

setSpeed(MAX_OUTPUT); // 固定輸出，遇阻力轉速下降
```

**修改後**：
```java
// VelocityVoltage 閉環轉速控制
private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
private static final double INTAKE_TARGET_RPS = 60.0;

// PID 設定 (X44 馬達)
config.Slot0.kV = 0.12;  // 前饋
config.Slot0.kP = 0.2;   // 比例
config.Slot0.kI = 0.01;  // 積分

setVelocity(INTAKE_TARGET_RPS); // 閉環控制，遇阻力自動補償電壓
```

**效果**：
- 不再限制最大電壓（由 PID 自動決定）
- 遇到阻力時自動加大電壓維持轉速
- 電流限制：Stator 60A + Supply 40A（保護馬達）
- 新增 Dashboard 監控：`Intake/Actual RPS`, `Intake/Leader Current`

---

## 完整修改檔案清單

| # | 檔案 | 修改類型 |
|---|------|----------|
| 1 | `Swerve.java` | Overrun 修復 + SlewRateLimiter + IMU 同步 + Limelight 校正 + sim |
| 2 | `SwerveModule.java` | CAN 讀取快取 |
| 3 | `SwerveModuleKraken.java` | kMaxPhysicalSpeedMps + sim 物理 + 錯誤處理 |
| 4 | `LightPollution.java` | LED 更新降頻 |
| 5 | `DriveSubsystem.java` | 移除重複 AutoBuilder |
| 6 | `RobotContainer.java` | Command 工廠方法 + AutoAimAndShoot 綁定 + teleopInit 校正 |
| 7 | `Constants.java` | kMaxPhysicalSpeedMps + AutoAimConstants |
| 8 | `ManualDrive.java` | 速度乘數改用 kMaxPhysicalSpeedMps |
| 9 | `ShooterSubsystem.java` | 新增 setTargetVelocity / stopShooter / getCurrentRps |
| 10 | `TransportSubsystem.java` | 新增 runTransport / stopTransport |
| 11 | `AutoAimAndShoot.java` | 🆕 自動瞄準射擊 Command |
| 12 | `IntakeRollerSubsystem.java` | DutyCycleOut → VelocityVoltage 閉環控制 |
| 13 | `Robot.java` | Loop 計時監控 |
| 14 | `build.gradle` | includeDesktopSupport = true |

---

### 優化 12 — 遙測節流 (Telemetry Throttle)

**問題**：所有 NetworkTables `putNumber()` / `putString()` 每 20ms 執行一次，佔用迴圈時間與頻寬

| 項目 | 內容 |
|------|------|
| **影響檔案** | `Constants.java`, `Robot.java`, `Swerve.java`, `ManualDrive.java`, `AutoAimAndShoot.java`, `ShooterSubsystem.java`, `IntakeRollerSubsystem.java`, `IntakeArmSubsystem.java` |
| **修復類型** | 性能優化 |

**修改前**：
```java
// 每個 periodic() 都會執行 putNumber()，50Hz × 所有子系統 ≈ 350 次/秒
@Override
public void periodic() {
    SmartDashboard.putNumber("Shooter/Velocity", getVelocityRps());
    SmartDashboard.putNumber("Shooter/Voltage", getMotorVoltage());
    // ... 每個值都是 50Hz
}
```

**修改後**：
```java
// Constants.java
public static final int kTelemetryDivider = 5; // 50Hz / 5 = 10Hz

// ShooterSubsystem.java (範例)
private int telemetryCounter = 0;
@Override
public void periodic() {
    if (++telemetryCounter >= Constants.kTelemetryDivider) {
        telemetryCounter = 0;
        SmartDashboard.putNumber("Shooter/Velocity", velocitySignal.refresh().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Voltage", voltageSignal.refresh().getValueAsDouble());
        // ...
    }
}
```

**效果**：
| 指標 | 修改前 | 修改後 | 降幅 |
|------|--------|--------|------|
| NT 寫入頻率 | 50Hz | 10Hz | ↓80% |
| 預估 NT 寫入/秒 | ~350 | ~70 | ↓80% |

**為什麼要這樣做**：

`SmartDashboard.putNumber()` 每次呼叫都要進行序列化、CRC 計算、並透過 NetworkTables 4.0 的 pubsub 發佈。  
50Hz 對於人類觀察儀表板根本用不到（眼睛分辨不到 >10fps），卻佔用了大量的 NT server 處理時間。  
10Hz 已經足夠所有調試需求，且大幅減少 GC 壓力和序列化開銷。

特別注意 `Swerve.java` 的實作方式：用一個 `telemetryThisCycle` boolean flag，  
在 `periodic()` 開頭設定，然後在 `runSetStates()` 和 `setModuleStates()` 中共用同一個 flag，  
確保一次「遙測 cycle」中所有值的時間戳一致。

---

### 優化 13 — Phoenix 6 StatusSignal 快取 & 頻率控制

**問題**：每次呼叫 `motor.getVelocity()` 都會建立新的 StatusSignal 物件，且預設 250Hz 更新

| 項目 | 內容 |
|------|------|
| **影響檔案** | `Swerve.java`, `SwerveModuleKraken.java`, `SwerveModule.java`, `ShooterSubsystem.java`, `IntakeRollerSubsystem.java`, `IntakeArmSubsystem.java` |
| **修復類型** | CAN Bus 優化 |

**修改前**：
```java
// 每次呼叫都建立新的 StatusSignal wrapper，且所有 signal 預設 250Hz
public double getVelocityRps() {
    return leaderMotor.getVelocity().getValueAsDouble(); // 臨時物件 + 250Hz polling
}
```

**修改後**：
```java
// Phoenix 6 v26 推薦方式：使用 units-aware 型別
private final StatusSignal<AngularVelocity> velocitySignal;
private final StatusSignal<Voltage> voltageSignal;
private final StatusSignal<Current> currentSignal;

// 建構子中：快取 + 設定頻率 + 最佳化
velocitySignal = leaderMotor.getVelocity();
velocitySignal.setUpdateFrequency(50);     // 閉環回饋需 50Hz
voltageSignal = leaderMotor.getMotorVoltage();
voltageSignal.setUpdateFrequency(10);       // 遙測用，10Hz 足夠
currentSignal = leaderMotor.getStatorCurrent();
currentSignal.setUpdateFrequency(10);
leaderMotor.optimizeBusUtilization();       // 未使用的 signal 自動降頻

// 使用時
public double getVelocityRps() {
    return velocitySignal.refresh().getValueAsDouble(); // 無新物件建立
}
```

**效果**：

| 元件 | Signal 數量 | 修改前頻率 | 修改後頻率 | 節省 frames/s |
|------|------------|-----------|-----------|--------------|
| Pigeon2 | 2 | 250Hz | 100/50Hz | ~350 |
| 4× SwerveModuleKraken | 8 | 250Hz | 100Hz | ~600 |
| 4× SwerveModule/CANcoder | 4 | 250Hz | 100Hz | ~600 |
| ShooterSubsystem | 3 | 250Hz | 50/10/10Hz | ~680 |
| IntakeRollerSubsystem | 3 | 250Hz | 50/10/10Hz | ~680 |
| IntakeArmSubsystem | 3 | 250Hz | 50/10/10Hz | ~680 |
| **合計** | **23** | | | **~3590** |

加上 `optimizeBusUtilization()` 將每台設備的 ~10 個未使用 signal 從 250Hz 降至 4Hz，  
額外節省 ~2000 frames/s。**總節省約 80% CAN 頻寬**。

**為什麼要這樣做**：

CAN Bus 頻寬有限（1Mbps），且 Phoenix 6 預設每個 `StatusSignal` 都是 250Hz polling。  
一台 TalonFX 有 ~12 個 signal（位置、速度、電壓、電流、溫度...），4 台 Swerve 就是 12000 frames/s。  
加上 Pigeon2、Shooter、Intake 的 signal，輕易超過 CAN Bus 理論上限。

快取 `StatusSignal<T>` 欄位避免了每次呼叫建立臨時物件（降低 GC 壓力），  
`setUpdateFrequency()` 只給需要的 signal 合理的更新頻率，  
`optimizeBusUtilization()` 自動處理其餘 signal。

---

### 優化 14 — 聯盟對稱重構 (Alliance Symmetry)

**問題**：紅/藍方座標分別定義，AutoAimAndShoot 和 RobotContainer 中大量 if/else 分支

| 項目 | 內容 |
|------|------|
| **影響檔案** | `Constants.java`, `AutoAimAndShoot.java`, `RobotContainer.java` |
| **修復類型** | 架構重構 |

**修改前**：
```java
// Constants.java — 重複定義
public static final double kBlueHubX = 4.626;
public static final double kBlueHubY = 4.035;
public static final double kRedHubX = 16.541 - 4.626;
public static final double kRedHubY = 4.035;
public static final double kBlueReturnAngleRad = Math.PI;
public static final double kRedReturnAngleRad = 0;

// AutoAimAndShoot.java — 大量分支
if (isRed) {
    if (robotX > kFieldMidX) {
        targetAngle = Math.atan2(kRedHubY - robotY, kRedHubX - robotX);
        targetRps = interpolateRps(distToRedHub);
    } else {
        targetAngle = kRedReturnAngleRad;
        targetRps = kMidFieldReturnRps;
    }
} else {
    if (robotX < kFieldMidX) {
        targetAngle = Math.atan2(kBlueHubY - robotY, kBlueHubX - robotX);
        targetRps = interpolateRps(distToBlueHub);
    } else {
        targetAngle = kBlueReturnAngleRad;
        targetRps = kMidFieldReturnRps;
    }
}
```

**修改後**：
```java
// Constants.java — 單一來源 + helper 方法
public static final double kHubX = 4.626;   // 藍方基準
public static final double kHubY = 4.035;
public static final double kReturnAngleRad = Math.PI;

public static Translation2d getHubPosition(boolean isRed) {
    return isRed ? new Translation2d(kFieldLength - kHubX, kHubY)
                 : new Translation2d(kHubX, kHubY);
}
public static double getReturnAngleRad(boolean isRed) {
    return isRed ? Math.PI - kReturnAngleRad : kReturnAngleRad;
}
public static boolean isInOwnZone(double robotX, boolean isRed) {
    return isRed ? robotX > kFieldMidX : robotX < kFieldMidX;
}

// AutoAimAndShoot.java — 統一邏輯（無分支）
Translation2d hub = AutoAimConstants.getHubPosition(isRed);
if (AutoAimConstants.isInOwnZone(robotX, isRed)) {
    targetAngle = Math.atan2(hub.getY() - robotY, hub.getX() - robotX);
    targetRps = ShooterSubsystem.interpolateRps(distToHub);
} else {
    targetAngle = AutoAimConstants.getReturnAngleRad(isRed);
    targetRps = kMidFieldReturnRps;
}
```

**效果**：
- 40 行 if/else → 15 行統一邏輯
- 未來調 Hub 座標只需改一處，紅方自動鏡像推導
- 消除「改藍忘紅」的風險

---

### 優化 15 — Follower 馬達修正（三項）

**問題**：Follower 馬達存在三個不同層面的 bug

| 項目 | 內容 |
|------|------|
| **影響檔案** | `TransportSubsystem.java`, `IntakeRollerSubsystem.java`, `IntakeArmSubsystem.java` |
| **修復類型** | Bug 修復（馬達控制） |

#### 15a — Transport：VelocityVoltage 請求物件共用

```java
// ❌ 修改前：兩顆馬達共用同一個 mutable 物件
private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
public void setTransportVelocity(double shootRps, double transportRps) {
    up_to_shoot.setControl(velocityRequest.withVelocity(shootRps));
    // ↑ velocityRequest 的內部 Velocity 被改為 shootRps
    transport.setControl(velocityRequest.withVelocity(transportRps));
    // ↑ velocityRequest 的內部 Velocity 被改為 transportRps
    // 但因為 CTRE 是非同步發送，up_to_shoot 可能拿到 transportRps
}

// ✅ 修改後：各自獨立物件
private final VelocityVoltage upToShootRequest = new VelocityVoltage(0);
private final VelocityVoltage transportRequest = new VelocityVoltage(0);
public void setTransportVelocity(double shootRps, double transportRps) {
    up_to_shoot.setControl(upToShootRequest.withVelocity(shootRps));
    transport.setControl(transportRequest.withVelocity(transportRps));
}
```

**為什麼要這樣做**：`VelocityVoltage.withVelocity()` 是 **mutable** 操作，  
它修改 `this` 並返回 `this`。因此共用物件時第二次 `withVelocity()` 會覆蓋第一次設定。

#### 15b — IntakeRoller：Follower 雙重反轉

```java
// ❌ 修改前：Inverted + Opposed = 雙重反轉 → 結果同向轉
followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
followerMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
// Opposed 已經自動反轉一次，Inverted 又反轉一次 → 互相抵消

// ✅ 修改後：移除 Inverted 設定
// followerConfig 不設定 Inverted（保持預設 CounterClockwise_Positive）
followerMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
// 只靠 Opposed 自動反轉
```

**為什麼要這樣做**：Phoenix 6 `Follower` 的 `Opposed` 參數已經處理了方向反轉。  
再手動設定 `Inverted = Clockwise_Positive` 等於反轉了兩次，最終效果是兩顆馬達同向轉。  
對 Intake 滾輪來說，同向轉意味著只有一側滾輪在吸球，另一側在推球。

#### 15c — IntakeRoller & IntakeArm：Follower PID 更新無意義

```java
// ❌ 修改前：periodic() 中對 Follower 也更新 PID
if (hasNewPIDValues) {
    Slot0Configs newSlot0 = new Slot0Configs().withKV(kV).withKP(kP).withKI(kI).withKD(kD);
    leaderMotor.getConfigurator().apply(newSlot0);
    followerMotor.getConfigurator().apply(newSlot0);  // ← 浪費 CAN
}

// ✅ 修改後：只更新 Leader
if (hasNewPIDValues) {
    Slot0Configs newSlot0 = new Slot0Configs().withKV(kV).withKP(kP).withKI(kI).withKD(kD);
    leaderMotor.getConfigurator().apply(newSlot0);
    // Follower 鏡像 Leader 的輸出，不跑自己的閉環，PID 參數完全不使用
}
```

**為什麼要這樣做**：Phoenix 6 的 `Follower` 控制模式下，馬達直接鏡像 Leader 的控制輸出，  
不會運行自己的 PID 計算。對 Follower 寫入 `Slot0Configs` 完全無效，  
卻每次觸發都會佔用一次 CAN `apply()` 操作（多個 CAN frame + 等待 ACK）。

---

### 修復 16 — IMU 重設與聯盟色感知 — `Swerve.java`

| 項目 | 內容 |
|------|------|
| **檔案** | `src/main/java/frc/robot/subsystems/Swerve.java` |
| **位置** | `setPose()`, `resetPose()`, `resetIMU()`, `resetIMU(double)` |
| **嚴重程度** | 🔴 高 — 紅方聯盟 field-oriented 驅動方向完全反轉 |

#### 16a — `setPose()` 不同步 IMU yaw

```java
// ❌ 修改前：只重設 poseEstimator/odometry，IMU yaw 不變
public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
}
// 問題：IMU 與位姿的角度不一致，field-oriented 駕駛基準錯誤

// ✅ 修改後：同步 IMU yaw，確保 field-oriented 驅動正確
public void setPose(Pose2d pose) {
    double headingDeg = pose.getRotation().getDegrees();
    mPigeonIMU.setYaw(headingDeg);          // 同步 IMU
    if (RobotBase.isSimulation()) {
        simGyroAngleDeg = headingDeg;
    }
    Rotation2d newGyroAngle = getGyroAngle();
    poseEstimator.resetPosition(newGyroAngle, getModulePositions(), pose);
}
```

**為什麼要這樣做**：`setPose()` 是設定機器人位姿的基礎方法，被 `resetPoseToLimelight()`、`resetIMU(double)` 等間接呼叫。  
如果只重設 poseEstimator 而不同步 IMU，`getGyroAngle()` 回傳的角度仍是舊值，  
field-oriented 駕駛的前進方向會與機器人實際朝向脫節。

#### 16b — `resetPose()` 簡化為委派 `setPose()`

```java
// ❌ 修改前：resetPose() 自己也做一遍完整的 IMU 同步 + poseEstimator 重設
public void resetPose(Pose2d pose) {
    mPigeonIMU.setYaw(headingDeg);
    poseEstimator.resetPosition(...);
    mOdometry.resetPosition(...);
}

// ✅ 修改後：直接委派，消除重複邏輯
public void resetPose(Pose2d pose) {
    setPose(pose);
}
```

**為什麼要這樣做**：`setPose()` 現在已經完整處理 IMU + poseEstimator 同步，  
`resetPose()`（PathPlanner `resetOdom: true` 呼叫的方法）不需要重複實作，單純委派即可。

#### 16c — `resetIMU()` 聯盟色感知

```java
// ❌ 修改前：永遠重設為 0°
public void resetIMU() {
    resetIMU(0);
}
// 問題：紅方聯盟開機時機器人面向藍方（WPILib 藍方原點座標系 = 180°），卻被重設為 0°

// ✅ 修改後：自動偵測聯盟色
public void resetIMU() {
    resetIMU(isAllianceRed() ? 180.0 : 0.0);
}
```

**為什麼要這樣做**：WPILib 使用藍方原點座標系，藍方面向對面是 0°，紅方面向對面是 180°。  
機器人開機或緊急按按鈕 8 時，`resetIMU()` 必須根據聯盟色設定正確的初始角度，  
否則 field-oriented 駕駛和 PathPlanner 路徑的方向都會反轉。

#### 16d — `resetIMU(double)` 消除冗餘

```java
// ❌ 修改前：先手動設 IMU，再呼叫 setPose()（內部又設一次 IMU）
public void resetIMU(double angle) {
    mPigeonIMU.reset();
    mPigeonIMU.setYaw(angle);      // 第一次設定
    setPose(correctedPose);         // 第二次設定（setPose 內部也 setYaw）
}

// ✅ 修改後：只透過 setPose() 統一處理
public void resetIMU(double angle) {
    Pose2d currentPose = getPose();
    Pose2d correctedPose = new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(angle));
    setPose(correctedPose);
}
```

**為什麼要這樣做**：`setPose()` 已經是唯一的 IMU 同步入口，所有需要重設角度的方法  
都應該透過它，避免散落在多處的 `setYaw()` 呼叫造成維護困難或遺漏。

---

### 修復 17 — 專案清理與 Deprecated API 消除

| 項目 | 內容 |
|------|------|
| **涉及檔案** | 6 個檔案（刪除 2 個 + 修改 4 個） |
| **嚴重程度** | 🟡 中 — 編譯警告 9 個 + 死碼累積 |

#### 17a — 刪除 `DriveSubsystem.java`

```java
// ❌ 修改前：空殼類別，只有空的建構子
public class DriveSubsystem extends SubsystemBase {
    public DriveSubsystem(Swerve swerve) {
        // AutoBuilder.configure() 已移至 RobotContainer 統一管理
    }
}

// ✅ 修改後：直接刪除檔案
// 同時清理 RobotContainer 中的 import 和註解
```

**為什麼要這樣做**：`DriveSubsystem` 的 `AutoBuilder.configure()` 已在修復 4 時移至 `RobotContainer`，  
檔案只剩空殼，保留只會混淆維護者。

#### 17b — 刪除 `AutoAim.java`

```java
// ❌ 修改前：整份檔案都被註解起來（100+ 行死碼）
// public class AutoAim extends SubsystemBase { ... }

// ✅ 修改後：直接刪除檔案
// 瞄準邏輯已轉移至 AutoAimAndShoot.java Command
```

**為什麼要這樣做**：`AutoAim` 的功能已被 `AutoAimAndShoot` 完全取代，  
留下全部被註解的舊檔案只會與新 Command 混淆。

#### 17c — 消除 Phoenix 6 Deprecated API 警告（3 處）

```java
// ❌ 修改前：使用已棄用的 String 參數構造函數（9 個 warning 中的 3 個）
throttleMotor = new TalonFX(throttleID, "DRIVETRAIN");
rotorMotor = new TalonFX(rotorID, "DRIVETRAIN");
mRotorEncoder = new CANcoder(rotorEncoderID, "DRIVETRAIN");

// ✅ 修改後：使用 CANBus 物件（Phoenix 6 v26+ 標準）
// Constants.java — 新增靜態常數
public static final CANBus kDrivetrainCANBus = new CANBus("DRIVETRAIN");

// SwerveModuleKraken.java
throttleMotor = new TalonFX(throttleID, SwerveConstants.kDrivetrainCANBus);
rotorMotor = new TalonFX(rotorID, SwerveConstants.kDrivetrainCANBus);

// SwerveModule.java
mRotorEncoder = new CANcoder(rotorEncoderID, SwerveConstants.kDrivetrainCANBus);
```

**為什麼要這樣做**：Phoenix 6 v26 標記 `new TalonFX(int, String)` 和 `new CANcoder(int, String)` 為  
`@Deprecated(forRemoval=true)`，2027 將移除。改用 `CANBus` 物件是官方推薦做法，  
同時統一 CAN bus 名稱到單一常數，避免多處寫死字串。

#### 17d — 消除 REVLib Deprecated API 警告（6 處）

```java
// ❌ 修改前：使用已棄用的 SparkBase.ResetMode / SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
throttleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

// ✅ 修改後：使用新的頂層類別
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
throttleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
// 方法呼叫不變，只有 import 路徑改為 com.revrobotics.* 頂層
```

**為什麼要這樣做**：REVLib 2026 將內嵌在 `SparkBase` 的 `ResetMode`/`PersistMode` 標為  
`@Deprecated(since="2026", forRemoval=true)`，並提供頂層替代類別。

#### 17e — 清理 `ShooterSubsystem.sys_manualShoot()`

```java
// ❌ 修改前：參數用 int、Javadoc 提到 triggerInput、殘留板機死區註解
public Command sys_manualShoot(int TargetRPS) {
    return this.runEnd(
        () -> {
            // double rawValue = triggerInput.getAsDouble();
            // double adjustedValue = MathUtil.applyDeadband(rawValue, TRIGGER_DEADBAND);
            setVelocity(TargetRPS);
        },
        () -> { stop(); }
    );
}

// ✅ 修改後：參數改 double、清除誤導註解、精簡
public Command sys_manualShoot(double targetRps) {
    return this.runEnd(
        () -> setVelocity(targetRps),
        () -> stop()
    );
}
// 同時移除未使用的 import: DoubleSupplier, MathUtil
```

**為什麼要這樣做**：RPS 是連續值應用 `double`；殘留的板機變速註解  
與實際「一鍵固定轉速」行為矛盾，會誤導未來維護者。

---

## 完整修改檔案清單

| # | 檔案 | 修改類型 |
|---|------|----------|
| 1 | `Swerve.java` | Overrun 修復 + SlewRateLimiter + IMU 同步 + Limelight 校正 + sim + StatusSignal 快取 + 遙測節流 + IMU 聯盟色感知 |
| 2 | `SwerveModule.java` | CAN 讀取快取 + StatusSignal 快取 + CANcoder CANBus 物件 |
| 3 | `SwerveModuleKraken.java` | kMaxPhysicalSpeedMps + sim 物理 + 錯誤處理 + StatusSignal 快取 + TalonFX CANBus 物件 |
| 4 | `LightPollution.java` | LED 更新降頻 |
| 5 | ~~`DriveSubsystem.java`~~ | ❌ 已刪除（空殼，AutoBuilder 移至 RobotContainer） |
| 6 | `RobotContainer.java` | Command 工廠方法 + AutoAimAndShoot 綁定 + teleopInit 校正 + 聯盟對稱 + 清理死碼 import |
| 7 | `Constants.java` | kMaxPhysicalSpeedMps + AutoAimConstants + 聯盟對稱 helper + kTelemetryDivider + kDrivetrainCANBus |
| 8 | `ManualDrive.java` | 速度乘數改用 kMaxPhysicalSpeedMps + 遙測節流 |
| 9 | `ShooterSubsystem.java` | setTargetVelocity / interpolateRps + StatusSignal 快取 + 遙測節流 + sys_manualShoot 清理 |
| 10 | `TransportSubsystem.java` | runTransport / stopTransport + VelocityVoltage 分離 |
| 11 | `AutoAimAndShoot.java` | 🆕 自動瞄準射擊 Command + 聯盟對稱 + 遙測節流 |
| 12 | `IntakeRollerSubsystem.java` | VelocityVoltage 閉環 + Follower 修正 + StatusSignal 快取 + 遙測節流 |
| 13 | `IntakeArmSubsystem.java` | Follower PID 修正 + StatusSignal 快取 + 遙測節流 |
| 14 | `Robot.java` | Loop 計時監控 + 遙測節流 |
| 15 | `build.gradle` | includeDesktopSupport = true |
| 16 | ~~`AutoAim.java`~~ | ❌ 已刪除（全部被註解，功能移至 AutoAimAndShoot） |
| 17 | `SwerveModuleNeo.java` | REVLib ResetMode/PersistMode import 更新 |
