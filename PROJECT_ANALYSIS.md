# RobotControl2026 — 完整專案深度解析

**最後更新**：2026-03-08  
**遊戲**：2026 FRC REBUILT  
**得分方式**：將 Fuel（遊戲物件）射入 Hub  
**場地**：16.541m × 8.069m（對稱場地）

---

## 一、系統架構總覽

```
┌──────────────────────────────────────────────────────────────┐
│                     TimedRobot (50Hz)                        │
│  Robot.java → RobotContainer.java                           │
│    ├─ CommandScheduler                                       │
│    ├─ Loop timing monitor (kTelemetryDivider=5 → 10Hz NT)   │
│    └─ ShuffleboardManager (分頁管理)                         │
├──────────────────────────────────────────────────────────────┤
│                      Commands                                │
│  ┌─────────────┐ ┌───────────────────┐ ┌──────────────┐    │
│  │ ManualDrive  │ │ AutoAimAndShoot   │ │  Drive2Tag   │    │
│  │ (default)    │ │ (LB / RT)         │ │  (A button)  │    │
│  │ req: Swerve  │ │ req: Shooter,     │ │ req: Swerve  │    │
│  │              │ │      Transport    │ │              │    │
│  └─────────────┘ └───────────────────┘ └──────────────┘    │
├──────────────────────────────────────────────────────────────┤
│                     Subsystems                               │
│  ┌────────┐ ┌──────────┐ ┌───────────┐ ┌──────────────┐    │
│  │ Swerve │ │ Shooter  │ │ Transport │ │IntakeRoller  │    │
│  │ 4×MK4i │ │ 2×TalonFX│ │ 2×TalonFX │ │ 2×TalonFX    │    │
│  │ L3     │ │ Leader+  │ │ 獨立控制   │ │ Leader+      │    │
│  │ Kraken │ │ Follower │ │           │ │ Follower     │    │
│  └────────┘ └──────────┘ └───────────┘ └──────────────┘    │
│  ┌────────────┐ ┌────────────┐                              │
│  │IntakeArm   │ │LightPoll.  │  (IntakeArm 目前未啟用)      │
│  │2×TalonFX   │ │ LED strip  │                              │
│  └────────────┘ └────────────┘                              │
├──────────────────────────────────────────────────────────────┤
│                     Hardware Layer                            │
│  DRIVETRAIN CAN bus: 8×Swerve motors + 4×CANcoder           │
│  Default CAN bus: Pigeon2, Shooter, Transport, IntakeRoller  │
│  Vision: Limelight (MegaTag2, pipeline 0=field, 1=Drive2Tag)│
│  PathPlanner: 2026.1.2                                       │
│  Phoenix 6: v26.1.1                                          │
└──────────────────────────────────────────────────────────────┘
```

---

## 二、硬體清單 & CAN ID 對照

### DRIVETRAIN CAN Bus
| 元件 | CAN ID | 類型 |
|------|--------|------|
| Left Front Throttle | 4 | TalonFX (Kraken X60) |
| Left Front Rotor | 3 | TalonFX (Kraken X60) |
| Left Front CANcoder | 12 | CANcoder |
| Right Front Throttle | 6 | TalonFX (Kraken X60) |
| Right Front Rotor | 5 | TalonFX (Kraken X60) |
| Right Front CANcoder | 13 | CANcoder |
| Left Rear Throttle | 2 | TalonFX (Kraken X60) |
| Left Rear Rotor | 1 | TalonFX (Kraken X60) |
| Left Rear CANcoder | 11 | CANcoder |
| Right Rear Throttle | 8 | TalonFX (Kraken X60) |
| Right Rear Rotor | 7 | TalonFX (Kraken X60) |
| Right Rear CANcoder | 14 | CANcoder |

### Default CAN Bus
| 元件 | CAN ID | 類型 | 備註 |
|------|--------|------|------|
| Pigeon2 IMU | 0 | Pigeon2 | Yaw 100Hz |
| Shooter Leader | 22 | TalonFX | VelocityVoltage |
| Shooter Follower | 21 | TalonFX | Follower (Opposed) |
| Transport | 30 | TalonFX | VelocityVoltage, Coast |
| Up-to-Shoot | 26 | TalonFX | VelocityVoltage, Coast |
| IntakeRoller Leader | 29 | TalonFX | VelocityVoltage, Brake |
| IntakeRoller Follower | 35 | TalonFX | Follower (Opposed) |
| IntakeArm Leader | 3 | TalonFX | ⚠️ 與 Swerve Rotor 衝突 |
| IntakeArm Follower | 4 | TalonFX | ⚠️ 與 Swerve Throttle 衝突 |

> ⚠️ **IntakeArm CAN ID 3/4 與 DRIVETRAIN bus 上的 Swerve Rotor/Throttle 衝突**。  
> 目前 IntakeArm 在 RobotContainer 中已被註解掉，不會實際產生衝突。  
> 若要啟用，必須更換 CAN ID 或將 IntakeArm 移到 DRIVETRAIN bus。

---

## 三、底盤 (Swerve) 子系統

### 3.1 機構參數
| 參數 | 值 | 說明 |
|------|-----|------|
| 模組型號 | MK4i | SDS (Swerve Drive Specialties) |
| 齒輪比 | L3 (6.12:1) | Very Fast |
| 輪徑 | 0.1m (4 inch) | 含胎皮壓縮 |
| 軸距/輪距 | 0.62865m | 正方形底盤 |
| 理論最大速度 | 5.13 m/s | `(6000/60/6.12) × 0.1 × π` |
| 加速度限制 | 20 m/s² (平移) / 15 rad/s² (旋轉) | SlewRateLimiter |

### 3.2 感測器融合架構
```
                 ┌─────────────┐
                 │   Pigeon2   │  Yaw: 100Hz, AngVel: 50Hz
                 │   IMU       │  optimizeBusUtilization()
                 └──────┬──────┘
                        │
          ┌─────────────┴──────────────┐
          │  SwerveDrivePoseEstimator  │  ← 主要位姿來源
          │  (WPILib 卡爾曼濾波器)     │
          │  wheel stddev: 0.05m/5°    │
          └──────┬──────────┬──────────┘
                 │          │
    ┌────────────┘          └────────────┐
    │ 底盤里程計                          │ 視覺融合
    │ 4× SwerveModule                    │ Limelight MegaTag2
    │ CANcoder 100Hz                     │ getBotPoseEstimate_wpiBlue()
    │ Throttle pos/vel 100Hz             │
    │                                    │ 過濾條件:
    │                                    │ - tagCount=1: ambiguity>0.7 → 拒絕
    │                                    │ - tagCount=1: dist>4m → 拒絕
    │                                    │ - gyroRate>720°/s → 拒絕
    │                                    │ 
    │                                    │ 動態信任度:
    │                                    │ - ≥2 Tags: xyStdDev=0.3
    │                                    │ - 1 Tag: 0.5+(dist×0.15)
    │                                    │ - 角度: 999999 (不融合)
    └────────────────────────────────────┘
```

### 3.3 控制流程
```
ManualDrive.execute()
  │  搖桿 → deadzone → multiplier → boost(0.5/1.0)
  │  射擊模式: z=0, xy×0.3
  │  × kMaxPhysicalSpeedMps → m/s
  └→ Swerve.setSpeed(x,y,z,fieldOriented)
       │  紅方: x,y 反轉
       │  → fromFieldRelativeSpeeds()
       └→ mTargetChassisSpeeds

AutoAimAndShoot.execute()
  └→ Swerve.setAimSpeed(ChassisSpeeds)
       └→ mAimChassisSpeeds (旋轉疊加)

Swerve.periodic()
  │  odometry update
  │  vision fusion
  └→ runSetStates()
       │  sumChassisSpeeds = mTarget + mAim
       │  SlewRateLimiter (加速度限制)
       │  toSwerveModuleStates()
       │  desaturateWheelSpeeds()
       └→ setModuleStates() → 4× SwerveModule.setState()
```

---

## 四、射手 (Shooter) 子系統

### 4.1 架構
- **Leader** (CAN 22): VelocityVoltage 閉環控制
- **Follower** (CAN 21): `Follower(leader, MotorAlignmentValue.Opposed)` — 自動反轉
- **NeutralMode**: Coast（飛輪慣性，不煞車）
- **StatusSignal 快取**: velocity 50Hz, voltage/current 10Hz

### 4.2 PID 參數 (Slot 0)
| 參數 | 預設值 | 可調？ |
|------|--------|--------|
| kV | 0.12 | ✅ TunableNumber |
| kP | 0.12 | ✅ TunableNumber |
| kI | 0.0 | ✅ TunableNumber |
| kD | 0.0 | ✅ TunableNumber |
| kS | 0.0 | ✅ TunableNumber |

### 4.3 距離自適應射擊 (interpolateRps)
```
距離(m) → RPS 線性內插查表:
  1.0m → 35 RPS
  1.5m → 40 RPS
  2.0m → 45 RPS
  2.5m → 50 RPS
  3.0m → 55 RPS
  3.5m → 60 RPS
  4.0m → 65 RPS
  5.0m → 70 RPS
  中立區 → 固定 70 RPS (kMidFieldReturnRps)

距離 < 1m: clamp 35 RPS
距離 > 5m: clamp 70 RPS
中間值: 線性內插
```

---

## 五、輸送帶 (Transport) 子系統

### 5.1 架構
- **Transport** (CAN 30): 輸送帶，`InvertedValue.CounterClockwise_Positive`
- **Up-to-Shoot** (CAN 26): 上膛推球，`InvertedValue.Clockwise_Positive`
- **兩顆獨立控制**（非 Leader/Follower），各有獨立的 `VelocityVoltage` 請求物件
- **NeutralMode**: Coast（兩顆都是）
- **電流限制**: Stator 60A, Supply 40A

### 5.2 速度設定
| 模式 | Transport RPS | Up-to-Shoot RPS |
|------|--------------|-----------------|
| 正常送球 | 35 | 80 |
| 慢速 (Intake) | 18 | 18 |
| 反轉吐球 | -35 | -80 |

### 5.3 關鍵修正：獨立 VelocityVoltage 物件
```java
// ❌ 修正前：共用同一個 mutable 物件
private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
up_to_shoot.setControl(velocityRequest.withVelocity(shootRps));   // 改了 velocityRequest
transport.setControl(velocityRequest.withVelocity(transportRps)); // 又改了 velocityRequest

// ✅ 修正後：各自獨立
private final VelocityVoltage upToShootRequest = new VelocityVoltage(0);
private final VelocityVoltage transportRequest = new VelocityVoltage(0);
```

---

## 六、Intake 滾輪 (IntakeRoller) 子系統

### 6.1 架構
- **Leader** (CAN 29): VelocityVoltage 閉環控制
- **Follower** (CAN 35): `Follower(leader, MotorAlignmentValue.Opposed)` — 自動反轉
- **NeutralMode**: Brake（停止時立刻煞車防球滑出）
- **StatusSignal 快取**: velocity 50Hz, voltage/current 10Hz

### 6.2 速度設定
| 模式 | 目標 RPS |
|------|---------|
| 吸球 (Intake) | +60 |
| 吐球 (Outtake) | -30 |

### 6.3 關鍵修正：移除 Follower 的 Inverted 設定
```java
// ❌ 修正前：雙重反轉（Opposed + Clockwise_Positive → 互相抵消 → 同向轉）
followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
followerMotor.setControl(new Follower(leader, MotorAlignmentValue.Opposed));

// ✅ 修正後：只靠 Opposed 自動反轉
// 不設定 Inverted，由 Follower(Opposed) 處理
followerMotor.setControl(new Follower(leader, MotorAlignmentValue.Opposed));
```

---

## 七、Intake 手臂 (IntakeArm) 子系統

> ⚠️ **目前在 RobotContainer 中已被註解掉，不會實際運行**

- **Leader** (CAN 3) + **Follower** (CAN 4): PositionVoltage + 重力補償 (Arm_Cosine)
- **NeutralMode**: Brake（抗重力必須）
- **軟體限位**: 0~8 圈
- **齒輪比**: 20:1
- **PID**: kP=2.0, kI=0, kD=0.1, kG=0.1

---

## 八、Commands 詳解

### 8.1 ManualDrive (Default Command)
```
addRequirements: Swerve
觸發: 預設命令，持續運行
功能:
  - 左搖桿: 平移 (X/Y)
  - 右搖桿: 旋轉 (Z)
  - Right Bumper: Boost (0.5x → 1.0x)
  - Right Stick Click: 切換場地導向/機器人導向
  - 射擊模式 (由 AutoAimAndShoot 啟用):
    - Z 歸零（旋轉交給 AutoAim PID）
    - XY × 0.3（降速避免慣性射偏）
```

### 8.2 AutoAimAndShoot
```
addRequirements: Shooter, Transport
不 require Swerve（透過 setAimSpeed 疊加旋轉）
觸發: Left Bumper (whileTrue) 或 Right Trigger (whileTrue)
                                    
Pipeline:                           
  1. getPose() → 判斷紅/藍聯盟     
  2. isInOwnZone(robotX, isRed)     
     ├─ 己方: getHubPosition(isRed) → atan2 → 瞄準角度
     └─ 中立: getReturnAngleRad(isRed) → 固定方向
  3. + kShooterAngleOffsetRad (π, 射手在背面)
  4. PID 旋轉: kP=5.0, kI=0, kD=0.1
  5. 距離 → interpolateRps() → setTargetVelocity()
  6. 遲滯送球邏輯:
     - 首次: 角度 ≤2° + 達速 → 開始送球
     - 已送: 角度 ≤5° (寬鬆) 保持連射
  7. ManualDrive.setShootingMode(true) → 降速+鎖旋轉
```

### 8.3 Drive2Tag
```
addRequirements: Swerve
觸發: A button (whileTrue)
Pipeline: Limelight → pipeline 1 → targetPose_RobotSpace → 3-axis PID → setSpeed
互斥: 會額外 require Shooter+Transport → 與 AutoAimAndShoot 自動互斥
```

### 8.4 PathPlanner Auto Commands
| 名稱 | 功能 |
|------|------|
| `Auto Shoot` | 自適應射擊（距離→RPS→達速→送球），4s timeout |
| `Far Auto Shoot` | 同 Auto Shoot（已統一為自適應） |
| `Auto Intake` | IntakeRoller + 慢速 Transport，2.5s timeout |
| `shoot work` | 僅啟動射手（不送球），用於預熱 |
| `transport wait shoot` | 等達速 → 送球 |
| `Start Intake` / `Stop Intake` | 手動控制 Intake |

---

## 九、性能優化清單

### 9.1 遙測節流 (Telemetry Throttle)
```
原始: 50Hz × 每個子系統 ~10 個 NT 寫入 = ~350 寫入/秒
優化: kTelemetryDivider = 5 → 10Hz → ~70 寫入/秒 (↓80%)

已套用的檔案:
  ✅ ShooterSubsystem    (5 個 entry)
  ✅ IntakeRollerSubsystem (5 個 entry)
  ✅ IntakeArmSubsystem   (4 個 entry)
  ✅ Swerve.java          (telemetryThisCycle flag, 跨 periodic/runSetStates/setModuleStates)
  ✅ ManualDrive.java     (6 個 entry)
  ✅ AutoAimAndShoot.java (10 個 entry)
  ✅ Robot.java           (3 個 entry)
```

### 9.2 Phoenix 6 StatusSignal 快取 & 頻率控制
```
原始: getVelocity().getValueAsDouble() → 每次呼叫都重新查找 signal，全部預設 250Hz
優化: 快取 StatusSignal<T> 欄位 + setUpdateFrequency() + optimizeBusUtilization()

預估 CAN 頻寬節省: ~5750 frames/s → ~1150 frames/s (↓80%)

各元件頻率設定:
┌──────────────────────┬─────────────────────┬──────────────┐
│ 元件                  │ Signal              │ 頻率 (Hz)    │
├──────────────────────┼─────────────────────┼──────────────┤
│ Pigeon2 IMU          │ Yaw                 │ 100          │
│                      │ AngularVelocityZ    │ 50           │
├──────────────────────┼─────────────────────┼──────────────┤
│ SwerveModuleKraken   │ throttlePosition    │ 100          │
│ (×4)                 │ throttleVelocity    │ 100          │
├──────────────────────┼─────────────────────┼──────────────┤
│ SwerveModule/CANcoder│ absolutePosition    │ 100          │
│ (×4)                 │                     │              │
├──────────────────────┼─────────────────────┼──────────────┤
│ ShooterSubsystem     │ velocity            │ 50           │
│                      │ motorVoltage        │ 10           │
│                      │ statorCurrent       │ 10           │
├──────────────────────┼─────────────────────┼──────────────┤
│ IntakeRollerSubsystem│ velocity            │ 50           │
│                      │ motorVoltage        │ 10           │
│                      │ statorCurrent       │ 10           │
├──────────────────────┼─────────────────────┼──────────────┤
│ IntakeArmSubsystem   │ position            │ 50           │
│                      │ motorVoltage        │ 10           │
│                      │ statorCurrent       │ 10           │
└──────────────────────┴─────────────────────┴──────────────┘

所有裝置都呼叫了 optimizeBusUtilization()，
將未使用的 Signal 自動降頻至 4Hz 或更低。
```

### 9.3 聯盟對稱重構 (Alliance Symmetry)
```
原始: Constants 中維護 kBlueHubX/Y + kRedHubX/Y + kBlueReturnAngleRad + kRedReturnAngleRad
     AutoAimAndShoot 和 RobotContainer 中大量 if(isRed)/else 分支

優化: 
  - 只保留藍方基準常數: kHubX, kHubY, kReturnAngleRad
  - 新增 3 個 static helper 方法:
    - getHubPosition(isRed) → 紅方自動 fieldLength - x 翻轉
    - getReturnAngleRad(isRed) → 紅方自動 π - θ 翻轉
    - isInOwnZone(robotX, isRed) → 統一 zone 判斷
  - AutoAimAndShoot: 40 行 if/else → 15 行統一邏輯
  - RobotContainer.getAdaptiveRps(): 5 行 → 1 行

效果: 未來調 Hub 座標只需改一處，紅方自動推導，消除「改藍忘紅」風險。
```

### 9.4 Follower 馬達修正
```
修正項目:
  1. IntakeRoller: 移除 Follower 的 Inverted = Clockwise_Positive
     (Opposed 已自動反轉，手動設定會造成雙重反轉)
  
  2. IntakeRoller + IntakeArm: 移除 periodic 中對 Follower 的 PID 更新
     (Follower 鏡像 Leader 輸出，不跑自己的 PID，寫入 Slot0 無意義且浪費 CAN)
  
  3. Transport: 分離 VelocityVoltage 請求物件
     (原本兩顆馬達共用一個 mutable 物件，withVelocity() 會原地修改)
```

---

## 十、場地常數 (AutoAimConstants)

### 10.1 座標系統
- **WPILib 藍方原點**：場地左下角 (0,0)
- **X 軸**：場地長邊 (0 → 16.541m)
- **Y 軸**：場地短邊 (0 → 8.069m)
- **角度**：0° = +X (場地右)，90° = +Y (場地上)，180° = -X (場地左)

### 10.2 Hub 座標
| 常數 | 值 | 推導 |
|------|-----|------|
| kHubX | 4.626m | 藍方 Hub X (藍方基準) |
| kHubY | 4.035m | Hub Y (紅藍相同) |
| 紅方 Hub X | 11.915m | `16.541 - 4.626` (自動計算) |
| kFieldMidX | 8.2705m | `16.541 / 2` (中場線) |
| kReturnAngleRad | π | 藍方回程角度 (紅方自動 π-π = 0) |

### 10.3 射擊策略
| 區域 | 判斷條件 | 行為 |
|------|----------|------|
| 己方場域 | Blue: X < 8.27m / Red: X > 8.27m | 瞄準 Hub + 距離自適應 RPS |
| 中立/對方 | 其餘 | 朝己方聯盟區方向射回 + 固定 70 RPS |

---

## 十一、操作者控制映射

### Xbox Controller (Port 0)
| 按鈕/軸 | 功能 | 備註 |
|---------|------|------|
| 左搖桿 | 平移 (X/Y) | Boost 模式下 100% 速度 |
| 右搖桿 X | 旋轉 | 射擊模式下歸零 |
| Right Bumper | Boost (0.5x → 1.0x) | 按住加速 |
| Left Bumper | AutoAimAndShoot | 按住：自動瞄準+射擊 |
| Right Trigger | AutoAimAndShoot | 同 LB（備用綁定） |
| Left Trigger | Intake 吸球 | 按住運轉 |
| A | Drive2Tag 對位 | 按住：AprilTag 自動導航 |
| B | Transport 反轉 | 按住：吐球 |
| X | Transport 送球 | 按住：手動送球 |
| Menu (Button 8) | 重設 IMU | 歸零航向 |
| Right Stick Click | 切換場地/機器人導向 | 切換模式 |

### Teleop 進入時自動行為
1. Limelight 自動校正 XY 位置 (`resetPoseToLimelight`)
2. 手把雙次震動提示 (0.3s × 2)

---

## 十二、已知問題 & 待辦事項

### 🔴 高優先

| # | 問題 | 位置 | 說明 |
|---|------|------|------|
| 1 | IntakeArm CAN ID 衝突 | Constants.java | CAN 3/4 與 Swerve Rotor/Throttle 衝突。目前未啟用。 |
| 2 | kShooterAngleOffsetRad 待確認 | Constants.java | 目前假設射手在背面 (π)，需在實際機器人確認 |
| 3 | 距離→RPS 查表待校準 | Constants.java | kDistanceToRpsTable 需要實測調整 |

### 🟡 中優先

| # | 問題 | 位置 | 說明 |
|---|------|------|------|
| 4 | setChassisSpeeds 魔術數字 | Swerve.java | `×0.185` / `×0.21` 縮放，疑為 dead code |
| 5 | ShooterSubsystem Follower apply(config) | ShooterSubsystem.java | 初始化時 PID 對 Follower 無意義（但 NeutralMode 需保留） |
| 6 | IntakeRoller Follower 初始化仍有 Slot0 | IntakeRollerSubsystem.java | PID 對 Follower 無效，但 CurrentLimits/NeutralMode 需保留 |
| 7 | Phoenix 6 deprecation warnings | SwerveModule/Kraken | `CANcoder(int,String)` 和 `TalonFX(int,String)` 已棄用 |
| 8 | 多處未使用的 import | 多個檔案 | Pose2d, Rotation2d, Units, Logger 等 |

### 🟢 低優先

| # | 問題 | 位置 | 說明 |
|---|------|------|------|
| 9 | SwerveModuleNeo 使用已棄用 API | SwerveModuleNeo.java | ResetMode, PersistMode, configure() 均已棄用 |
| 10 | Drive2Tag 無 isFinished | Drive2Tag.java | 永遠不會自動結束，依靠 whileTrue 放開按鈕 |
| 11 | putLimeLight() 全部註解 | RobotContainer.java | 可移除整個方法 |

---

## 十三、依賴版本

| 依賴 | 版本 |
|------|------|
| WPILib | 2026 |
| GradleRIO | 2026.2.1 |
| Phoenix 6 (CTRE) | 26.1.1 |
| REVLib | 2026.x |
| PathPlanner | 2026.1.2 |
| Gradle Wrapper | 8.11.1 |
| JDK | WPILib 2026 JDK |

---

## 十四、檔案結構

```
src/main/java/frc/robot/
├── Main.java                    # 程式進入點
├── Robot.java                   # TimedRobot 主類別，Loop 計時監控
├── RobotContainer.java          # 綁定按鈕、AutoBuilder、ShuffleboardManager
├── Constants.java               # 所有常數集中管理（含 helper 方法）
├── LimelightHelpers.java        # Limelight API 封裝
│
├── commands/
│   ├── ManualDrive.java         # 手動駕駛（含射擊模式）
│   ├── AutoAimAndShoot.java     # 自動瞄準射擊（距離自適應）
│   └── Drive2Tag.java           # AprilTag 自動對位
│
└── subsystems/
    ├── Swerve.java              # 4 模組 Swerve Drive + 視覺融合
    ├── SwerveModule.java        # 抽象 Swerve 模組（CANcoder + PID）
    ├── SwerveModuleKraken.java  # TalonFX (Kraken) 實作
    ├── SwerveModuleNeo.java     # SparkMax (NEO) 實作（備用）
    ├── ShooterSubsystem.java    # 雙馬達射手（Leader+Follower）
    ├── TransportSubsystem.java  # 雙馬達輸送帶（獨立控制）
    ├── IntakeRollerSubsystem.java # 雙馬達 Intake 滾輪
    ├── IntakeArmSubsystem.java  # 雙馬達 Intake 手臂（未啟用）
    ├── LightPollution.java      # LED 燈條
    └── RoboArm.java             # 機械臂（未使用）
```
