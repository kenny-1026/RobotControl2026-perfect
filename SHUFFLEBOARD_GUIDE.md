# 🤖 Simulate Robot Code + Shuffleboard 使用教學

> **適用版本**：WPILib 2026、VS Code + WPILib Extension  
> **最後更新**：2026-03-08（依據 Fix 17 後程式碼全面更新）

---

## 目錄

1. [環境準備](#1-環境準備)
2. [啟動模擬器](#2-啟動模擬器-simulate-robot-code)
3. [啟動 Shuffleboard](#3-啟動-shuffleboard)
4. [連線確認](#4-連線確認)
5. [各分頁功能說明](#5-各分頁功能說明)
6. [即時修改 PID 參數](#6-即時修改-pid-參數)
7. [Field2d 場地地圖](#7-field2d-場地地圖)
8. [常見問題](#8-常見問題)
9. [🔫 Shooter 完整系統架構與參數調整指南](#9--shooter-完整系統架構與參數調整指南)

---

## 1. 環境準備

確認以下軟體已安裝：

- ✅ **VS Code** + **WPILib 2026 Extension**
- ✅ 專案可以成功執行 `.\gradlew.bat build`（Build Successful）

---

## 2. 啟動模擬器 (Simulate Robot Code)

### 步驟

**① 開啟指令面板**

按下鍵盤快捷鍵：
```
Ctrl + Shift + P
```

**② 搜尋並執行**

輸入以下指令並按 Enter：
```
WPILib: Simulate Robot Code
```

> 💡 也可以點選 VS Code 右上角的 **WPILib 圖示（W）**，在選單中找到 `Simulate Robot Code`

**③ 選擇模擬模式（彈出視窗）**

出現視窗後，勾選以下選項後按 **OK**：

| 選項 | 說明 |
|------|------|
| ✅ `halsim_gui` | 開啟 **Glass** 視窗（DS 面板 + 感測器模擬） |

**④ 等待 Glass 視窗出現**

程式編譯完成後，會自動跳出 **Glass（SimGUI）** 視窗。

### Glass 視窗操作

```
Robot State 面板：
  ┌──────────────────────────┐
  │  Disabled  │  Teleop  │  │  ← 點選 "Teleop" 讓機器人進入遙控模式
  │  Auto      │  Test    │  │
  └──────────────────────────┘
```

> ⚠️ **必須切換到 Teleop 或 Auto 模式**，PID 控制和馬達指令才會執行

---

## 3. 啟動 Shuffleboard

**① 開啟指令面板**
```
Ctrl + Shift + P
```

**② 搜尋並執行**
```
WPILib: Start Tool
```

**③ 選擇工具**

在下拉選單中選擇：
```
Shuffleboard
```

> 💡 Shuffleboard 需要在模擬器**已啟動**的狀態下開啟，否則無法自動連線

---

## 4. 連線確認

Shuffleboard 開啟後，檢查左上角狀態：

```
🟢 Connected to localhost   ← 模擬器連線成功
🔴 Disconnected             ← 尚未連線，等待或重新啟動
```

### 手動設定連線位址（如需要）

```
File → Preferences → Server → 
  模擬器填入: localhost
  實體機器人填入: 10.TE.AM.2  (例如隊號1234 → 10.12.34.2)
```

---

## 5. 各分頁功能說明

本專案共有 **6 個分頁**（由 `ShuffleboardManager.java` 統一管理），點選 Shuffleboard 上方標籤切換：

> **注意**：`IntakeArmSubsystem` 目前在 `RobotContainer` 中已被註解掉，IntakeArm PID Tab 不會有即時資料。如需使用，取消 `RobotContainer.java` 中 IntakeArm 那行的註解即可。

### 🏠 Main Tab

| Widget | 類型 | 說明 |
|--------|------|------|
| **Field** | Field2d | 場地地圖，機器人位置即時顯示（Limelight + IMU 融合推算） |
| **Auto Mode** | ComboBoxChooser | 下拉選擇自動模式（PathPlanner Auto） |
| **Loop ms** | Graph | 機器人每圈執行時間波形圖，正常應 < 20ms |
| **Loop Max ms** | TextView | 歷史最大迴圈時間（ms） |
| **Overrun?** | BooleanBox | 🟢 正常 / 🔴 超過 20ms 警告 |

---

### 🔫 Shooter PID Tab

> 來源：`ShooterSubsystem.java`，馬達 ID：Leader=22, Follower=21

| Widget | 類型 | 可編輯 | 說明 |
|--------|------|--------|------|
| `kV` | Number Slider | ✅ | 前饋速度增益（預設 0.12） |
| `kP` | Number Slider | ✅ | 比例增益（預設 0.12） |
| `kI` | Number Slider | ✅ | 積分增益（預設 0.0） |
| `kD` | Number Slider | ✅ | 微分增益（預設 0.0） |
| `kS` | Number Slider | ✅ | 靜摩擦補償（預設 0.0） |
| `Current RPS` | Graph | ❌ | 目前馬達轉速波形 |
| `Target RPS` | TextView | ❌ | 目標轉速 |
| `Error RPS` | TextView | ❌ | 誤差值（Target - Current） |
| `Output V` | Graph | ❌ | 輸出電壓波形 |
| `Stator A` | TextView | ❌ | 定子電流 |

---

### 🦾 IntakeArm PID Tab

> 來源：`IntakeArmSubsystem.java`，馬達 ID：Leader=32, Follower=33  
> ⚠️ **目前已停用**（`RobotContainer` 中 IntakeArm 初始化被註解）

| Widget | 類型 | 可編輯 | 說明 |
|--------|------|--------|------|
| `kP` | Number Slider | ✅ | 比例增益（預設 2.0） |
| `kI` | Number Slider | ✅ | 積分增益（預設 0.0） |
| `kD` | Number Slider | ✅ | 微分增益（預設 0.1） |
| `kG` | Number Slider | ✅ | 重力補償增益（預設 0.1） |
| `Arm Rotations` | Graph | ❌ | 手臂圈數（換算後） |
| `Motor Rotations` | TextView | ❌ | 馬達圈數（原始值） |
| `Output V` | Graph | ❌ | 輸出電壓 |
| `Stator A` | TextView | ❌ | 電流 |

---

### 🔄 IntakeRoller PID Tab

> 來源：`IntakeRollerSubsystem.java`，馬達 ID：Leader=29, Follower=35

| Widget | 類型 | 可編輯 | 說明 |
|--------|------|--------|------|
| `kV` | Number Slider | ✅ | 前饋速度增益（預設 0.12） |
| `kP` | Number Slider | ✅ | 比例增益（預設 0.2） |
| `kI` | Number Slider | ✅ | 積分增益（預設 0.01） |
| `kD` | Number Slider | ✅ | 微分增益（預設 0.0） |
| `Actual RPS` | Graph | ❌ | 實際轉速波形 |
| `Target RPS` | TextView | ❌ | 目標轉速 |
| `Error RPS` | TextView | ❌ | 誤差值 |
| `Output V` | Graph | ❌ | 輸出電壓 |
| `Leader A` | TextView | ❌ | 主馬達電流 |

---

### 🚗 Swerve Tab

> 來源：`Swerve.java`，IMU：Pigeon2 ID=0，底盤：MK4i L3 Kraken X60

| Widget | 類型 | 說明 |
|--------|------|------|
| `Chassis vx` | Graph | X 方向速度波形（m/s） |
| `Chassis vy` | Graph | Y 方向速度波形（m/s） |
| `Chassis omega` | Graph | 旋轉速度波形（rad/s） |
| `Gyro Angle` | TextView | 陀螺儀當前角度（度） |
| `Mod0 Speed` | TextView | 左前模組速度（m/s） |
| `Mod0 Angle` | TextView | 左前模組角度（度） |
| `Vision Tags` | TextView | Limelight 偵測到的 AprilTag 數量 |
| `Vision Rejected` | BooleanBox | 🟢 使用視覺 / 🔴 拒絕更新（模糊/太遠/高速旋轉） |
| `Vision X` | TextView | Limelight 估算的 X 座標 |
| `Vision Y` | TextView | Limelight 估算的 Y 座標 |

---

### 🎯 AutoAim Tab

> 來源：`AutoAimAndShoot.java`，按住 Left Bumper 或 Right Trigger 時顯示資料

| Widget | 類型 | 可編輯 | 說明 |
|--------|------|--------|------|
| `Rotation kP` | Number Slider | ✅ | 旋轉 PID 比例增益（預設 5.0） |
| `Rotation kI` | Number Slider | ✅ | 旋轉 PID 積分增益（預設 0.0） |
| `Rotation kD` | Number Slider | ✅ | 旋轉 PID 微分增益（預設 0.1） |
| `Distance` | TextView | ❌ | 機器人到目標距離（m） |
| `Target Angle` | TextView | ❌ | 目標角度（度） |
| `Current Angle` | TextView | ❌ | 機器人當前角度（度） |
| `Angle Error` | Graph | ❌ | 角度誤差波形（度） |
| `Rot Output` | Graph | ❌ | PID 旋轉輸出波形 |
| `Target RPS` | TextView | ❌ | 目標射手轉速 |
| `Current RPS` | TextView | ❌ | 實際射手轉速 |
| `Aligned?` | BooleanBox | ❌ | 🟢 已對齊 / 🔴 未對齊 |
| `At Speed?` | BooleanBox | ❌ | 🟢 達速 / 🔴 未達速 |
| `Feeding?` | BooleanBox | ❌ | 🟢 正在送球 / 🔴 等待中 |

> 💡 只有按住 Left Bumper（帶 Tab 的版本）才會更新 Shuffleboard 上的 AutoAim Widget。  
> Right Trigger 的 `AutoAimAndShoot` 沒有傳入 Tab，會改用 SmartDashboard 輸出。

---

## 6. 即時修改 PID 參數

這是本系統最核心的功能，可以**不重新部署**直接調整 PID。

### 操作步驟

```
① 切換到對應的 PID Tab（例如 "Shooter PID"）

② 找到要修改的參數輸入框（例如 kP）

③ 直接點擊數字框 → 輸入新數值 → 按 Enter

④ 程式在下一個 20ms 週期自動套用到馬達控制器

⑤ 觀察波形圖，確認響應變化
```

### 調參流程建議

```
1. 先把 kI、kD 設為 0，只調 kV (前饋) 和 kP
2. 計算 kV：12V / 空載最大 RPS（例如 95 RPS → kV ≈ 0.126）
3. 慢慢增加 kP，直到剛好有輕微震盪
4. 加入 kD 抑制震盪
5. 最後微調 kI 消除穩態誤差
6. 確認效果後，把數值寫回 Constants.java！
```

> ⚠️ **重要提醒**：重新部署程式碼或重開模擬器後，Shuffleboard 上的修改會**消失**，一定要手動更新原始碼！

### 寫回 Constants.java 的位置

| 參數 | 常數位置 |
|------|---------|
| Shooter kV/kP/kI/kD/kS | `Constants.java` → `ShooterConstants.kDefaultKV / kP / kI / kD / kS` |
| IntakeRoller kV/kP/kI/kD | `Constants.java` → `IntakeRollerConstants.kDefaultKV / kP / kI / kD` |
| IntakeArm kP/kI/kD/kG | `Constants.java` → `IntakeArmConstants.kDefaultKP / kI / kD / kG` |
| AutoAim Rotation kP/kI/kD | `Constants.java` → `AutoAimConstants.kRotation_kP / kI / kD` |
| RPS 多項式係數 | `Constants.java` → `AutoAimConstants.kRpsA / kRpsB / kRpsC` |
| 射擊模式速度倍率 | `Constants.java` → `AutoAimConstants.kShootingModeSpeedMultiplier` |
| 射手容許誤差 | `Constants.java` → `AutoAimConstants.kShooterToleranceRps` |
| 角度容許誤差（首次觸發） | `Constants.java` → `AutoAimConstants.kRotationToleranceDeg` |
| 遲滯角度（連射保持） | `Constants.java` → `AutoAimConstants.kFeedingHysteresisDeg` |
| Hub 座標 | `Constants.java` → `AutoAimConstants.kHubX / kHubY`（單一藍方座標，紅方自動鏡像） |
| ManualDrive 倍率/死區 | `Constants.java` → `ManualDriveConstants.kXMultiplier / kYMultiplier / kZMultiplier / kXDeadzone ...` |

---

## 7. Field2d 場地地圖

### 在 Main Tab 查看

切換到 **Main** 分頁後，`Field` Widget 會顯示：

```
┌─────────────────────────────────────────┐
│                                         │
│    [藍方]              [紅方]            │
│                                         │
│           ▶ 機器人圖示                   │  ← 箭頭方向 = 機器人朝向
│                                         │
└─────────────────────────────────────────┘
```

### 說明

- **機器人圖示位置** = 由 Limelight MegaTag2 + Pigeon2 IMU 融合推算的估計位置
- **機器人箭頭方向** = 陀螺儀角度
- 模擬模式下，位置從 `(0,0)` 開始，需要手動控制才會移動

### 讓機器人在地圖上移動（模擬模式）

```
1. Glass 視窗 → 切換到 Teleop
2. Glass → Joysticks 面板 → 設定模擬搖桿輸入
   - 或使用真實手把（Xbox Controller 插入電腦）
3. 在 Shuffleboard Main Tab 觀察機器人在地圖上移動
```

---

## 8. 常見問題

### Q: Shuffleboard 顯示 Disconnected？
```
✅ 確認模擬器已啟動（Glass 視窗有出現）
✅ File → Preferences → Server 設定為 localhost
✅ 重新啟動 Shuffleboard
```

### Q: 找不到我的 PID 參數 Widget？
```
✅ 左側 Sources 面板 → 展開 Shuffleboard → 找到對應 Tab
✅ 把數值欄位拖曳到畫面上
✅ 或等待幾秒，Widget 有時需要時間出現
```

### Q: 修改 kP 之後沒有反應？
```
✅ 確認機器人在 Teleop 模式（不是 Disabled）
✅ 確認有啟動對應的 Command（例如按住按鈕）
✅ 觀察波形圖，確認數值有更新
```

### Q: 波形圖沒有顯示數據？
```
✅ 右鍵 Widget → Edit Properties → 確認資料來源正確
✅ 確認馬達 Command 有在執行中
✅ 模擬模式下部分感測器值為 0 屬正常
```

### Q: 每次重開都要重新排版？
```
✅ Shuffleboard → File → Save Layout → 存成 .json 檔
✅ 下次開啟 → File → Load Layout
```

### Q: IntakeArm PID Tab 沒有資料？
```
✅ IntakeArm 目前在 RobotContainer.java 中被註解
✅ 取消 intakeArm 那行的註解，重新 build 即可
```

---

## 9. 🔫 Shooter 完整系統架構與參數調整指南

### 9.1 系統架構概覽

```
┌───────────────────────────────────────────────────────────────┐
│                        操作者控制流程                           │
│                                                               │
│  ┌─────────────┐    ┌──────────────────┐    ┌──────────────┐ │
│  │  Xbox 手把   │───▶│   ManualDrive    │───▶│    Swerve    │ │
│  │  左搖桿=平移 │    │  setSpeed(x,y,z) │    │ 底盤移動控制  │ │
│  │  右搖桿=旋轉 │    │                  │    │              │ │
│  └─────────────┘    └──────────────────┘    └──────────────┘ │
│                                                    ▲          │
│  ┌─────────────────────────────────────────────────┤          │
│  │   按住 Left Bumper 或 Right Trigger 時          │          │
│  │  ┌──────────────────────┐                       │          │
│  │  │  AutoAimAndShoot     │  setAimSpeed(旋轉PID) │          │
│  │  │                      │───────────────────────┘          │
│  │  │  ① 判斷區域(己方/中場) │                                  │
│  │  │  ② 選擇目標(Hub/     │                                  │
│  │  │     回傳點)           │                                  │
│  │  │  ③ PID 控制底盤旋轉   │                                  │
│  │  │  ④ 依距離多項式曲線   │                                  │
│  │  │     計算射手 RPS      │                                  │
│  │  │  ⑤ 達速+對準→自動送球  │                                  │
│  │  └──────┬──────┬────────┘                                  │
│  │         │      │                                           │
│  │         ▼      ▼                                           │
│  │   ┌──────────┐ ┌───────────────┐                           │
│  │   │ Shooter  │ │  Transport    │                           │
│  │   │ 射手馬達  │ │  送球+上膛     │                           │
│  │   │ ID 22+21 │ │  ID 30+26     │                           │
│  │   └──────────┘ └───────────────┘                           │
│  └────────────────────────────────────────────────────────────│
└───────────────────────────────────────────────────────────────┘
```

### 9.2 射擊模式 (Shooting Mode)

**按住 Left Bumper 或 Right Trigger** 啟動 `AutoAimAndShoot` 時，`ManualDrive` 自動進入射擊模式：

| 行為 | 正常模式 | 射擊模式 |
|------|---------|---------|
| 左搖桿平移 | 100%（按 Right Bumper Boost）/ 50%（預設） | **×0.3** (30%)，避免慣性射偏 |
| 右搖桿旋轉 | 正常操控 | **鎖定為 0**（由 AutoAim PID 全速控制） |
| Boost (Right Bumper) | 2× 加速 | 仍可使用（但效果 ×0.3） |
| 底盤旋轉速度 | 手動控制 | **不限速**，PID 輸出直接驅動底盤旋轉 |

> 放開按鈕後自動恢復正常操控。

### 9.3 自動瞄準流程 (AutoAimAndShoot)

每個 20ms 週期執行以下步驟：

```
 1. 取得機器人目前位置 (Swerve.getPose())
 2. 判斷區域：
    ├── 己方聯盟區 → 計算面向 Hub 的角度 (atan2) + 射手安裝偏移
    └── 中立區（中場）→ 使用固定角度朝向己方聯盟區
 3. 計算到 Hub 的距離
 4. 取得目前角度 (getPose().getRotation())
 5. PID 計算旋轉速度（不限速）→ setAimSpeed() 疊加到底盤
 6. 根據區域選擇射手速度：
    ├── 己方聯盟區 → 多項式曲線計算 RPS:
    │     RPS = -0.686d² + 13.186d + 21.912
    │     clamp 在 35~70 RPS
    └── 中立區 → 固定 kMidFieldReturnRps (70 RPS)
 7. 設定 Shooter 目標速度（按下按鈕就開始轉，不等對齊）
 8. 判斷角度對齊（遲滯邏輯，見下方）
 9. 角度對齊 AND 射手達速?
    ✅ → 啟動 transport + up_to_shoot 送球（同時啟動、同時停止）
    ❌ → 停止 transport + up_to_shoot
```

### 9.3.1 遲滯防抖 (Hysteresis)

為避免機器人移動中些許角度偏差導致連射中斷，使用**雙閾值遲滯邏輯**：

```
                    ┌───── kFeedingHysteresisDeg (5°) ─────┐
                    │                                       │
  ──────────────────┤        送球中保持發射                   ├──────
                    │                                       │
          ┌── kRotationToleranceDeg (2°) ──┐                │
          │                                │                │
  ────────┤      初次觸發門檻               ├────────────────┤
          │                                │                │
          └────────────────────────────────┘                │
                    └───────────────────────────────────────┘
```

| 狀態 | 觸發條件 | 說明 |
|------|---------|------|
| 尚未送球 → 開始送球 | 角度誤差 ≤ **2°** AND 射手達速 | 嚴格門檻，確保首發精準 |
| 正在送球 → 繼續送球 | 角度誤差 ≤ **5°** AND 射手達速 | 寬鬆門檻，連射不中斷 |
| 正在送球 → 停止送球 | 角度誤差 > **5°** OR 射手失速 | 偏差太大才暫停 |

### 9.3.2 中場區域雙目標切換 (Mid-Field Zone Logic)

機器人在**中立區（中場）**時，射擊行為與在己方聯盟區不同：

```
場地座標示意圖 (wpiBlue 座標系) — 2026 REBUILT
X = 0m                    X ≈ 8.27m (中線)             X = 16.541m
 ┌────────────────────────────┬────────────────────────────┐
 │         藍方聯盟區          │         紅方聯盟區          │
 │                            │                            │
 │     🔵 藍方 Hub             │            🔴 紅方 Hub     │
 │     (4.63, 4.04)           │       (16.541-4.63, 4.04)  │
 │                            │       = (11.92, 4.04)      │
 │  ⬅ 藍方回傳方向 (180°)      │   ➡ 紅方回傳方向 (0°)      │
 │                            │                            │
 └────────────────────────────┴────────────────────────────┘
  ※ Hub 在場地中央區域，不在牆壁邊！
  ※ 紅方 Hub 座標由 kFieldLengthMeters - kHubX 自動計算（對稱場地）
  ※ 只需維護藍方座標 (kHubX, kHubY)，紅方自動鏡像
```

**區域判斷邏輯**（`AutoAimConstants.isInOwnZone()`）：

| 聯盟顏色 | 己方聯盟區 | 中立區（中場） |
|---------|-----------|--------------|
| **藍方** | robotX < 8.27m | robotX ≥ 8.27m |
| **紅方** | robotX ≥ 8.27m | robotX < 8.27m |

**不同區域的行為差異**：

| | 己方聯盟區 | 中立區（中場） |
|---|---|---|
| **瞄準方式** | atan2 計算面向 Hub 的角度 | 固定角度朝向己方聯盟區 |
| **目的** | 射入 Hub 得分 (Fuel) | 將球回傳至己方區域 |
| **射手速度** | 多項式曲線：RPS = -0.686d² + 13.186d + 21.912 | 固定 70 RPS |
| **RPS 範圍** | 35~70 RPS（clamp） | 70 RPS |

> 💡 **實際場景**：比賽中在中場拿到球時，按住按鈕不會瞄向 Hub（太遠射不進），而是面向己方聯盟區方向，將球高速射回，讓隊友或自己回到近距離再射擊得分。

### 9.4 所有可調參數一覽表

#### 🔫 Shooter 馬達 PID（Shuffleboard "Shooter PID" Tab 即時調整）

| 參數 | 預設值 | 常數位置 | 效果說明 |
|------|-------|---------|---------|
| **kV** | `0.12` | `ShooterConstants.kDefaultKV` | 前饋速度增益：每 1 RPS 需要多少伏特。計算方式：`12V / 空載最大RPS`。**第一個要調的參數** |
| **kP** | `0.12` | `ShooterConstants.kDefaultKP` | 比例增益：誤差越大，補償越多。太大會震盪 |
| **kI** | `0.0` | `ShooterConstants.kDefaultKI` | 積分增益：消除穩態誤差。太大造成積分飽和 |
| **kD** | `0.0` | `ShooterConstants.kDefaultKD` | 微分增益：抑制震盪。飛輪通常設 0 |
| **kS** | `0.0` | `ShooterConstants.kDefaultKS` | 靜摩擦補償：克服靜止摩擦所需最小電壓 |

#### 🔄 IntakeRoller 馬達 PID（Shuffleboard "IntakeRoller PID" Tab 即時調整）

| 參數 | 預設值 | 常數位置 | 效果說明 |
|------|-------|---------|---------|
| **kV** | `0.12` | `IntakeRollerConstants.kDefaultKV` | 前饋速度增益 |
| **kP** | `0.2` | `IntakeRollerConstants.kDefaultKP` | 比例增益 |
| **kI** | `0.01` | `IntakeRollerConstants.kDefaultKI` | 積分增益 |
| **kD** | `0.0` | `IntakeRollerConstants.kDefaultKD` | 微分增益 |

#### 🦾 IntakeArm 馬達 PID（Shuffleboard "IntakeArm PID" Tab 即時調整 — 目前停用）

| 參數 | 預設值 | 常數位置 | 效果說明 |
|------|-------|---------|---------|
| **kP** | `2.0` | `IntakeArmConstants.kDefaultKP` | 比例增益 |
| **kI** | `0.0` | `IntakeArmConstants.kDefaultKI` | 積分增益 |
| **kD** | `0.1` | `IntakeArmConstants.kDefaultKD` | 微分增益 |
| **kG** | `0.1` | `IntakeArmConstants.kDefaultKG` | 重力補償：抵抗手臂重力所需電壓 |

#### 🎯 自動瞄準旋轉 PID（Shuffleboard "AutoAim" Tab 即時調整）

| 參數 | 預設值 | 常數位置 | 效果說明 |
|------|-------|---------|---------|
| **Rotation kP** | `5.0` | `AutoAimConstants.kRotation_kP` | 旋轉比例增益。太大會來回震盪 |
| **Rotation kI** | `0.0` | `AutoAimConstants.kRotation_kI` | 旋轉積分。通常設 0 |
| **Rotation kD** | `0.1` | `AutoAimConstants.kRotation_kD` | 旋轉微分。抑制震盪 |

#### 📏 距離 → 射手 RPS 多項式曲線

> **取代了原本的查表法**。使用 2 次多項式迴歸擬合 8 個實測數據點，最大誤差 ≈ 1.3 RPS。

**公式**：`RPS(d) = kRpsA × d² + kRpsB × d + kRpsC`

| 係數 | 值 | 常數位置 |
|------|------|---------|
| **kRpsA** | `-0.686275` | `AutoAimConstants.kRpsA` |
| **kRpsB** | `13.186275` | `AutoAimConstants.kRpsB` |
| **kRpsC** | `21.911765` | `AutoAimConstants.kRpsC` |

**原始測量數據（校驗用）**：

| 距離 (m) | 目標 RPS | 多項式計算值 |
|----------|---------|-------------|
| 1.0 | 35 | 34.41 |
| 1.5 | 40 | 40.36 |
| 2.0 | 45 | 45.63 |
| 2.5 | 50 | 50.21 |
| 3.0 | 55 | 54.12 |
| 3.5 | 60 | 57.33 |
| 4.0 | 65 | 63.87 |
| 5.0 | 70 | 70.11 |

**安全限制**：

| 參數 | 值 | 說明 |
|------|------|------|
| `kRpsMinDistance` | `1.0 m` | 最近測量距離 |
| `kRpsMaxDistance` | `5.0 m` | 最遠測量距離 |
| `kRpsMin` | `35.0 RPS` | 最低限制 |
| `kRpsMax` | `70.0 RPS` | 最高限制 |

> ⚠️ 超出 1.0~5.0m 範圍直接回傳邊界值，曲線結果也會 clamp 到 35~70 RPS。  
> 如果更換射手機構或重新測量，使用 Python 腳本重新擬合係數。

---

#### ⚙️ 容許誤差 & 閾值

| 參數 | 預設值 | 常數位置 | 效果說明 |
|------|-------|---------|---------|
| **kRotationToleranceDeg** | `2.0°` | `AutoAimConstants` | **首次觸發**送球的角度門檻 |
| **kFeedingHysteresisDeg** | `5.0°` | `AutoAimConstants` | **連射中**保持送球的寬鬆門檻 |
| **kShooterToleranceRps** | `3.0 RPS` | `AutoAimConstants` | 射手在目標 ± 此值內視為「達速」 |
| **kShootingModeSpeedMultiplier** | `0.3` | `AutoAimConstants` | 射擊模式下平移速度倍率（30%） |

#### 🎯 Hub 座標（2026 REBUILT — 單一來源）

| 參數 | 值 | 說明 |
|------|------|------|
| **kHubX** | `4.626 m` | 藍方 Hub X 座標 |
| **kHubY** | `4.035 m` | Hub Y 座標（紅藍相同） |

> ✅ **只需維護藍方 Hub 座標**！紅方由 `getHubPosition(true)` 自動計算：  
> `RedHubX = kFieldLengthMeters - kHubX = 16.541 - 4.626 = 11.915m`

#### 🔄 中場回傳

| 參數 | 值 | 常數位置 | 說明 |
|------|------|---------|------|
| **kReturnAngleRad** | `π (180°)` | `AutoAimConstants` | 藍方基準回傳角度（紅方自動翻轉為 0°） |
| **kMidFieldReturnRps** | `70.0 RPS` | `AutoAimConstants` | 中場回傳固定射速 |
| **kFieldLengthMeters** | `16.541 m` | `AutoAimConstants` | 場地全長 |
| **kFieldMidX** | `≈8.27 m` | `AutoAimConstants`（自動計算） | 場地中線 X |

#### 🗺️ 射手安裝方向

| 參數 | 值 | 常數位置 | 說明 |
|------|------|---------|------|
| **kShooterAngleOffsetRad** | `π (180°)` | `AutoAimConstants` | 射手在機器人背面，需 +180° 偏移。正前方設 0 |

---

#### 🚗 ManualDrive 相關

| 參數 | 預設值 | 常數位置 | 效果說明 |
|------|-------|---------|---------|
| **kXMultiplier** | `1.0` | `ManualDriveConstants` | X 方向搖桿靈敏度倍率 |
| **kYMultiplier** | `1.0` | `ManualDriveConstants` | Y 方向搖桿靈敏度倍率 |
| **kZMultiplier** | `-0.4` | `ManualDriveConstants` | 旋轉靈敏度倍率（負號=反轉方向），旋轉最大速度 = 平移的 40% |
| **kXDeadzone** | `0.05` | `ManualDriveConstants` | X 搖桿死區（5%） |
| **kYDeadzone** | `0.05` | `ManualDriveConstants` | Y 搖桿死區 |
| **kZDeadzone** | `0.05` | `ManualDriveConstants` | 旋轉搖桿死區 |
| **boostTranslation** | `0.5 / 1.0` | `ManualDrive.java` | 不按 Right Bumper = 50% 速度，按住 = 100% 速度 |

#### 🔄 Transport 馬達

| 參數 | 預設值 (RPS) | 常數位置 | 效果說明 |
|------|-------------|---------|---------|
| **kTransportRps** | `35.0` | `TransportConstants` | 輸送帶正常轉速 |
| **kUpToShootRps** | `80.0` | `TransportConstants` | 上膛推球速度（推球進射手飛輪） |
| **kSlowTransportRps** | `18.0` | `TransportConstants` | 慢速（Auto Intake 時使用） |
| **kDefaultKV** | `0.12` | `TransportConstants` | 前饋增益 |
| **kDefaultKP** | `0.2` | `TransportConstants` | 比例增益 |
| **kStatorCurrentLimit** | `60.0 A` | `TransportConstants` | 定子電流限制 |
| **kSupplyCurrentLimit** | `40.0 A` | `TransportConstants` | 供電電流限制 |

#### 🚀 Swerve 加速度限制 (SlewRateLimiter)

| 參數 | 預設值 | 位置 | 效果說明 |
|------|-------|------|---------|
| **xSpeedLimiter** | `20 m/s²` | `Swerve.java` 建構式 | X 平移加速度上限 |
| **ySpeedLimiter** | `20 m/s²` | `Swerve.java` 建構式 | Y 平移加速度上限 |
| **zSpeedLimiter** | `15 rad/s²` | `Swerve.java` 建構式 | 旋轉加速度上限 |

---

### 9.5 AutoAimAndShoot 使用場景對照

左肩鍵 (Left Bumper) 和右扳機 (Right Trigger > 10%) **功能完全相同**，都觸發 `AutoAimAndShoot`。

差異僅在 Shuffleboard：
- **Left Bumper** 的 AutoAimAndShoot 傳入 `autoAimTab`，會更新 AutoAim Tab 上的 Widget
- **Right Trigger** 的 AutoAimAndShoot 不傳 Tab（`null`），改用 SmartDashboard 輸出

| 功能 | 自動瞄準射擊 (LB / RT) |
|------|----------------------|
| **底盤** | 旋轉鎖定（PID 接管），平移 ×0.3 |
| **瞄準** | 己方區 → Hub；中場 → 己方回傳點 |
| **射手速度** | 己方區 → 多項式曲線自動計算 (35~70 RPS)；中場 → 70 RPS |
| **送球** | transport + up_to_shoot 同時啟動/停止，遲滯邏輯保護 |
| **適用場景** | 任意距離、自動對齊；中場回傳球 |

---

### 9.6 調參步驟建議

#### Step 1: 調 Shooter 馬達 PID

```
1. 開啟 Shuffleboard → "Shooter PID" Tab
2. 設定 kI=0, kD=0, kS=0
3. 先算 kV：量測空載最大 RPS（例如 95 RPS）
   kV = 12V / 95 = 0.126
4. 設定一個目標 RPS（例如 50），觀察 Current RPS 波形
5. 慢慢增加 kP 直到響應夠快但不震盪
6. 如果有穩態誤差 → 微調 kI（建議 0.01 起步）
7. 調好後寫回 Constants.java → ShooterConstants.kDefaultKV/kP/...
```

#### Step 2: 調自動瞄準旋轉 PID

```
1. 開啟 Shuffleboard → "AutoAim" Tab
2. 按住 Left Bumper 觸發自動瞄準
3. 觀察 "Angle Error" 波形
4. 如果震盪嚴重 → 降低 Rotation kP 或增加 kD
5. 如果收斂太慢 → 增加 Rotation kP
6. 調好後寫回 Constants.java → AutoAimConstants.kRotation_kP/kI/kD
```

#### Step 3: 校準距離-RPS 曲線

```
1. 把機器人放在 1m、2m、3m、4m、5m 處
2. 每個距離手動測試不同 RPS
3. 找到每個距離能穩定進球的 RPS
4. 用 Python 做 2 次多項式迴歸擬合：
   np.polyfit([1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0], 
              [新RPS1, 新RPS2, ...], 2)
5. 將新係數寫入 Constants.java → AutoAimConstants.kRpsA/kRpsB/kRpsC
```

#### Step 4: 調容許誤差與遲滯

```
1. 觀察 Shuffleboard AutoAim Tab 的 "Aligned?" 和 "At Speed?" 指示燈
2. "Aligned?" 一直不亮 → 放寬 kRotationToleranceDeg（2° → 3°）
3. 進球率低 → 收緊 kRotationToleranceDeg
4. 連射頻繁中斷 → 增加 kFeedingHysteresisDeg（5° → 7°）
5. 連射中打歪 → 減小 kFeedingHysteresisDeg（5° → 3°）
6. 建議範圍：首次觸發 1°~3°，遲滯 3°~8°，射手速度誤差 2~5 RPS
```

---

## 遙測節流機制

所有 Shuffleboard/SmartDashboard 輸出都使用節流機制，避免 NetworkTables 佔用過多 CPU 和網路頻寬：

```java
// Constants.java
public static final int kTelemetryDivider = 5;
// → 更新頻率 = 50Hz / 5 = 10Hz (每 100ms 更新一次)
```

每個子系統和 Command 各自維護 `telemetryCounter`，只有 `counter >= kTelemetryDivider` 時才寫入 NT。

比賽時如需進一步降頻（例如網路壅塞），將 `kTelemetryDivider` 改為 10（= 5Hz）。

---

## 快速參考卡

```
啟動流程：
  Ctrl+Shift+P → "Simulate Robot Code" → 勾選 halsim_gui → OK
  Ctrl+Shift+P → "Start Tool" → Shuffleboard
  Glass 視窗 → 點選 "Teleop"
  Shuffleboard → 確認 Connected → 切換到對應 Tab

PID 調參：
  Tab 上直接點擊數值 → 輸入 → Enter → 即時生效
  調好後一定要寫回 Constants.java！

看場地位置：
  Main Tab → Field Widget → 機器人圖示即時更新

看迴圈超時：
  Main Tab → Loop ms Graph + Overrun 指示燈
  超過 20ms = 🔴 紅燈 = 需要優化程式碼

CAN ID 快查（Shuffleboard 遙測來源）：
  Shooter:       Leader=22, Follower=21
  IntakeRoller:  Leader=29, Follower=35
  IntakeArm:     Leader=32, Follower=33 (目前停用)
  Transport:     transport=30, up_to_shoot=26
  Swerve Throttle: 2/4/6/8
  Swerve Rotor:    1/3/5/7
  CANcoder:        11/12/13/14
  Pigeon2 IMU:     ID=0
```

---

*最後更新：2026-03-08（Fix 17 後全面更新：多項式 RPS、統一 Hub 座標、Transport RPS 常數、Vision 遙測）*
