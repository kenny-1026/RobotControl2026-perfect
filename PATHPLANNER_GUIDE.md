# 🤖 PathPlanner 自動程式撰寫指南
**RobotControl2026 — FRC 2026 賽季**

> 本指南說明如何使用 PathPlanner GUI 為本機器人編寫自主期（Autonomous）程式，包含路徑規劃與觸發吸球、射擊等子系統功能。

---

## 目錄

1. [基本概念](#基本概念)
2. [機器人設定參數](#機器人設定參數)
3. [可用的 Named Commands（命名指令）](#可用的-named-commands)
4. [在 PathPlanner GUI 中建立自動程式](#在-pathplanner-gui-中建立自動程式)
5. [Auto 結構類型](#auto-結構類型)
6. [重要設定：resetOdom](#重要設定resetodom)
7. [自動射擊流程](#自動射擊流程)
8. [自動吸球流程](#自動吸球流程)
9. [現有自動程式範例](#現有自動程式範例)
10. [建立新路徑的步驟](#建立新路徑的步驟)
11. [常見問題與注意事項](#常見問題與注意事項)

---

## 基本概念

### PathPlanner 與機器人的連接方式

```
PathPlanner GUI (.auto 檔案)
        ↓
AutoBuilder.configure() ← 設定於 RobotContainer.java
        ↓
NamedCommands.registerCommand() ← 連接 GUI 指令名稱與實際 Java Command
        ↓
子系統執行（Shooter / Transport / IntakeRoller）
```

- **路徑 (.path)**：機器人行駛的軌跡
- **Auto (.auto)**：包含路徑與命令的完整自主期程序
- **Named Command**：在 Java 中預先定義好的指令，在 PathPlanner GUI 中以「名稱字串」呼叫

---

## 機器人設定參數

在 PathPlanner Settings 中設定好的機器人規格（供參考，**不要修改**）：

| 參數 | 數值 |
|------|------|
| 機器人寬度 | 0.845 m |
| 機器人長度 | 0.845 m |
| 驅動模式 | Holonomic（全向） |
| 預設最大速度 | 1.5 m/s |
| 預設最大加速度 | 1.5 m/s² |
| 最大角速度 | 25.0 deg/s |
| 驅動馬達 | Kraken X60 |
| 最高馬達速度 | 5.3 m/s |
| 機器人質量 | 52.16 kg |
| 輪距 | 0.546 m |

---

## 可用的 Named Commands

以下是在 `RobotContainer.java` 中已註冊的所有命名指令，**名稱必須完全一致（含大小寫、空格）**：

### 🎯 射擊指令

| 指令名稱 | 功能 | 超時限制 | 說明 |
|----------|------|----------|------|
| `Auto Shoot` | 完整自動射擊 | **4 秒** | 同時啟動射手（依距離自動計算 RPS）+ 等待達速（最多 2 秒）+ 啟動輸送帶出球 |
| `Far Auto Shoot` | 完整自動射擊 | **4 秒** | 功能與 `Auto Shoot` 完全相同（保留兩個名稱以相容舊 Auto 檔案） |
| `shoot work` | 僅啟動射手 | 無超時 | 只讓飛輪轉速達到目標值，**不**觸發輸送帶（常與 `transport wait shoot` 搭配） |
| `transport wait shoot` | 等速後出球 | 無超時 | 等射手達到目標 RPS 後才啟動輸送帶（需搭配 `shoot work` 使用） |

> **推薦使用 `Auto Shoot`**，它包含完整的射擊流程並有 4 秒超時保護。

### 🟡 吸球指令

| 指令名稱 | 功能 | 超時限制 | 說明 |
|----------|------|----------|------|
| `Auto Intake` | 完整自動吸球 | **2.5 秒** | 同時啟動吸球滾輪 + 慢速輸送帶，球會進入待射位置 |
| `Start Intake` | 僅啟動吸球滾輪 | 無超時 | 只啟動滾輪，不含輸送帶，需另外停止 |
| `Stop Intake` | 停止吸球滾輪 | 立即完成 | 停止 IntakeRoller |

> **推薦使用 `Auto Intake`**，有 2.5 秒超時保護，不會讓吸球無限持續。

---

## 在 PathPlanner GUI 中建立自動程式

### 步驟一：建立或選擇路徑

1. 開啟 PathPlanner 應用程式
2. 連接到本專案的 `src/main/deploy/pathplanner/` 目錄
3. 點擊左側 **Paths** → **新增路徑** 或開啟現有路徑
4. 設定路徑的起點、終點與中間控制點
5. 設定 **globalConstraints**（速度/加速度）
6. 設定 **goalEndState**（到達終點時的方向）
7. 儲存路徑（`.path` 檔案）

### 步驟二：建立 Auto 程式

1. 點擊左側 **Autos** → **新增 Auto**
2. 點擊畫面右側的 **+** 按鈕，加入各項元素

### 步驟三：加入路徑

- 選擇 **Follow Path**（跟隨路徑）
- 從下拉選單選擇已建立的路徑名稱
- 路徑可串聯，也可平行執行

### 步驟四：加入 Named Command

1. 點擊 **+** 按鈕
2. 選擇 **Named Command**
3. 在名稱欄輸入指令名稱（**必須與 Java 中的名稱完全一致**）
4. 設定是否需要超時（Named Command 本身的超時，獨立於 Java 中的超時）

---

## Auto 結構類型

### 1. 循序執行（Sequential Group）

一個接一個按順序執行，**前一個完成後才執行下一個**：

```
[路徑 A] → [Auto Shoot] → [路徑 B] → [Auto Shoot]
```

用途：射擊後再移動，確保先完成射擊再走

---

### 2. 平行執行（Parallel Group）

多個動作**同時執行**，等所有動作完成才繼續：

```
┌─ [路徑：去吃球] ─────────┐
│                           │ → 都完成後繼續
└─ [Auto Intake：吸球] ────┘
```

用途：邊移動邊吸球，節省時間

> ⚠️ 平行群組中，**路徑跟隨與 Named Command 同時進行**。
> 通常路徑的行駛時間 > 指令時間（2.5s），讓吸球在移動中自然完成。

---

### 3. 混合結構（Sequential + Parallel）

現有 Auto 常見模式：

```
[起始路徑] → [Auto Shoot] → [平行：移動到球 + Auto Intake] → [返回路徑] → [Auto Shoot]
```

---

## 重要設定：resetOdom

在 `.auto` 檔案頂部設定 `"resetOdom": true`：

```json
{
  "version": "2025.0",
  "resetOdom": true,
  ...
}
```

**效果**：
- 自主期開始時，機器人 Odometry（位置追蹤）自動重置到第一條路徑的**起點**
- 同時同步 Pigeon2 IMU 的 Yaw 角度
- 確保自主期路徑執行位置準確

**⚠️ 強烈建議所有 Auto 都設定 `resetOdom: true`**，確保定位正確。

---

## 自動射擊流程

使用 `Auto Shoot`（或 `Far Auto Shoot`）時，Java 內部執行流程：

```
Auto Shoot 觸發
    ↓
[平行執行，總超時 4 秒]
├─ 啟動射手飛輪
│   └─ getAdaptiveRps() 計算目標轉速
│       └─ 取得 Limelight 距離 d
│       └─ RPS = -0.686d² + 13.186d + 21.912
│       └─ 限制在 35–70 RPS 之間
│
└─ 等待達速（最多 2 秒）→ 啟動輸送帶出球
    └─ 達速後：TransportSubsystem 開始送球
    └─ 未達速（2 秒後）：強制繼續輸送帶
    └─ 整體 4 秒後超時停止
```

### 使用時機
- 機器人在好的射擊位置**停下後**使用
- 建議：路徑的 `goalEndState` 速度設為 `0`，讓機器人停穩再射擊

---

## 自動吸球流程

使用 `Auto Intake` 時，Java 內部執行流程：

```
Auto Intake 觸發
    ↓
[平行執行，總超時 2.5 秒]
├─ IntakeRoller 啟動（吸球滾輪持續轉動）
└─ TransportSubsystem 慢速正轉（將球推向待射位置）
    ↓
2.5 秒後自動停止
```

### 最佳搭配：邊移動邊吸球

```
[平行群組]
├─ [路徑：移動到球的位置]  ← 路徑行駛中
└─ [Auto Intake]          ← 同時吸球（2.5 秒後停止）
```

> 確保路徑設計使機器人能在移動過程中通過球的位置，讓吸球有足夠時間（建議路徑 > 2 秒）。

---

## 現有自動程式範例

### 📋 New right Auto.auto（推薦參考）

**結構**：
```
[重置定位 resetOdom: true]
    ↓
[路徑：1_Start] → 移動到射擊位置
    ↓
[Auto Shoot] → 射出預裝球（4 秒）
    ↓
[平行執行]
├─ [路徑：2_Go_2_mid] → 移動到球的位置
└─ [Auto Intake] → 移動時吸球（2.5 秒）
    ↓
[路徑：3_Eat_ball_and_shoot] → 移動到射擊位置（含 Auto Shoot 觸發）
    ↓
[Auto Shoot] → 射出吸入的球（4 秒）
```

**設計亮點**：
- 首先射出預裝球，再去吸更多球
- 利用移動時間同步吸球
- 使用 `resetOdom: true` 確保定位

---

### 📋 manual right auto.auto

**結構**：
```
[路徑：Straight Right Path] → 移動到射擊位置
    ↓
[Auto Shoot] → 射擊
    ↓
[路徑：go right forward] → 前進到球
    ↓
[平行執行]
├─ [路徑：right eat ball] → 對準球
└─ [Auto Intake] → 吸球
    ↓
[路徑：right back] → 回到射擊位置
    ↓
[路徑：Straight Right Path] → 定位
    ↓
[Auto Shoot] → 射擊吸入的球
```

---

### 📋 strange Right Auto.auto（遠距射擊版）

**結構**：
```
[重置定位 resetOdom: true]
    ↓
[路徑：Straight Right Path] → 移動
    ↓
[Far Auto Shoot] → 遠距射擊（功能同 Auto Shoot）
    ↓
[路徑：go forward] → 前進
    ↓
[平行執行]
├─ [路徑：eat ball] → 對準球
└─ [Auto Intake] → 吸球
    ↓
[路徑：back and shoot] → 返回並觸發射擊
    ↓
[Far Auto Shoot] → 射擊
```

---

## 建立新路徑的步驟

1. **在 PathPlanner GUI 中建立路徑**
   - 設定合理的速度限制（建議最大 2.0 m/s，加速度 1.5–2.0 m/s²）
   - 設定 `goalEndState.velocity = 0.0`（停下）如果之後要射擊
   - 設定 `goalEndState.rotation`（面向 Hub 或球的方向）

2. **規劃球的撿取路徑**
   - 讓路徑通過球的位置附近（0.3–0.5m 內）
   - 路徑行駛時間建議 > 2 秒（讓 `Auto Intake` 有時間完成）

3. **在 Auto 中組合**
   - 使用 Parallel Group 同時跑路徑和 `Auto Intake`
   - 使用 Sequential 確保到位後再射擊

4. **設定起始點**
   - 第一條路徑的起點 = 機器人擺放位置
   - 開啟 `resetOdom: true`

---

## 常見問題與注意事項

### ⚠️ Named Command 名稱錯誤

**問題**：Auto 中的指令沒有執行
**原因**：PathPlanner GUI 中填入的名稱與 Java 中 `NamedCommands.registerCommand()` 的名稱不符
**解決**：

對照下表確認名稱（**區分大小寫**）：

```
✅ 正確：  Auto Shoot
❌ 錯誤：  auto shoot / AutoShoot / Auto  Shoot
```

---

### ⚠️ `Auto Shoot` 與 `Far Auto Shoot` 的差異

兩者功能**完全相同**，都是 4 秒超時的完整射擊指令。
`Far Auto Shoot` 是為了相容舊 Auto 程式而保留的別名。
**新程式一律使用 `Auto Shoot`**。

---

### ⚠️ 射擊前確認停穩

如果機器人還在移動時觸發 `Auto Shoot`，可能導致射擊偏差。
建議：
- 前一條路徑的 `goalEndState.velocity = 0.0`
- 或在路徑與 `Auto Shoot` 之間加 `waitSeconds(0.2)` 等待穩定

---

### ⚠️ Limelight 必須看到 Hub

`Auto Shoot` 使用 Limelight 距離計算 RPS。
如果 Limelight 沒有偵測到目標，距離回傳 0，RPS 會被夾限到最小值（35 RPS）。
確保射擊路徑規劃時機器人朝向 Hub。

---

### ⚠️ `Auto Intake` 超時 2.5 秒

如果移動路徑很短（< 2.5 秒），`Auto Intake` 會在路徑完成前就停止。
反之，如果路徑很長，`Auto Intake` 先停止，機器人還在繼續移動（這是正常的）。

---

### ⚠️ `resetOdom` 需要機器人擺放位置正確

`resetOdom: true` 會將機器人位置重置到第一條路徑的起點座標。
放置機器人時必須對準場地標記，否則所有路徑都會偏移。

---

## 快速參考卡

```
新自動程式 Checklist：
□ 路徑設計：起點 = 機器人放置點
□ 射擊路徑：goalEndState.velocity = 0.0
□ Auto 設定：resetOdom: true
□ Named Command 名稱確認：
    - 射擊：Auto Shoot
    - 吸球：Auto Intake
□ 平行群組：移動路徑 + Auto Intake
□ 在 Shuffleboard/FMS 選擇正確 Auto
□ 測試前確認 Limelight 有目標
```

---

*最後更新：RobotControl2026 Fix 17 後*
