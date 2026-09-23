1. 現在的retargetting projectC:\Users\Howard\retargeting_crx是一個雙臂crx+leap hand靈巧手，我們現在想要安裝兩隻sharpa 的 wave hand 到兩個crx上，由於sharpa hand 的自由度很高，想要請你評估有哪邊需要改動
2. 我們現在只有和兩隻crx發布joint指令的script，在scripts/run_crx_joint_teleop.py，crx那邊的launch file在這C:\Users\Howard\dual_crx_control\launch\dual_arm.launch.py，我想要另外創一個單獨控制兩隻sharpa手的script，目前雙手使用的launch檔案在這邊C:\Users\Howard\dual_sharpa_wave_ros2\launch\dual_sharpa.launch.py
3. 最後希望還可以有一個script可以通時發布指令給dua_crx和sharpa手
4.請你讓大部分的改動都在retargetting project裡面，可以啟動ros做測試，但現在這台筆電沒有接上任何機器人和裝置
5. minimal effort，先以東西串的起來，可用為主


你的method:

## 建議方案

採用「沿用現有 retargeting 與兩個 ROS driver，新增 Sharpa 模型設定和兩個薄入口 script」的方式。先接通雙手，再接雙臂＋雙手；所有新增控制邏輯放在 `retargeting_crx`，原有 CRX／LEAP 使用方式保持相容。

本文件是依目前三個本機 repository 的程式碼撰寫的實作計畫。已完成左右 Sharpa 資產的本地複製及靜態檔案檢查；尚未修改控制程式、完成 CRX＋Sharpa 組合模型或執行 ROS／preview 測試。

## 1. 已確認的現況與必要改動

| 項目 | 目前程式碼 | Sharpa 接入方式 |
| --- | --- | --- |
| CRX 發布介面 | `scripts/run_crx_joint_teleop.py` 發布 `/crx5ia/joint_targets`，訂閱 `/crx5ia/joint_states` | 沿用 `sensor_msgs/msg/JointState`，兩臂共 12 個關節 |
| Sharpa 發布介面 | `dual_sharpa_wave/hand_node.py` 在左右 namespace 接收 `joint_command`、發布 `joint_states` | 每手單獨一筆完整 22 關節命令，ROS 單位為 rad |
| Sharpa 模型 | 左右 `*_sharpa_wave_with_flange.urdf`；每手 22 個 revolute joint，無 mimic | 使用左右各自模型，不以右手模型鏡射代替左手 |
| 現有模型維度 | CRX 6＋LEAP 16＝單側 22，雙側 44 | CRX 6＋Sharpa 22＝單側 28，雙側 56；純雙手為 44 |
| 雙側 execution flow | `src/teleoperation/bimanual_execution.py` 固定使用 44、22、6、16 及對應切片 | 由左右 robot/profile 推導總長度、分界及 arm／hand 切片；保留原有預設行為 |
| composition／viewer | `composition.py` 手部 filter 使用 `6:22`；`visualization/execution/bimanual.py` 以 22 切 actual qpos | composition 與 viewer 都配合各側維度；靜態 preview 和 Quest 動態 preview 為必要交付 |
| 既有 CRX script | `PROFILE_NAMES`、`ARM_INDICES`、`checked_qpos()` 綁定 LEAP 44 維資料 | 不直接把這些常數換成 56；新入口使用 Sharpa 配置與名稱映射 |
| 核心求解器 | 已有 `vector_wrist_joint_panda_shadow.yaml` 五指設定，`Retargeter` 按 fingertip 設定建立目標 | 優先新增 robot/profile，先不重寫 optimizer |

特別注意：舊 CRX＋LEAP 和純雙 Sharpa 都是 44 維，但排列意義完全不同。不能只檢查陣列長度，必須同時檢查模型的關節名稱與順序。

目前使用的是 `dual_crx_control` 的一般 JointState 介面，不需要為此接入舊 `dual_crx_ros2` 的 `TeleopCommand`、lease 或 LEAP contract。`src/teleoperation/backends/dual_crx_contract.py` 的既有 22／44 維約定應保留。

## 2. 模型與五指 retargeting

1. 新增純 Sharpa 左／右手，以及 CRX＋Sharpa 左／右側的 robot YAML、method profile 和 bimanual YAML。沿用既有 YAML 欄位，初始角度、关節順序、限制、手腕 frame、指尖 frame、權重及速度限制全部由設定提供。
2. 已將左右手 URDF、各 25 個 mesh 及授權檔實際複製到 `assets/robots/sharpa_wave_left/` 和 `assets/robots/sharpa_wave_right/`，並記錄來源 revision 與 SHA256。URDF 使用 `../meshes/`，不使用外部路徑或 symlink；後續模型組合與 robot YAML 一律讀取這份本地副本，不再讀取鄰近 `dual_sharpa_wave_ros2`。XML、引用檔案、checksum 和模型內容一致性已檢查；目前 `.venv` 缺少 Pinocchio，尚未做 kinematics 載入驗證。
3. 組合模型沿用目前 CRX 臂段，移除其 LEAP 子樹，接到 Sharpa 的 `{side}_hand_flange`；固定接合的 xyz／rpy 必須明確記錄。現有 LEAP 安裝旋轉和 Sharpa preview 的左右顯示偏移都不能當成新手的安裝標定。沒有實際安裝尺寸時，先使用明確標示的 mock 暫定值，實機前再確認。
4. 使用 Sharpa 五個 `{side}_*_fingertip` frame，對應 MANO 指尖 index `4, 8, 12, 16, 20`；方向參考點為 `3, 7, 11, 15, 19`。依實際末節 frame 與 FK 確認指尖方向軸，不直接複製 LEAP 的方向設定。
5. 參考 Shadow 的五指目標排列：world→thumb、wrist→五指、thumb→其餘四指、五指末節方向，共 15 組 link pairs。為 22／28 維模型分別提供完整長度的 posture、temporal 和速度權重。
6. 22 DOF 較多不代表需要換求解器，但冗餘自由度可能抖動。先使用原有 solver、URDF bounds、前一幀 warm start 和適量 posture／temporal regularization；以張手、握拳、拇指對指案例觀察穩定性，再調權重。PIP／DIP 先依模型獨立求解，不自行加入硬耦合。
7. 初始以 20 Hz 為目標，量測左右合計求解時間及過期幀丟棄情況。若無法穩定落在 50 ms 週期內，再調整求解容差／迭代數或降低輸入頻率；不承諾插值能彌補 retargeting 延遲。

純手模式使用 `arm_dof: 0`，將 observation 的 wrist pose 固定在模型手腕座標，只追蹤手腕局部的手指形狀，world-thumb／wrist-rotation 權重設為 0。先驗證現有 mapper／profile 對此組合的支援，必要時在 `teleoperation/observation_mapping.py` 補一個用途明確的固定手腕 mapper；不能只關閉 CRX publisher，卻仍讓虛擬手臂參與求解。

## 3. Script 和程式責任分配

以下為預計新增的名稱，尚未實作：

- `scripts/run_sharpa_joint_teleop.py`：只控制左右 Sharpa，使用兩個純手 retargeter；不等待 CRX feedback，也不建立 CRX publisher。
- `scripts/run_crx_sharpa_joint_teleop.py`：一份輸入、同一個 `BimanualExecutionFlow`，求解左右各 28 維，將結果分送 CRX 與左右手。
- `src/retargeting_apps/sharpa_teleop.py`：共用 CLI／composition，兩個 script 只提供模式預設和入口，不複製完整 loop。
- `src/retargeting_ros/sharpa_joint.py`：一個平坦的 ROS adapter，可依模式建立雙手或臂＋手 publishers／subscriptions，提供 flow 所需的 feedback、execute、pause、resume、stop 和 close；ROS import 保持 lazy。
- `src/teleoperation/bimanual_execution.py`：只改模型維度相關驗證與切片，沿用初始化、掉追蹤暫停、恢復、超時與生命週期。

路徑原則：本次新增程式不使用外部 repository 的絕對路徑、`sys.path` 插入或跨 repository Python import。模型與配置放在本專案；Sharpa 關節名稱由本地 URDF／配置提供，對照 driver 的介面契約驗證即可。ROS driver 透過已安裝的套件名稱啟動、透過 topic 溝通；ROS、Python 依賴及既有 pinned submodule 仍屬環境依賴，不複製成另一套本地程式碼。

新 adapter 參考既有 CRX script 的 feedback freshness、從實測姿態初始化、stop／close 行為。首版直接發布目標，CRX 插值交給 `dual_arm.launch.py`，Sharpa 插值交給既有 driver；不先搬動整份舊 script 或抽象出新的 runtime/session 層。

單獨雙手與整合模式共用手部輸出程式，但整合模式不能用 subprocess 同時啟動兩個 teleop script，避免重複開 Quest receiver、分別求解或各自管理 stop。

## 4. ROS 命令與 feedback 約定

| 用途 | Topic | 內容 |
| --- | --- | --- |
| 雙臂 command | `/crx5ia/joint_targets` | `left_J1..left_J6`、`right_J1..right_J6`，12 個 rad |
| 雙臂 feedback | `/crx5ia/joint_states` | 按名稱取出上述 12 關節 |
| 左手 command | `/sharpa/left_hand/joint_command` | 22 個 `left_*` canonical joint names 與 rad |
| 右手 command | `/sharpa/right_hand/joint_command` | 22 個 `right_*` canonical joint names 與 rad |
| 左／右手 feedback | `/sharpa/{side}_hand/joint_states` | 各自完整 22 關節實測值 |

Sharpa 名稱及順序以目前 driver 的 `dual_sharpa_wave/joint_names.py` 為依據：thumb 5、index 4、middle 4、ring 4、pinky 5。URDF 名稱中的 `thumb_IP` 不可自行改成 SDK 文件中的 DIP。ROS 端維持 rad，SDK 的 degree 轉換留在原 driver。

- 命令明確帶 `name`；feedback 按名稱重排，拒絕缺失、重複、未知必需關節、NaN／Inf 和錯誤長度。Sharpa QoS 對齊既有 `RELIABLE + VOLATILE + KEEP_LAST(depth=1)`。
- 56 維排列為 `[left_arm(6), left_hand(22), right_arm(6), right_hand(22)]`；由設定產生索引，發布前一次驗證全部輸出與限位，再送三個 topic。
- 三筆命令源自同一組左右輸入和同一次求解結果，可使用共同時間戳，但不同 ROS topic 不保證原子送達，也不代表四個裝置硬體同步到位。
- 初始化等待啟用裝置的完整、新鮮 feedback 與命令 subscriber，從當前實測值開始，避免先發 home／零角度。純雙手模式只等待兩手。
- 每個 feedback stream 獨立檢查接收時間與時間戳是否持續前進，不能用其中一個 topic 更新掩蓋另一個已中斷的裝置。
- 整合模式任一必要 feedback 或輸入逾時，就共同暫停新追蹤命令；協調既有 CRX／Sharpa timeout 與 hold 行為，測試恢復後重新由實測姿態校準。持續重發過期目標不能算作有效新輸入。
- Bimanual flow 現有 filter 不等於速度限制。新路徑須接上既有 command limiter 的適用部分或補最小必要的每關節限速，並在 ROS 發布前檢查 URDF bounds；不能只在 YAML 填 `max_joint_speed` 就視為已生效。

## 5. 沒有硬體的驗證順序

### A. Headless 測試

先加針對下列行為的測試，全部使用 `.venv/bin/python`：

1. 左右 Sharpa URDF 可載入，22／28 維 actuated joints、limits、初始值、指尖 frame、組合 flange transform 一致。
2. 原 LEAP 44 維、純 Sharpa 44 維、CRX＋Sharpa 56 維都能正確初始化、分割及 filter；關節名稱錯誤即使長度相同也必須失敗。
3. 合成或既有離線手部 observation 可得到有限且限位內的五指輸出；包含小指、左右手、固定手腕、warm start 與時間量測。
4. ROS adapter 的按名稱重排、輸出分流、錯誤命令拒絕、freshness、tracking loss、恢復與 shutdown，先用 fake transport 驗證。
5. 回歸現有 `test_bimanual_execution.py`、`test_bimanual_quest.py`、`test_crx_joint_script.py` 和 package import boundary 測試；共用 flow 調整後再跑完整 headless suite。

### B. ROS mock 串接

先驗證本機 `.venv/bin/python` 能同時 import retargeting 依賴和 ROS Jazzy 的 `rclpy`；目前程式要求 ROS 相容的 Python 3.12。ROS 測試需 source 對應 ROS／workspace，不沿用 headless 測試的清除 ROS 路徑做法。缺套件時明確記錄缺項及 skip，不宣稱已通過。

先只啟動雙手：

```bash
ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=mock use_rviz:=false
```

再增加雙臂 mock；若新入口實際以 20 Hz 直接發布，設定：

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false method:=ruckig input_rate_hz:=20.0
```

Quest 尚未接上時，測試用 source 注入同一個 flow，提供有限幀數、帶新鮮時間戳的合成／離線雙手 sample；先以直接關節目標驗證 ROS 傳輸，再驗證 mapper→retargeter→flow→adapter 的完整路徑。現在 composition 只接受 online Quest，需提供最小的測試 source 注入點，不能把尚未支援的 offline CLI 當成既成功能。使用者之後會將 Quest 接到 WSL，執行下述 live preview 驗收。

驗收內容為：純雙手模式無需 CRX 即可工作；整合模式三個 command topic 名稱／維度／單位正確、mock feedback 跟隨目標；中斷任何必要 stream 會觸發預期暫停；恢復無初始姿態跳轉；結束後不繼續發布。ROS 測試隔離 domain，關閉 RViz／viewer，不開 SDK hardware backend。

### C. 使用者驗收：先靜態擺放，再 Quest → WSL 動態追蹤

**第一階段：靜態 preview。** 先提供完整的雙 CRX＋左右 Sharpa 組合模型，沿用 `scripts/view_bimanual_initial.py --config ...` 的入口，在 Windows 瀏覽器查看 WSL 提供的 Viser 頁面（預設 port 9219）。不需要 Quest 或 ROS driver。確認左右臂位置、底座間距和朝向、初始關節姿態、法蘭接合位置、左右手型、手掌／拇指方向，以及 mesh 是否完整。顯示底座、CRX flange 和 Sharpa wrist 座標軸，讓使用者能指出需要修正的方向與位置。未知的安裝 transform 明確標示為暫定值，由使用者 preview 確認後記錄在本專案設定。

靜態入口目前直接將 initial qpos 傳給 viewer，需改為依關節名稱映射到 URDF 的順序，避免 Sharpa 關節排列不同造成錯誤外觀；座標軸也須使用新模型實際 frame 名稱。靜態和動態 preview 必須使用同一份模型、initial qpos 與 placement，避免兩邊顯示不同擺放。

**第二階段：Quest 接到 WSL，做動態 preview。** 使用既有 Quest WebXR／ADB 輸入與 kinematic backend，畫面同時顯示雙臂、兩隻 Sharpa、左右人手骨架與目標手腕標記。這一步只更新虛擬模型，無需實體機器人，也不發布實機控制命令。

1. 先確認執行 retargeting 的 WSL 環境中，`adb devices -l` 能看到已授權的 Quest。ADB 從 PATH 取得，不把 Windows 使用者目錄或 SDK 絕對路徑寫死。USB／ADB 與 WSL 的連線方式依實際環境確認。
2. 確認 Quest Browser 可連到 WSL 的 WebXR receiver（目前預設 port 8765），以及 Windows 瀏覽器可開啟 Viser（預設 port 9219）。不能只以 Windows ADB 看得到 Quest 就判定 WSL 串流已接通；需實際收到持續更新的雙手資料。
3. 雙手進入追蹤範圍後初始化，逐一檢查左／右手不互換，手腕平移、旋轉方向正確，以及五指張合、握拳、拇指對指和小指動作。保留左右獨立的 sensor-to-robot 校準，不直接假設 LEAP 的軸向設定適用 Sharpa。
4. 遮住其中一手或暫停串流，確認雙側暫停；恢復後重新校準，不能直接跳回舊目標。顯示或記錄接收幀、有效追蹤狀態、求解耗時及輸出頻率，以便區分串流問題和 solver 延遲。
5. 同時提供純雙 Sharpa 的 preview 選擇，用於檢查固定手腕模式下的手指追蹤；主要擺放驗收仍以完整雙臂＋雙手場景為準。

交付可直接執行的靜態 preview 與 Quest preview 命令，使用本專案相對設定路徑。上述入口需等 Sharpa 配置及 flow／viewer 維度調整完成後才可使用；目前匯入的獨立手部資產尚不代表完整場景已完成。ROS mock 測試仍保留為控制介面驗證，不能替代這兩階段的使用者驗收。

### D. 最後階段：接入實體機器人

使用者指定的驗收順序為：靜態模型擺放 preview → Quest／WSL 的虛擬模型追蹤 → 最後接入實體機器人。ROS mock 介面測試在實機前完成。

先完成前兩階段驗收，再於機器人可用且使用者明確要求啟動實機時，依序驗證雙手獨立 script、既有雙臂 command 路徑，最後驗證雙臂＋雙手整合 script。實機前核對實際安裝 transform、左右裝置對應、關節順序與限位，從各裝置的實測姿態初始化，先做小幅、低速動作，再測 Quest 追蹤、追蹤中斷、恢復和停止。記錄實機結果，不能將 mock 或 preview 通過標記為實機驗收通過。

目前先完成可供 preview 和 Quest 驗收的程式與設定，實機接入入口也需實作和 mock 驗證；機器人尚未接上不阻擋這些工作，但不能代替最後的實機測試。

## 6. 實作優先順序與交付範圍

1. 優先完成 CRX＋Sharpa 左右組合模型、robot／bimanual 設定和靜態 preview，交由使用者確認擺放。
2. 完成五指 profile、flow／viewer 維度處理與 Quest kinematic preview，讓使用者接 Quest 到 WSL 驗證雙臂＋雙手追蹤；同時支援純手固定手腕模式。
3. 完成 Sharpa ROS adapter、直接目標 mock smoke test、雙手獨立控制 script 與 56 維整合控制 script。
4. 完成 regression、ROS mock 端到端測試，以及可重現的 preview、Quest 連線與控制啟動文件。
5. 在使用者完成 preview／Quest 驗收並接上設備後，執行最後的實機接入測試。

首版以 URDF kinematics、靜態／Quest preview、headless 測試和 ROS mock 串通為主，不加入 MuJoCo 動力學、接觸／力控或新 SDK。外部兩個 driver repository 預期只需沿用 launch；若發現確實缺介面，再提出最小外部修改。

正式實作前仍需確認實際 CRX↔Sharpa 安裝 transform；它不阻擋 mock 接通。mock 通過只能驗證軟體接口與資料流，實機的安裝方向、校正和動作效果待裝置到位後驗證。
