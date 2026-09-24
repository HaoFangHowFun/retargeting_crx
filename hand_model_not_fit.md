1. 我剛剛在執行retargetting_CRX測試的時候，用這個指令.venv/bin/python scripts/run_sharpa_joint_teleop.py \
  --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50

  發現tracking手的模型和sharpa手本身的大小好像不是很fit，我在想tracking手的模型會不會是之前是用leap hand的大小來做的，所以現在套到sharpa比較不准，如果是的話

  能不能再做另一個關節模型讓他可以fit sharpa的手，我覺得可以用initial位置相減誤差最小來調整，用這個只有一雙手的應該最好比較，你先幫我確認遺下問題，然後提出解法


  你的解法:

  更正：這條指令和前述 CRX 手臂指令不同。我先前判斷「目前仍用 LEAP 手模型」是錯的。`scripts/run_sharpa_joint_teleop.py` 透過 `sharpa_teleop` 載入 `configs/bimanual/sharpa_wave.yaml`，左右各用 Sharpa Wave 的 URDF、22 關節與五指 profile。目前只能確認程式使用哪套模型，無法從程式碼單獨證明實體 Sharpa 尺寸、Quest 追蹤點或兩者之間哪個有誤差。`--publish-hz` 和 `--interpolation-horizon-ms` 只調整 ROS 輸出的時間取樣，不影響幾何。

  **可以調的縮放比例：`human_hand_scale`。Sharpa 左、右手目前都是 `1.0`；LEAP 設定是 `1.5`，但這條 Sharpa 指令不會使用 LEAP 的值。** 修改位置分別是 `configs/robots/sharpa_wave_left.yaml` 與 `configs/robots/sharpa_wave_right.yaml` 的 `human_hand_scale`。它會把 Quest 的手腕局部 keypoint 座標乘上該比例，再送進求解器；例如 `1.1` 代表追蹤骨架的手腕到指尖向量放大 10%。它**不會改變 Sharpa URDF 或實體手尺寸**，左右手也可分開校正。先量出誤差方向與比例，再調整這個值，避免只憑畫面猜數字。

  **即時比對 preview：**執行 `env -u PYTHONPATH .venv/bin/python scripts/preview_sharpa_scale.py`，在瀏覽器開啟 `http://localhost:9219`，用 **Left Quest hand scale** 和 **Right Quest hand scale** 滑桿邊看 Quest 骨架與 Sharpa 網格邊調整。滑桿範圍 `0.50–2.00`，預設讀取目前左右 Sharpa 設定（現在各為 `1.0`）；也可用 `--left-scale 1.2 --right-scale 1.1` 指定起始值。這個 preview 不連 ROS，也不送實體手命令。按 Ctrl+C 後終端會印出最終數值；確認多個手勢都合適後，再將其填回上述左右設定檔。若畫面中只有單一指節不合，應檢查關節與追蹤點對應，不要只靠整體比例修正。

  **不連 Quest 先看畫面：**在同一指令後加 `--demo`，程式會用動畫假手部資料顯示 Sharpa 網格、骨架和相同比例滑桿；按 Ctrl+C 結束。demo 模式保留原本的手部平滑。所有執行及 replay viewer 的追蹤標記點預設直徑現在都是 `0.006 m`，仍可用各自的 `human_keypoint_size` 設定覆寫。假手尺寸不能當成 Quest 的校正結果。若只想看 Sharpa 零位模型，不需要骨架與滑桿，則用 `env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py --config configs/bimanual/sharpa_wave.yaml`。

  這個 hand-only 模式的 Sharpa 手腕固定在模型位置，Quest 的 21 個點先轉成手腕局部座標，再用它們求手指關節。初始化時會以 ROS 回報的實際關節角作為模型起點，但不會用「人手初始位置減機器手初始位置」校正手指長度。因此先相減整隻手的初始位置，只能消除平移，不能修正手掌寬、手指長、關節軸、零位或手勢對不上的問題。ROS viewer 顯示的是回報關節角下的 Sharpa 網格與 Quest 追蹤骨架；若兩者有落差，也要區分是模型幾何與真手不符、Quest keypoint 有偏差，還是 ROS 回報關節角與真手姿態不一致。

  我的建議是先做一次可量化的對照，不急著新增第二套模型：

  1. 先用 `env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py --config configs/bimanual/sharpa_wave.yaml` 看零位 Sharpa 網格，再用 `env -u PYTHONPATH .venv/bin/python scripts/run_sharpa_joint_teleop.py --backend preview` 看 Quest 骨架與求解後的手。這兩步不會發送實體命令；若 preview 已經不合，優先查追蹤、座標或模型。既然你已經在 ROS 模式看到差異，也要記下 viewer 中是「藍色 Quest 骨架對 Sharpa 網格」不合，還是「網格對實體手」不合。
  2. 對左右手各錄一組穩定的張手、半握、握拳及單指彎曲資料，同步保存原始 Quest keypoint、映射後的手腕局部 keypoint、ROS 實測的 22 個關節角和求解命令。以實測關節角做 Sharpa URDF 的正向運動學，把手腕、五個指尖與指根都轉成手腕座標，依手指列出三維殘差與長度比。初始姿勢要和實測 `qpos` 配對，不能假定人的張手等於 URDF 的全零關節角。
  3. 如果實體手和 URDF 在相同實測關節角下仍有固定尺寸差，先以尺量手掌寬、各節長與指尖位置，核對 URDF 尺寸及關節零位／方向；只有確認 URDF 與真手幾何不同時，才製作校正後的 Sharpa 模型。如果 URDF 對真手正確、Quest 骨架卻整體同比例偏大或偏小，再用多個指尖及姿勢估計 `human_hand_scale`，例如最小化 `Σ ||p_URDF,i(q) - s·p_Quest,i||²`，並用未參與擬合的姿勢驗證。若只一根手指或某種彎曲動作不合，應查對應關節軸、限位、追蹤點對應及 profile 權重，不宜用全域縮放掩蓋它。

  所以「用初始位置讓誤差最小」可以當作量測的起點，但至少要比較多個對應點與多個姿勢，並先確定 ROS 實測角度和 URDF 零位一致。這樣才知道該調縮放、校正關節，還是真的需要另一套 Sharpa 幾何模型。
