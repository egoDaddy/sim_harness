# autoware_srunner — CARLA scenario_runner + Autoware e2e 联合仿真工程

## 工程结构

- `scenarios/ego/`    : ego 轨迹资产生成 (drive_straight.py 生成链 + 轨迹 json)
- `scenarios/npc/`    : NPC xosc 生成 (gen_accel_xosc.py + npc_accel_multistep.xosc)
- `config/autoware_srunner.yaml` : 联合仿真配置 (autoware / scenario_runner / ego 注入 / npc marker / 生命周期)
- `launch/autoware_srunner_launch.py` : 主控点
- `autoware_srunner/` : npc_marker_publisher / npc_ready_trigger / scenario_monitor /
                        wait_for_ready / set_route / trajectory_publisher /
                        call_execute_scenario / destroy_actors /
                        scenario_contract_check (跨仓契约校验)

> ego 注入链工具 (wait_for_ready / set_route / trajectory_publisher) 均注册为
> console_scripts, 由 scenario_monitor 按 ament 安装前缀调用, 不依赖源码目录形状.
> 场景目录只保留 .xosc / .json 数据资产.

## 路径参数化 (多仓分离 / CI 注入点)

harness 只通过三个资产根访问外部数据, 不拼源码/工作区绝对路径:

| 根 | yaml | CLI 参数 | 内容 |
|---|---|---|---|
| 场景库 | `paths.scenario_root` | `scenario_root:=` | `xosc_list.json` / `*.xosc` / ego 轨迹 json |
| 故障库 | `paths.fault_root` | `fault_root:=` | `*_fault_presets.yaml` / `*_fault_matrix.json` |
| 结果库 | `paths.results_root` | `results_dir:=` | 判定 JSON + `run_summary.json` |

约定: 场景库/故障库内部互相引用一律写**相对路径** (相对各自根目录),
绝对路径仅保留在环境基线字段 (`environment.autoware_install` 等)。

```bash
# CI: 场景库 checkout 到任意路径 + 外部配置 + 独立结果目录
ros2 launch autoware_srunner autoware_srunner_launch.py \
  asr_config:=$CI_PROJECT_DIR/ci/sim.yaml \
  scenario_root:=$CI_PROJECT_DIR/scenario_lib \
  fault_root:=$CI_PROJECT_DIR/scenario_lib/fault \
  results_dir:=$CI_PROJECT_DIR/out \
  xosc_list:=smoke/xosc_list.json
```

## 跨仓契约校验 (CI 提前拦)

拆仓后隐式保证消失, 用 `scenario_contract_check` 在跑仿真前做秒级文件校验
(纯 stdlib + yaml, 不 import rclpy, 可在 build 之前直接跑源码):

```bash
python3 src/autoware_srunner/autoware_srunner/scenario_contract_check.py \
  --scenario-root "$SCN_LIB" --fault-root "$SCN_LIB/fault" \
  --mapping-dir "$AUTOWARE_SRC/.../autoware_carla_interface/config" \
  --strict --json contract.json      # 退出码 0=通过 1=违约束
```

| 组 | 校验内容 | 不拦的后果 |
|---|---|---|
| C1 配置自洽 | 资产根存在; `ego_role_names` 含 `ego_role_name`; `supported_sensors` | 就绪信号永不产生 |
| C2 场景清单 | xosc_list 每条 file 可解析; name 唯一 | 启动即 RuntimeError |
| C5 故障库 | presets_file / preset / sensor / scenario 可解析 | 加载矩阵即抛错 |
| C3 场景结构 | ego ScenarioObject 名 = `ego_role_name`; catalog 条目命中; `ego_trajectory_ref` 同名 json 存在; 有 `criteria_` 判据 | 附着超时 / 注入链静默降级 / 只能拿到 exit 3 |
| C4 传感器三侧 | yaml 标签 = catalog 条目后缀 = `sensor_mapping_<tag>.yaml`; catalog 的 sensor_id 覆盖 mapping 的 id | 感知无数据 |
| C6 结果目录 | 残留 json | 按 mtime 取最新 -> 上次结果冒充本次 |
| C7 版本照片 | 场景库/故障库/autoware 的实际 commit 与 `versions.lock.yaml` 一致; scenario_runner 补丁 sha256 未被覆盖 | 跑的与记录的不是同一份, "昨天绿今天红" 无法归因 |

## 版本照片 (versions.lock.yaml)

三个仓 + 被测物各记一个版本, 跑测试前先对账。lock 只声明不改变任何东西,
由 `--lock` 交给 C7 执行 (默认自动定位本文件同级 `versions.lock.yaml`):

| 对象 | 锁什么 | 为什么 |
|---|---|---|
| 场景库 / 故障库 | `commit` + `allow_dirty` | 加删测例会改变结论; 开发期允许脏 (WARN), CI 用 `--strict` 变 FAIL |
| autoware | `commit` | 被测物换了 commit, 红线基线不再可比 |
| scenario_runner | `version` + 补丁文件 `sha256` | 它非 git 仓: 重装/覆盖会丢掉 `SCENARIO_RUNNER_SYNC_FLAG` 补丁, 表现为随机 NO_CRITERIA |

判定: commit 不符 / 目录缺失 / 非 git 仓 → FAIL; 有未提交改动且 `allow_dirty: true` → WARN。
换版本的正确姿势: 在新版本上重跑全量 → 绿了再改 lock (`改照片` 是一次显式提交)。
lock 里的 `path` 相对本文件所在目录解析, 换机器/拆仓用 `--lock` 指另一份或改 path。

## 需求 1 补丁 (scenario_runner)

`/home/carla_disk/scenario_runner-0.9.16/srunner/scenariomanager/carla_data_provider.py` 的
`set_world()` 支持环境变量 `SCENARIO_RUNNER_SYNC_FLAG`:
- e2e 联调: launch 给 carla_ros_scenario_runner 注入 `SCENARIO_RUNNER_SYNC_FLAG=false`,
  scenario_runner 只旁观不 tick (tick 单主 = autoware_carla_interface);
- 单独跑 scenario_runner: 不设该环境变量, 仍按世界实际同步状态自己 tick, 行为不变。

## 构建

```bash
cd /home/carla_disk/carla_project/autoware_srunner_ws
colcon build --symlink-install --packages-select autoware_srunner
source install/setup.bash
