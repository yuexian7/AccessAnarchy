# Access Anarchy（出入口无政府）

**都市：天际线 II (Cities: Skylines II) 代码模组 · 适配游戏版本 1.6.0f1 · 零 Harmony 补丁**

让车辆在通过停车场、仓库、商场等建筑物的出入口时不再因行人减速停车，直接穿过行人；行人的行走行为、车辆之间的避让完全不受影响。

## 它解决什么问题

原版游戏中，建筑出入口（停车场匝道、商场/仓库的连接道、停车楼坡道）是车辆与行人冲突的高发点：只要有行人横穿，车辆就会排队等待甚至堵死整个出入口。本模组按照 Anarchy 系列模组的设计哲学（"优先 ECS 数据编辑而非 Harmony 补丁"），通过编辑共享 ECS 数据实现该行为。

## 工作原理（一句话版）

游戏里车辆避让行人的唯一动态机制，是车辆读取自己车道的 `LaneOverlap`（车道重叠关系）缓冲，看到重叠的行人道上有行人登记就刹车。本模组用一个 ECS 系统**持续删除"出入口车道 ↔ 行人道"的重叠条目**——车辆看不到行人道，自然不再减速。行人自己根本不读这份数据，所以行人行为不变；车辆之间的重叠条目原样保留，所以车-车避让不变。

没有 Harmony 补丁、没有系统替换、没有改动任何游戏方法。

## 安装

开发构建已部署在本机游戏用户数据目录：

```
C:\Users\10975\AppData\LocalLow\Colossal Order\Cities Skylines II\Mods\AccessAnarchy\
```

启动游戏后在"内容管理 → 模组"里启用 **Access Anarchy** 即可。

## 设置

模组选项页（主菜单或游戏内 → 选项 → 模组 → Access Anarchy）：

| 选项 | 默认 | 说明 |
|---|---|---|
| 车辆穿过行人 | 开 | 总开关。关闭后完全恢复原版行为 |
| 作用范围 | 仅出入口区域 | `仅出入口区域` 或 `全部道路（全局）`。全局模式会禁用全市所有车让人行为，含人行横道 |
| 车库坡道 | 开 | 车库/停车楼的出入口坡道 |
| 建筑出入连接道 | 开 | 建筑/场站与路网之间的连接道（driveway） |
| 停车场与建筑内部车道 | 开 | 停车场内部行驶道、停车位道、建筑自有道路 |

设置自动保存，改动即时生效，无需重启游戏。

## 兼容性

- **零 Harmony 补丁**——不存在补丁被其他模组 `UnpatchAll` 拆掉的问题，也不与任何基于 Harmony 的模组（Traffic、Skyve 等）产生补丁冲突。
- 与修改交通行为的模组理论上可共存：本模组不持有任何缓存状态，每 4 个模拟帧重新应用一次删除，别的模组怎么改车道数据都不会导致永久性不一致。
- Realistic Path Finding 等改行人寻路的模组不受影响：行人寻路不读取本模组删除的 `LaneOverlap` 数据。

## 构建

需要游戏官方模组工具链（游戏内启用模组支持后自动配置 `CSII_*` 环境变量）：

```bash
cd D:\WorkSpace\AccessAnarchy
dotnet build -c Release
```

构建产物自动部署到 `...\Cities Skylines II\Mods\AccessAnarchy\`（含 Windows/macOS/Linux 三平台 Burst 原生库）。

## 目录结构

```
AccessAnarchy/
├── AccessAnarchy.csproj              # 官方工具链工程（net48，Mod.props/targets 导入）
├── AccessAnarchyMod.cs               # IMod 入口：注册选项页 + 注册 ECS 系统
├── Setting.cs                        # 选项面板（ModSetting 框架，中英双语）
├── Systems/
│   └── AccessZoneOverlapSystem.cs    # 核心：删除出入口车道↔行人道的 LaneOverlap
├── Properties/PublishConfiguration.xml  # Paradox Mods 发布元数据（未发布，ModId 留空）
├── research/                         # 开发期反编译调研产物（不参与编译）
└── template/                         # 官方模组模板副本（不参与编译）
```

`开发笔记.md` 记录了完整的机制调研结论、反编译证据位置和已知限制，是后续迭代的交接文档。
