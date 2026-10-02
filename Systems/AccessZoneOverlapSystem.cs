using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using Colossal.Collections;
using Colossal.Mathematics;
using Game;
using Game.Buildings;
using Game.Common;
using Game.Net;
using Game.Simulation;
using Game.Tools;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Scripting;

namespace AccessAnarchy.Systems
{
	/// <summary>
	/// 让车辆在指定范围内不再避让行人（v0.7.0，性能重构）。
	///
	/// 机制结论（对 Game.dll 1.6.0f1 反编译核实 + 实机日志验证，详见 开发笔记.md）：
	/// 车辆"看见"行人的唯一通道是车道的 LaneObject 注册缓冲——CarNavigationSystem 的
	/// CarLaneSpeedIterator 无论走 CheckCurrentLane（本车道）还是 CheckOverlappingLanes
	/// （经 LaneOverlap 指向的对端车道），最终都读某个车道的 LaneObject buffer 找行人
	/// （带 Game.Creatures.Creature 组件）并压低 CarNavigation.m_MaxSpeed。
	///
	/// v0.6.0 之前的性能问题（57km 地图帧数减半的根因）：
	/// - 干预点 A 每 4 帧对【全城】车辆车道做 chunk 迭代 + 每条 overlap 条目 2 次组件查找；
	/// - 干预点 C 每帧对【全城】行人道做 chunk 迭代 + 每实体 Curve 查找 + 3×3 网格查询。
	///   城市越大车道/行人道越多（大地图几万条），这两处是 O(全城) 的每帧/每4帧扫描。
	///
	/// v0.7.0 优化（行为不变）：
	/// 1. 邻域集合：把「类型出入口车道 + 邻近引道」预收进 NativeHashSet（双缓冲），
	///    分帧增量构建（每帧主链上处理 kSetBuildStep 个候选）。
	/// 2. access 模式 A 只遍历车辆邻域集合（StripAccessOverlapsFromSetJob），
	///    不再对全城 chunk 做 ScheduleParallel——v0.7.0 初版虽加了集合过滤，
	///    但仍是 O(全城) chunk 迭代，57km 地图帧数改善有限。
	/// 3. 删除干预点 B（连接道注册删除）——v0.3 实机日志证明连接道没有 LaneObject
	///    buffer（registration lanes = 0），它是每帧空转的冗余 job。
	/// 4. 全局模式：A 保持全城 ScheduleParallel（本就有效）。
	/// 5. 邻域集合每帧只推进一次（初版曾同帧调两次 AdvanceAccessSets）。
	///
	/// 保留的语义：
	/// - 出入口模式：人行横道（PedestrianLaneFlags.Crosswalk）条目永不删，横道避让保持原版
	///   （v0.5 的横道误伤修复——横道天然紧邻出入口）；
	/// - 全局模式：横道条目一起删（v0.7.7，见下）；
	/// - 车车 overlap、行人寻路不碰。
	///
	/// v0.7.2 性能架构重建（实机：v0.7.1 帧数仍无改善后的根因结论）：
	/// 原实现的真正热点不是「集合过滤不够」，而是【每帧和游戏抢写 buffer】：
	/// - C 每帧对邻域行人道 ECB SetBuffer 删 LaneObject，HumanNavigationSystem 随后
	///   又把行人注册回去 → 持续 structural write-war，ECB/脏 chunk/重加全在烧帧；
	/// - A 每 4 帧对全城/邻域再写一遍 overlap；
	/// - 集合重建还带 WaitForJobs()（同步等 job 收尾）+ 全城 ToEntityArray，周期性卡主线程。
	///
	/// 新架构原则：**低频一次性数据编辑，不进模拟写战**。
	/// 1. 完全删除干预点 C。车辆看见行人只靠 LaneOverlap→行人道 LaneObject；
	///    A 删掉 overlap 后车辆逻辑上读不到行人道，C 每帧删注册纯冗余，且是写战主因。
	///    （行人真走上车行道本体的 CheckCurrentLane 路径极罕见，维持原版行为。）
	/// 2. A 的重刷改为事件驱动：每帧查 m_RefreshQuery（All=[LaneOverlap, Updated]），
	///    只对刚被 LaneOverlapSystem 重建过 overlap 的车道重删。Updated 是单帧脏标记
	///    （CleanUpSystem 帧末移除），LaneOverlapSystem 在 Modification4B 阶段重建后，
	///    本系统在 GameSimulation 阶段仍能看到 → 当帧修复，零延迟。稳态匹配 0 实体。
	///    世界加载后全局模式做一次全量 A（m_OverlapQuery），access 模式由集合就绪触发。
	/// 3. 邻域集合改为低频整轮重建（v0.8.4 起由 <see cref="ShouldRebuildAccessSets"/> 决定：
	///    每 kSetRebuildCheckInterval 步看一眼"车道数变了吗 / 原版闸门开过吗"，最长 kSetRebuildFallback 步兜底一次），
	///    去掉每帧增量 IsNearAnchor；
	///    重建日志去掉（原先每轮打一行也是 I/O）。
	///
	/// v0.7.6 关闭恢复修复（实机：v0.7.5 关闭后车辆仍永远不避让）：
	/// v0.7.5 的「给车辆车道打 Updated」在原版里根本不可能生效，反编译 LaneOverlapSystem.OnUpdate 确认：
	///   - 闸门：`if (!m_UpdatedOwnersQuery.IsEmptyIgnoreFilter)`，而
	///     `m_UpdatedOwnersQuery = [SubLane, Updated, !Deleted]` —— 要的是**拥有者实体**
	///     （道路 edge / 节点 node / 建筑），车道只有 Lane 没有 SubLane 缓冲 → 闸门永远关着；
	///   - 工作清单：`nativeList = entityQuery.ToEntityListAsync()`（同一批拥有者），
	///     `UpdateLaneOverlapsJob.Execute(i)` 做的是 `UpdateOverlaps(entity, m_SubLanes[entity], ...)`，
	///     即按拥有者的 SubLane 缓冲逐条 `bufferData.Clear()` 再重算 —— 车道自己的 Updated
	///     只喂给 CollectLaneDirectionsJob / UpdateLaneFlagsJob（m_UpdatedLanesQuery），不负责重建。
	/// 所以 v0.7.5 打上的 Updated 被 CleanUpSystem 帧末清掉，删除永久残留，且「重进存档才恢复」
	/// 正是因为读档走的是 loaded=true 全量分支（m_AllOwnersQuery + m_UpdateAll=true）。
	///
	/// v0.7.6 恢复策略（关闭跳变、以及范围收窄时各一次）：
	/// 1. 主路径：反射把 LaneOverlapSystem.m_Loaded 置 true —— 下一帧该系统走与读档完全相同的
	///    全量重建分支（m_AllOwnersQuery + m_AllLanesQuery + m_UpdateAll=true），overlap 整体回到
	///    原版内容，与用户实机验证过的「重进存档就恢复避让」同一条代码路径。
	///    已复核：CS2 只有一个模拟世界（GameManager 里 new World("Game") == DefaultGameObjectInjectionWorld），
	///    LaneOverlapSystem 在 SystemUpdatePhase.Modification4B（SystemOrder.cs:178），
	///    m_Loaded 除 GetLoaded() 外无其他读者，故外部置位安全且一次性。
	/// 2. 兜底路径（反射拿不到字段 / 系统不在本世界）：给**所有拥有者实体**（SubLane 缓冲、非 Deleted，
	///    同 m_AllOwnersQuery 口径）+ **全部车道**（Lane、非 Deleted/SecondaryLane，同 m_AllLanesQuery 口径）
	///    打 Updated，走原版增量重建：拥有者开闸并进入重建清单（节点还会被 AddNonUpdatedEdgesJob
	///    带出相邻 edge），车道侧喂给 CollectLaneDirectionsJob / UpdateLaneFlagsJob 两趟，
	///    与 m_UpdateAll=true 等价（只标车辆车道会漏掉行人道侧的标记）。
	/// 3. 恢复后排一次检查 job，统计车辆车道上残存的行人 overlap 条目数写进日志，
	///    判据见下面 v0.7.8；不达标就换机制加力重试，最多 kRestoreMaxChecks 轮。
	///    注：这套恢复机制与"我们删了什么"无关——原版重建是整表 Clear 后重算，
	///    所以全局模式关闭（含横道条目被删）同样能回到原版。
	///
	/// v0.8.0 出入口仍会避让的根因（实机：个别停车场出入口偶尔仍会礼让行人，同一个停车场早前测是好的）：	/// 本系统一直把「车道自己带 Updated」当成原版重建 overlap 的事件信号（m_RefreshQuery 注释里
	/// 已就地更正那个早期假设）。逐行读反编译件 LaneOverlapSystem 后确认这个信号根本不成立：
	///   - 全文只有 4 处读 Updated（:29 AddNonUpdatedEdgesJob.m_UpdatedData、:975 UpdateLaneOverlapsJob、
	///     :1083/:1087 跨拥有者取节点、:1893-1894 两个查询定义），**没有任何一处 AddComponent&lt;Updated&gt;**；
	///   - 它重建时按 owner 的 SubLane 逐条 `bufferData.Clear()`(:1077) 再 Add(:1400/:1420)，
	///     被改写的车道不会因此带上 Updated。
	/// 车道级 Updated 的真实来源只有 LaneSystem.cs:2092（重生成车道）、RoadConnectionSystem.cs:1321
	/// （新建连接道）这类"车道被重建"的场合。于是任何一次「owner 被 Updated 而车道没被 Updated」的改写
	/// ——最典型就是停车场/建筑状态变化、出入口相邻节点被别的编辑牵动——行人条目回来后我们收不到，
	/// 只能等下一次集合整轮重建（v0.8.4 起由 ShouldRebuildAccessSets 决定：车道数一变、或原版闸门
	/// 在本周期开过，最迟 kSetRebuildCheckInterval=512 步起一轮；完全没动静时靠 kSetRebuildFallback=8192 步兜底）
	/// 由集合内批量重删捡回来；
	/// 全局模式更糟：它没有周期性重删，条目一旦回来就永远回来。
	/// "同一个停车场早先是好的"由此解释——好与不好取决于那一刻离上次整轮重建过了多久。
	///
	/// 修法：新增干预点 (0) StripRebuiltLanesJob，闸门与原版逐条同口径
	/// （m_RebuildOwnerQuery=[SubLane,Updated,!Deleted] ≡ LaneOverlapSystem.cs:1893），
	/// 遍历对象也与原版工作清单一致（owner 的 SubLane，加上 Updated 节点经 ConnectedEdge
	/// 拉进来的相邻 edge，即 AddNonUpdatedEdgesJob :26-54）。时间上成立的理由是阶段顺序：
	/// LaneOverlapSystem 在 Modification4B（SystemOrder.cs:178）早于本系统的 GameSimulation
	/// （SystemUpdatePhase 枚举顺序 Modification4B &lt; GameSimulation &lt; Cleanup），
	/// 而 Updated 要到帧末 Cleanup 才被 CleanUpSystem.cs:53 摘掉
	/// （PrepareCleanUpSystem 只是在 MainLoop 抄一份清单，不移除），
	/// 所以同一个非空状态我们在同一帧看得见 → 一帧内收敛，不再靠 34 秒兜底。
	/// 没被改写过、或改写后本来就没有行人条目的车道在计数为 0 时直接返回，不产生 ECB 写入，
	/// 因此不会退化成 v0.7.2 特意删掉的每帧写战。
	/// 顺带把判定侧的曲线采样从 3 点补到与放锚点一致的 5 点（长/弯引道的"弦中点"会偏离曲线本体）。
	/// 心跳日志新增三行自证指标：Rebuild gate / Anchor grid capacity / Access set audit，
	/// 判读方法见 ReportRebuildDiagnostics 的注释。
	///
	/// v0.8.0 第二轮 / v0.8.2 第二轮：「不避让范围」三个勾选框试图把出入口再分成
	/// 停车场 / 建筑与外部道路的车辆出入口 / 建筑内部道路三类，作者实机连续反馈"互相覆盖"
	/// （只勾第二项时 `inScope=54578`，占全城车辆车道 43%）。三轮修法（收录侧按掩码过滤 →
	/// 判定侧给锚点打类别位 → 沿 Owner 链上溯认 ParkingFacility）都没能把它分开，实测证据：
	/// ① 三类在数据上不可靠可分——`ConnectionLane` 大门道商店与停车场是同一个组件、同一套 Owner，
	///    官方只在节点对侧的 `RouteConnectionData.m_AccessConnectionType` 上区分（OutsideConnectionSystem.cs:229-236），
	///    而普查读的是车道侧，`access=0` 是盲区不是空集；
	/// ② "邻近锚点"这条 32 米兜底天生跨类——商店/住宅大门与停车场紧贴，锚点半径一盖就把另一类
	///    的车道一起扫进范围。只要兜底还在，勾选项就不可能真正独立。
	///
	/// v0.8.4（玩家反馈"游戏变卡/模拟速度变慢"，并贴出 `Anchor grid capacity hit: 233500 dropped`
	/// 在一局里出现 2157 次）：核查确认**处理机制有实质问题**，不是错觉，作者本地"帧数没怎么变"也对得上——
	/// 花掉的是每条模拟步的时间预算，不是渲染帧。三条硬证据（作者本机实机日志 2026-10-01 15:50-15:52）：
	/// <list type="number">
	/// <item>锚点池被**行人道**灌满：`Anchor sources (mask=7): access=106283 parking=4823 internal=6
	///  | anchors=75754 cells=15362 dropped=479806`。旧判据"有 ConnectionLane 就是出入口"没看 flags，
	/// 而原版给纯行人连道只打 `Inside|Pedestrian`（<c>RoadConnectionSystem.cs:1336-1351</c>），
	/// 人行道/横道则是带 `PedestrianLane` 的车道（车道类型位互斥，见 <c>Pathfind/LaneDataSystem.cs:16-131</c>）。
	/// 11.1 万条"锚点来源"里真正车行的只有约 4.8 千条 ⇒ 每轮要试插 55 万个锚点、86% 被容量截断，
	/// 判定侧邻格内层循环**永远跑满**。同一份日志里只勾停车场那档是 `anchors=8798 cells=1504`（1/10 的格子）。</item>
	/// <item>范围因此爆掉：同一张图 `inScope=65523 / 128042` 条车道 = 全部 overlap 车道的 51% 被当作"出入口区域"，
	/// 邻域集合 6.5 万条；这也是作者此前实机"很混乱"的另一半原因（行人连道在整条街到处放锚点）。</item>
	/// <item>稳态里这些活是**白做**的：连续 15 个心跳周期 `Rebuild gate: 0 frame(s) ... re-stripped 0`，
	/// 也就是原版 overlap 一次都没重算过，而集合/锚点整轮重建仍每 2048 步无条件跑一遍
	/// （≈1× 速度 17 秒一轮，一轮 = 全城 12.8 万候选 × 5 采样 × 9 邻格 + 55 万次锚点插入，占一条 worker 线程）。</item>
	/// </list>
	/// 修法四条：① 出入口判据与锚点来源一律**只认车行车道**（有 `PedestrianLane` 直接出局；
	/// `ConnectionLane` 必须带 Road/Track/Parking 之一，与全局模式从 v0.5 起就用的同一组位统一）；
	/// ② 整轮重建改为条件触发（<see cref="ShouldRebuildAccessSets"/>：车道数变化 / 原版闸门开过 / 
	/// 模式切换立即重建，完全没动静时 8192 步兜底一次）；③ 同格 6 米内锚点去重，让容量截断回到"真塞不下"才发生；
	/// ④ 心跳诊断只在数值变化时写一行，`dropped==0` 时降级为 INFO，不再刷屏。
	/// ⚠ 顺带消掉一个一直存在、没人报的隐患：行人道自己的 `LaneOverlap` 是有读者的——
	/// `TrafficLightInitializationSystem.cs:515-530` 用 `m_Overlaps[subLane]` 给**行人车道做信号灯相位分组**，
	/// 旧判据在出入口模式会把行人连道当"持有者"、删掉它们指向行人道的条目，等于动了红绿灯分组。现在不再触碰。
	/// 车辆侧不读行人道的 LaneOverlap（`HumanNavigationSystem` 全文只把缓冲传给 job、没有读取行），
	/// 所以这次收紧不会削弱"车不让人"的效果。
	///
	/// v0.8.3 三合一（作者拍板："干脆不要区分了"）：三个勾选项、IncludeMask、锚点类别位、
	/// Scope census 普查全部删除。范围只由「取消避让范围」下拉框决定：
	/// 全局 = 所有道路 + 人行横道；出入口 = 所有类型出入口/停车道 + 邻域集合 + 邻近锚点，
	/// 三条判据收敛到 <see cref="IsAccessOrFacilityLane"/> 与 <see cref="IsInNoYieldScope"/> 一处。
	/// 网格值回到 `float2`（无类别位）⇒ 每格容量从 10 涨回 14。
	/// 横道仍然只在全局分支参与（`m_IncludeCrosswalks = globalMode ? 1 : 0`），出入口模式一条不动。
	///
	/// v0.8.2 第三轮（作者实测「全部道路切回仅出入口后，人行横道再也不避让行人，只能关总开关才好」）：
	/// **模式从全局切回出入口** 那一支曾写成恒假条件（`m_LastMode == kModeGlobal && !toAccessMode`：
	/// 都离开全局了还要"不 toAccessMode"，永远为假），于是全局期删掉的横道/整城条目没有任何一次
	/// 还原动作 → 正是作者看到的"只有总开关关掉再打开才正常"（关闭路径 RestoreVanillaOverlaps 不看
	/// narrowed，无条件重建，所以它好使）。现改为 `m_LastMode == kModeGlobal && Mode != kModeGlobal`。
	/// 三合一后这是**唯一**的范围收窄来源，别再把它改回恒假；实机验证看日志这一行：
	/// `AccessZoneOverlapSystem scope narrowed: requested vanilla FULL LaneOverlap rebuild (...)`。
	/// 开关变化走 v0.7.6/0.7.8 的「先让原版整表重建、再按新范围重删」路径，
	/// 范围变窄会在 kReapplyDelayFrames(120 帧) 后恢复原版避让，不会留下永久剥离态。
	///
	/// v0.7.8 出入口模式恢复判据修正（实机：出入口模式下只要开启过就永远不避让）：
	/// 用户日志实据（Logs\AccessAnarchy.AccessAnarchyMod.log，2026-09-19 15:41:33 与 15:43:26 两轮）：
	/// 那两次"Restore audit: ... vanilla avoidance restored"报的是 54510 车道 / 73063 条目，
	/// 其中横道就占 71915 条 → 非横道只剩 1148 条，而正常全量重建后是 96003 条（非横道 24088）。
	/// 也就是说：出入口模式下原版重建那次根本没落地，我们删掉的非横道条目一条都没回来，
	/// 但 v0.7.7 的判据是"只要有行人条目就算恢复"——出入口模式我们从不动横道，
	/// 横道那几万条永远在场，判据必被满足 → 既不误报失败也不触发兜底重试，
	/// 表现就是"关不掉、永远不避让"。
	/// 修法两点：① 判据改为看"这个模式下我们一定删过的那一类"——出入口模式看非横道条目数>0，
	/// 全局模式看横道条目数>0，另加一道"总量回到基线 8 成"的闸（基线=启用后第一遍删之前扫到的条目数）；
	/// ② 不达标不再只试一次，而是按 kRestoreCheckDelays(30/90/240/600/1200 帧)
	/// 逐次放大间隔重试至多 kRestoreMaxChecks 轮，第 1 轮再推 m_Loaded，第 2 轮起同时补打
	/// owner+lane Updated（两套机制交替，防单点被游戏吞掉），每轮都把三个计数写日志。
	/// 加时窗的原因：GameSimulation 一帧渲染内可连追多步模拟，而 LaneOverlapSystem 在 MainLoop
	/// 的 Modification4B，只等 6 帧可能在"它一次都没轮到"的窗口里就测完了（那两轮恰好只有 15ms）。
	///
	/// v0.7.7 全局模式补横道（实机：v0.7.6 出入口穿透与开关恢复都正常了，但「全部道路」下
	/// 车辆在人行横道仍然永远让行人）：干预点 A 的 Crosswalk 排除原本无条件生效，
	/// 等于把「全部道路」缩水成「全部道路除了横道」，与 12 种语言 mode.desc 早已写明的
	/// 「包括人行横道」自相矛盾。现按模式取值：StripPedestrianOverlapsJob.m_IncludeCrosswalks
	/// = 全局 1 / 出入口 0（StripAccessOverlapsFromSetJob 只在出入口跑，恒 0）。
	/// 机制复核：车辆让横道人走的仍是 LaneOverlap→对端 LaneObject→CheckPedestrian 这一条路
	/// （research/Game.Simulation.CarLaneSpeedIterator.cs:1174 取本车道 LaneOverlap、
	/// :1193 遍历条目、:1346 对端 LaneObject 带 Creature → :1348/:1403 CheckPedestrian 压速），
	/// 原版没有"横道专用"的车辆让行通道，删掉 car→crosswalk 条目即可；
	/// 信号灯路口的停车是 LaneSignal，与本模组无关（维持原版）。
	/// 全局→出入口 的切换属于"范围收窄"，由 v0.7.6 的还原重删路径把横道条目要回来。
	/// </summary>
	[Preserve]
	public partial class AccessZoneOverlapSystem : GameSystemBase
	{
		/// <summary>多久检查一次"要不要整轮重建"（模拟步）。检查本身只有两次
		/// CalculateEntityCount（按 archetype 数，不遍历实体）与一个标志位，成本可忽略。</summary>
		private const int kSetRebuildCheckInterval = 512;

		/// <summary>原版重建闸门刚开过时，距离**上一轮整轮跑完**至少隔这么多步才允许再起一轮（v0.8.5）。
		/// 施工期间闸门可能连着开，不设间隔就会一轮接一轮地全城扫描：一轮本身要跑
		/// ceil(候选数 / kSetBuildStep) 步（实机 12.8 万候选 ≈ 63 步），间隔必须明显大于一轮时长，
		/// 否则占空比直接冲到 50% 以上。256 步（1× 速度约 2 秒）既能把"电梯/建筑状态变化"这类
		/// 事件在几秒内跟到，又比 kSetRebuildCheckInterval 的快速通道还快一倍。</summary>
		private const int kGateRebuildMinGap = 256;

		/// <summary>兜底周期：即使没探测到任何变化信号，最多隔这么多步也整轮重建一次。
		/// v0.8.3 及以前这里是**无条件**每 2048 步重建一轮（≈1× 速度 17 秒），而作者实机日志显示
		/// 稳态城市里连续 15 个心跳周期 「Rebuild gate: 0 frame(s)」 —— 原版一次都没重算过 overlap，
		/// 那一轮"全城 12.8 万条候选 × 5 采样 × 9 邻格 + 55 万次锚点插入"全是白做的，
		/// 玩家反馈的模拟速度变慢就来自这里。现在只在**有变化信号**（车道数变了 / 原版重建闸门开过 /
		/// 模式或范围切换）时重建，剩下的用这个长兜底保证极端漏检也能自愈。</summary>
		private const int kSetRebuildFallback = 8192;

		/// <summary>同一网格单元内两个锚点靠得比这根距离还近就视为重复，只留第一个。
		/// 停车场/坡道的曲线采样点非常密（实机：每格平均 16 个采样点、闸门 14 个），
		/// 不去重就会长期"容量截断"，既让 dropped 永远是正数（玩家看到 2157 次 WARN 就是这么来的），
		/// 又让判定用的邻格内层循环一直跑满。6 米远小于 32 米判定半径，对范围形状的影响可忽略。</summary>
		private const float kAnchorDedupDistSq = 36f;

		/// <summary>空间判定的有效半径（米）。锚点落在 32m 网格格子里，判定时校验实际距离。</summary>
		private const float kAccessRadius = 32f;

		private const float kGridCellSize = 32f;

		/// <summary>每帧在主链上处理的候选实体数（集合分帧构建的步长）。</summary>
		private const int kSetBuildStep = 2048;

		/// <summary>每隔多少个模拟帧输出一次心跳日志（诊断用）。</summary>
		private const int kHeartbeatInterval = 1800;

		/// <summary>关闭恢复后第 N 次检查前等待的模拟帧数（逐次放大）。
		/// 不能只等几帧：GameSimulation 一帧渲染内可连追多步模拟，而 LaneOverlapSystem 在
		/// MainLoop 的 Modification4B，追帧 burst 里它可能一次都没轮到——
		/// v0.7.7 实机日志就出现过"关闭后 15ms 就测完、看到的还是剥离态"。首档 30 帧、逐档放大。</summary>
		private static readonly int[] kRestoreCheckDelays = new int[5] { 30, 90, 240, 600, 1200 };

		/// <summary>恢复检查最多做几轮（每轮不达标就换机制再推一次重建）。</summary>
		private const int kRestoreMaxChecks = 5;

		/// <summary>范围收窄后等多久再按新范围重删。必须给足整帧边界：v0.7.7 复核确认
		/// GameSimulation 一帧渲染内可连追多步模拟，而 LaneOverlapSystem 在 MainLoop 的
		/// Modification4B，等待太短会出现"我们先重删、原版重建随后把它冲掉"，
		/// 表现就是切了范围没反应。代价是切换后约 1-2 秒才收敛。</summary>
		private const int kReapplyDelayFrames = 120;

		/// <summary>审计状态机：闲置 / 等帧（到点后同步跑完，不跨帧）。</summary>
		private const int kAuditIdle = 0;
		private const int kAuditWaitFrames = 1;

		/// <summary>审计计数每块占几个 int：车辆车道数、行人条目数、横道车道数、横道条目数。</summary>
		private const int kAuditCountersPerChunk = 4;

		/// <summary>重删计数的**初始** chunk 槽位数（每块两格：该 chunk「修好的车道数」「删掉的行人条目数」）。
		/// 槽位是 `unfilteredChunkIndex * 2` 精确独占的（见 <see cref="EnsureRepairCapacity"/> 与
		/// StripRebuiltLanesJob.m_RepairCounters 上的 [NativeDisableParallelForRestriction]），
		/// 不像早期版本按 &amp; 63 取模撞槽，所以计数是准确值而不是下界；
		/// 实际容量会按当帧 chunk 数扩容，这个常量只是首次分配的大小。计数仍不参与任何行为判断。</summary>
		private const int kRepairSlots = 64;

		/// <summary>ECB 回放排序键的键空间划分（v0.8.2 稳定性/正确性修正）。
		/// 同一个 EndFrameBarrier 的 ParallelWriter，一帧内可能有多批 job 对**同一条车道**下 `SetBuffer&lt;LaneOverlap&gt;`：
		/// 键值相同则两条命令的先后顺序不确定，而 SetBuffer 是**整表覆盖**——被旧快照后回放就会把
		/// 已经不该删的条目又写回去（表现为"个别出入口偶发仍然不避让"，反过来也会让还原失效）。
		/// 现在按 OnUpdate 里的**调度先后**分配互不重叠的基址：回放顺序＝执行顺序，最后跑的那批
		/// （每帧事件驱动重刷，手里的快照最新）拿到最大的键，一定覆盖前面的。</summary>
		private const int kGateSortKeyBase = 0;

		private const int kSortKeyInitialPass = 1000000;

		private const int kSortKeyFromSet = 2000000;

		private const int kSortKeyRefresh = 4000000;

		/// <summary>关闭功能的兜底重建（MarkOwnersUpdatedJob / MarkLanesUpdatedJob）自己的基址。
		/// 这两批写的是 `AddComponent&lt;Updated&gt;`、与上面三批的 `SetBuffer&lt;LaneOverlap&gt;` 不是同一组件，
		/// 而且恢复沿与剥离沿在同一次 OnUpdate 里互斥（各自 `return`），本来就不会同帧撞键；
		/// 以前它们直接拿裸 `unfilteredChunkIndex` 当键，等于压在 kGateSortKeyBase=0 的键空间里，
		/// 只靠"调用路径不同帧"这条隐式约束成立。v0.8.3 给它独立基址，把约束写死在常量里。</summary>
		private const int kSortKeyRestore = 6000000;

		/// <summary>锚点网格诊断槽（BuildAnchorGridJob 单线程写，心跳读）。
		/// v0.8.3 三合一：4~6 号"按勾选项分类的来源计数"槽随普查一起删了。
		/// v0.8.4 玩家性能反馈回来时重新开了两格，但这次数的是**车行来源**与**被排除的行人道**
		/// ——用于自证"锚点池不再被人行道/横道灌满"。</summary>
		private const int kGridDropped = 0;
		private const int kGridMaxCell = 1;
		private const int kGridAnchors = 2;
		private const int kGridCells = 3;
		private const int kGridSources = 4;
		private const int kGridPedSkipped = 5;
		private const int kGridDeduped = 6;

		private const int kGridStatsSlots = 7;

		/// <summary>车行连接道的判定位。原版给建筑连道打 flags 时（RoadConnectionSystem.cs:1336-1404）
		/// 纯行人连道**只有** Inside|Pedestrian，车行/轨道/停车行才有下面这三位之一，
		/// 所以这三位是"这条连道确定是车走的"的官方判据（原版自己也是这么判的，
		/// VehicleUtils.cs:491-503）。全局分支从 v0.5 起就在用同一组位，出入口分支以前没用过。
		/// ⚠ 但它**不是**"不是出入口"的判据：`RouteConnectionType.Cargo` 的连道只有
		/// Inside|AllowCargo（:1354），所以命中这三位=立即是出入口，没命中只=继续按组件判
		/// （见 IsAccessOrFacilityLane 的 v0.8.5 修正）。</summary>
		private const ConnectionLaneFlags kVehicleConnectionFlags = ConnectionLaneFlags.Road | ConnectionLaneFlags.Track | ConnectionLaneFlags.Parking;

		/// <summary>每个网格单元最多存几条锚点——这是**我们设的闸门**（放第 15 条之前就停手并计入
		/// <see cref="kGridDropped"/>），不是容器容量：<c>FixedList128Bytes&lt;float2&gt;</c> 本体是 128 字节硬上限，
		/// 减长度前缀按 8 字节/条约可放 15 条，闸门取 14 留一格余量，绝不指望容器替我们越界检查。
		/// 历史日志里出现过的 `max 14 per cell` 就是这个值，别按容器容量去对。
		/// v0.8.3 之前为了记"这条锚点属于哪个勾选项"曾换成 <c>float3</c>（12 字节/条，同样算法只剩 10 格），
		/// 三合一之后类别位没有读者了，回到 <c>float2</c> 拿回 4 格。</summary>
		private const int kAnchorPerCell = 14;

		private SimulationSystem m_SimulationSystem;
		private EndFrameBarrier m_EndFrameBarrier;
		private EntityQuery m_OverlapQuery;
		private EntityQuery m_AnchorQuery;

		/// <summary>事件驱动重刷查询：匹配刚被 LaneOverlapSystem 重建过 overlap 的车道（带 Updated 标记）。
		/// 注意 v0.8.0 的更正：这条路径**不是**主要机制。反编译 LaneOverlapSystem 全文，
		/// Updated 只以 ComponentLookup 形式被读取（:29 :975 :1083 :1087 :1893-1894），
		/// 该系统从不给车道 AddComponent&lt;Updated&gt;，所以"原版重建完 overlap 后车道还带着 Updated，
		/// 我们当帧重删"这个早期假设不成立。车道级 Updated 的真正来源是
		/// LaneSystem.cs:2092 与 RoadConnectionSystem.cs:1321（新建/重生成车道），
		/// 这条查询只覆盖到那批实体；其余被原版改写过 overlap 的车道由 m_RebuildOwnerQuery 兜住。
		/// 稳态下匹配 0 实体，成本 ≈ 0。</summary>
		private EntityQuery m_RefreshQuery;

		/// <summary>v0.8.0 主要重删闸门：与原版 LaneOverlapSystem.m_UpdatedOwnersQuery 逐条同口径
		/// （All=[SubLane,Updated]，None=[Deleted]，见 LaneOverlapSystem.cs:1893，闸门 :1918-1920）。
		/// 原版每帧只要这个查询非空，就会把清单内实体（以及 Updated 节点经 ConnectedEdge 拉进来的
		/// 相邻 edge，AddNonUpdatedEdgesJob :26-54）的 SubLane 逐条 bufferData.Clear() 后重算 overlap
		/// （UpdateOverlaps :1038-1043 / :1077）——被删掉的行人条目就是在这一刻回来的。
		/// 该阶段（Modification4B，SystemOrder.cs:178）早于本系统的 GameSimulation，而 Updated
		/// 要到帧末 Cleanup 才被摘掉（PrepareCleanUpSystem 只在 MainLoop 抄清单，
		/// CleanUpSystem.cs:53 才是移除点，SystemUpdatePhase 枚举顺序 Modification4B &lt; GameSimulation &lt; Cleanup），
		/// 所以本系统在**同一帧**能看到同一个非空状态 → 按同一批实体重删，一帧内收敛。
		/// 稳态（无人修路、无建筑状态变化）匹配 0 实体，成本 = 一次 IsEmptyIgnoreFilter。</summary>
		private EntityQuery m_RebuildOwnerQuery;

		/// <summary>关闭恢复兜底用：网络"拥有者"实体（道路 edge / 节点 node / 建筑，即带 SubLane 缓冲者），
		/// 与原版 LaneOverlapSystem.m_AllOwnersQuery 同口径（那里只有 SubLane + !Deleted）。
		/// 额外排除 Lane / LaneOverlap，使其与 m_OverlapQuery（车辆车道）互斥，
		/// 避免同一实体在同一个 ECB 里被 AddComponent&lt;Updated&gt; 两次。</summary>
		private EntityQuery m_OwnerQuery;

		/// <summary>关闭恢复兜底用的车道侧查询，与原版 LaneOverlapSystem.m_AllLanesQuery 同口径
		/// （Lane + !Deleted + !SecondaryLane）：包含行人道，使 m_UpdatedLanesQuery 那两趟
		/// （CollectLaneDirectionsJob / UpdateLaneFlagsJob，刷新 CarLane Yield/Stop/Approach 等
		/// overlap 派生标记）覆盖全部车道，而不只是车辆车道。与 m_OwnerQuery 互斥（那边排除了 Lane）。</summary>
		private EntityQuery m_RestoreLaneQuery;

		/// <summary>LaneOverlapSystem.m_Loaded 的反射句柄（私有字段，进程级缓存一次）。
		/// 置 true = 让原版系统下一帧走读档同款全量重建。</summary>
		private static FieldInfo s_LoadedField;
		private static bool s_LoadedFieldResolved;

		/// <summary>关闭恢复检查状态机（见类注释 v0.7.6 第 3 点 / v0.7.8）。检查同步完成，不留跨帧句柄。</summary>
		private int m_AuditPhase;
		private int m_AuditFrames;

		/// <summary>本轮关闭已做了几次恢复检查（1 起）。</summary>
		private int m_AuditChecks;

		/// <summary>关闭那一刻是否处于全局模式：决定检查该看哪一类条目——
		/// 出入口模式横道条目从没被删过，只有"非横道行人条目回来了"才算恢复。</summary>
		private bool m_RestoreCheckGlobal;

		/// <summary>原版基线计数：启用后第一遍（还没删过任何东西时）扫到的行人条目数/横道条目数。
		/// 系统实例随世界重建，所以它就是"这个存档本来的样子"，关闭后拿它当标尺判恢复。</summary>
		private bool m_BaselineTaken;
		private int m_BaselineEntries;
		private int m_BaselineCrosswalk;

		/// <summary>范围/模式切换后等待原版重建落地的帧数；期间不删，等重建完成再按新范围重删。
		/// 没有这一步，global→access 的收窄会把先前删掉的条目永久留在范围外车道上
		/// （和"关不掉"是同一个根因：删除不会自动还原）。</summary>
		private int m_ReapplyCountdown;

		/// <summary>启用后第一次读到模式/范围：不做还原重刷（读档数据本来就是干净的）。</summary>
		private bool m_ScopeObserved;

		private NativeHashMap<int2, FixedList128Bytes<float2>> m_AnchorGrid;

		/// <summary>邻域集合（双缓冲）：偶数轮 A/B，奇数轮 B/A。Item1 = 活跃（判定用），Item2 = 构建中。</summary>
		private NativeHashSet<Entity>[] m_VehicleSets;
		private int m_ActiveSetIndex;

		/// <summary>分帧构建的候选列表与游标（-1 表示闲置，需要起新一轮）。</summary>
		private NativeList<Entity> m_Candidates;
		private int m_Cursor;
		private bool m_SetsInitialized;
		private bool m_BuildInProgress;

		/// <summary>范围/模式改变或关闭后：活跃集里还留着旧范围的成员，而事件驱动重刷 job
		/// 无条件用活跃集（v0.7.7 复核发现的漏口——收窄范围内这些旧成员会被继续重删）。
		/// 置位后在下一轮集合起始处（已 WaitForJobs，无在飞 job）把两侧一起清空，
		/// 过渡期内判定退化为"类型 + 锚点邻域"，不会误删也不会漏判。</summary>
		private bool m_SetsStale;

		/// <summary>全局模式世界加载后的首次全量 A 是否已完成。完成后只走 Updated 事件驱动。</summary>
		private bool m_InitialGlobalApplyDone;

		/// <summary>上一帧观察到的启用状态。用于捕捉 启用→关闭 / 关闭→启用 跳变：
		/// 关闭时必须触发原版 LaneOverlapSystem 重建（恢复行人条目），
		/// 否则我们删掉的 overlap 永远不会被还原（它只在拥有者实体带 Updated 脏标记或读档时才重建）。
		/// 注意 v0.7.5 曾以为"给车道打 Updated"即可，实测无效，见类注释 v0.7.6。</summary>
		private bool m_WasEnabled;

		/// <summary>本局是否已经打过键位诊断（Setting.ReportKeyBindings "in-game"）。
		/// 只能在系统跑起来之后打第一次：那时所有模组都注册完自己的动作了，
		/// 游戏自己也在 UIReady 后算过 hasConflicts，冲突判定才有意义。</summary>
		private bool m_KeyReported;

		/// <summary>上次观察到的作用模式/范围开关，变化时强制重建集合。</summary>
		private int m_LastMode = -1;

		private bool m_LoggedFirstRun;
		private ulong m_TotalPasses;

		/// <summary>重删诊断计数：由 StripRebuiltLanesJob 写，心跳时读完清零。
		/// v0.8.2 稳定性修正：以前固定 64 槽、按 `chunkIndex % 64` 取槽 ⇒ 不同线程会写**同一个 int**
		/// （Burst 下是无同步的并发写，虽只是诊断值，但属于实打实的数据竞态）。
		/// 现在按 chunk 数分配，每块独占两格，永不重叠；容量不够时在 `WaitForJobs()` 之后扩容。</summary>
		private NativeArray<int> m_RepairCounters;

		/// <summary>m_RepairCounters 当前能容纳多少个 chunk（容量，不是元素数）。</summary>
		private int m_RepairCounterChunks;

		/// <summary>锚点网格诊断计数（见 kGridStatsSlots）：BuildAnchorGridJob 单线程写，心跳时读。</summary>
		private NativeArray<int> m_GridStats;

		/// <summary>本心跳周期内原版重建闸门被打开的帧数，以及最后一帧的 owner 数（主线程计数，无竞态）。</summary>
		private int m_GateFrames;
		private int m_GateLastOwners;

		/// <summary>上一次已经写进日志的几组诊断值（-1 = 还没写过）。心跳是每 ~1800 步一次，
		/// 一局高倍速能跑到两千次，值不变还每次都写 = 玩家眼里"模组一直在报警"（v0.8.4 的直接起因）。
		/// 只在数字变化时写一行，读档/新世界由 ResetWorldCaches 把它们打回 -1 保证至少各写一次。</summary>
		private int m_LastGateFrames = -1;
		private int m_LastGateLanes = -1;
		private int m_LastGateEntries = -1;
		private int m_LastGridSources = -1;
		private int m_LastGridAnchors = -1;
		private int m_LastGridCells = -1;
		private int m_LastGridDropped = -1;
		private int m_LastAuditSet = -1;
		private int m_LastAuditLanes = -1;
		private int m_LastAuditEntries = -1;

		/// <summary>上一轮整轮重建开始时看到的两批实体数，以及"之后原版重建闸门有没有开过"。
		/// 这是 v0.8.4 的变化探测器：施工/拆建必然改动带 LaneOverlap 的车道数或带曲线的出入口车道数，
		/// 而任何一次原版重算 overlap 都会被 m_RebuildOwnerQuery 抓到。</summary>
		private int m_LastAnchorCount = -1;
		private int m_LastCandidateCount = -1;
		private bool m_GateSeenSinceRebuild;
		private uint m_LastRebuildFrame;

		/// <summary>整轮重建的实际发生次数与被"没变化"挡掉的检查次数（只用于心跳自证）。
		/// 稳态城市里应当看到 rounds 不再增长、skipped 持续增长——这就是 v0.8.4 那条修复的证据。</summary>
		private int m_RebuildRounds;
		private int m_RebuildSkips;
		private int m_LastLoggedRounds = -1;
		private int m_LastLoggedSkips = -1;

		/// <summary>
		/// ⚠ 本系统唯一的 job 依赖链，**不能用 <see cref="Game.GameSystemBase.Dependency"/> 代替**
		/// （原生闪退自查 2026-09-30 查出的根因级问题，比并发写 HashSet 更致命）：
		/// <list type="bullet">
		/// <item>《Game.Simulation.SimulationSystem》一个渲染帧里会跑 **1–8 个模拟步**
		/// （ilspycmd 实读：`for (int i = 0; i &lt; num; i++) … m_UpdateSystem.Update(SystemUpdatePhase.GameSimulation, frameIndex, i)`，
		/// `num` 上限 `num7 = max(1, min(8, selectedSpeed * … * 2))` ⇒ **普通 1× 速度就会追 2 步**）；</item>
		/// <item>《Game.UpdateSystem.cs:247-249》对每个系统：`if (systemData.m_ResetInterval &lt;= iterationIndex) ResetDependency();`
		/// 而 `m_ResetInterval = (system is GameSystemBase) ? interval : int.MaxValue`（:36），`interval = GetUpdateInterval(phase)`
		/// 我们没覆写 ⇒ 1（《Game/GameSystemBase.cs:131-134》）⇒ **第 2 步及以后，进 OnUpdate 之前 `Dependency` 已被清成 default**；</item>
		/// <item>《GameSystemBase.cs:141-144》`ResetDependency()` 就是 `Dependency = default`，而排出去的 job 只注册给
		/// 《EndFrameBarrier》（它的 `OnUpdate` 才 `producerHandle.Complete()`）⇒ 上一步的 job 与本步主线程的
		/// `Clear()` / `Dispose()` / 扩容**并发跑在同一块 Allocator.Persistent 内存上**：
		/// 一个线程 free、另一个沿旧指针写 ⇒ Unity 持久分配器的池被写坏，之后**任何**模组或原版的 job 拿到重叠内存就 AV，
		/// 崩点与肇事点分离、托管栈为空——与实机「三次同一指令同一坏值」的形状一致。</item>
		/// </list>
		/// 结论：所有排程一律串到本字段上，所有"等我的 job 跑完"都走 <see cref="WaitForJobs"/>。
		/// </summary>
		private JobHandle m_JobChain;

		/// <summary>把游戏留在 <c>Dependency</c> 上的句柄吸收进自持链（第 1 步游戏不清零、第 2 步起是 default，
		/// 两种情况都要吸，否则可能漏掉上一帧最后一步排出的 job）。</summary>
		private void SyncJobChain()
		{
			m_JobChain = JobHandle.CombineDependencies(m_JobChain, Dependency);
		}

		/// <summary>等本系统排出去的全部 job 收尾。**这是唯一可信的同步点**：直接
		/// <c>Dependency.Complete()</c> 在多步模拟的第 2 步之后是空转（见 m_JobChain 注释）。</summary>
		private void WaitForJobs()
		{
			SyncJobChain();
			m_JobChain.Complete();
		}

		/// <summary>
		/// 取本帧的命令缓冲，接住游戏的拒绝。机制：本模组的 ECB 挂在《Game.EndFrameBarrier》上，
		/// 它是《Game.SafeCommandBufferSystem》（ilspycmd 实读），<c>CreateCommandBuffer()</c> 在
		/// <c>m_IsAllowed == false</c> 时**直接 throw new Exception**（该标志由 barrier 自己的 OnUpdate 置 false、
		/// 由游戏的 AllowBarrier&lt;EndFrameBarrier&gt; 置 true）。《Game.UpdateSystem.cs:253-261》会把异常吞成
		/// 一条 Critical 日志继续跑，但那时本步已经排出去的 job 就永远进不了 <c>AddJobHandleForProducer</c>，
		/// 与 m_JobChain 注释里那个"每步清零"叠起来正是无人等待的在飞 job。所以这里宁可接住：
		/// 先把已排的 job 等收尾（容器就不会被并发读写），再放弃本帧的删除批。
		/// </summary>
		private bool TryCreateBarrierBuffer(out EntityCommandBuffer.ParallelWriter ecb)
		{
			try
			{
				ecb = m_EndFrameBarrier.CreateCommandBuffer().AsParallelWriter();
				return true;
			}
			catch (Exception ex)
			{
				ecb = default(EntityCommandBuffer.ParallelWriter);
				WaitForJobs();
				AccessAnarchyMod.log.Warn($"EndFrameBarrier refused a command buffer, this step's strip batch skipped: {ex.Message}");
				return false;
			}
		}

		[Preserve]
		protected override void OnCreate()
		{
			base.OnCreate();
			m_SimulationSystem = World.GetOrCreateSystemManaged<SimulationSystem>();
			m_EndFrameBarrier = World.GetOrCreateSystemManaged<EndFrameBarrier>();
			m_AnchorGrid = new NativeHashMap<int2, FixedList128Bytes<float2>>(4096, Allocator.Persistent);
			m_VehicleSets = new NativeHashSet<Entity>[2]
			{
				new NativeHashSet<Entity>(8192, Allocator.Persistent),
				new NativeHashSet<Entity>(8192, Allocator.Persistent)
			};
			m_Candidates = new NativeList<Entity>(65536, Allocator.Persistent);
			m_Cursor = -1;
			m_RepairCounterChunks = kRepairSlots;
			m_RepairCounters = new NativeArray<int>(m_RepairCounterChunks * 2, Allocator.Persistent, NativeArrayOptions.ClearMemory);
			m_GridStats = new NativeArray<int>(kGridStatsSlots, Allocator.Persistent, NativeArrayOptions.ClearMemory);
			m_OverlapQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[1]
				{
					ComponentType.ReadOnly<LaneOverlap>()
				},
				Any = new ComponentType[5]
				{
					ComponentType.ReadOnly<CarLane>(),
					ComponentType.ReadOnly<ParkingLane>(),
					ComponentType.ReadOnly<GarageLane>(),
					ComponentType.ReadOnly<ConnectionLane>(),
					ComponentType.ReadOnly<AreaLane>()
				},
				None = new ComponentType[2]
				{
					ComponentType.ReadOnly<Deleted>(),
					ComponentType.ReadOnly<Temp>()
				}
			});
			m_AnchorQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[1]
				{
					ComponentType.ReadOnly<Curve>()
				},
				Any = new ComponentType[3]
				{
					ComponentType.ReadOnly<GarageLane>(),
					ComponentType.ReadOnly<ConnectionLane>(),
					ComponentType.ReadOnly<AreaLane>()
				},
				None = new ComponentType[2]
				{
					ComponentType.ReadOnly<Deleted>(),
					ComponentType.ReadOnly<Temp>()
				}
			});
			// 事件驱动重刷：只匹配刚被 LaneOverlapSystem 重建过 overlap 的车道。
			// Updated 是单帧脏标记（CleanUpSystem 帧末移除），稳态下匹配 0 实体。
			m_RefreshQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[2]
				{
					ComponentType.ReadOnly<LaneOverlap>(),
					ComponentType.ReadOnly<Updated>()
				},
				Any = new ComponentType[5]
				{
					ComponentType.ReadOnly<CarLane>(),
					ComponentType.ReadOnly<ParkingLane>(),
					ComponentType.ReadOnly<GarageLane>(),
					ComponentType.ReadOnly<ConnectionLane>(),
					ComponentType.ReadOnly<AreaLane>()
				},
				None = new ComponentType[2]
				{
					ComponentType.ReadOnly<Deleted>(),
					ComponentType.ReadOnly<Temp>()
				}
			});
			RequireForUpdate(m_OverlapQuery);
			// v0.8.0 主要重删闸门：逐条照抄原版 LaneOverlapSystem.m_UpdatedOwnersQuery
			// （Game/Net/LaneOverlapSystem.cs:1893）。刻意不排除 Lane——原版的清单里
			// edge/node/建筑都算 owner，我们只是拿它的 SubLane 缓冲找车道路径去重删，
			// 过滤条件在 job 内部按车道类型判定。
			m_RebuildOwnerQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[2]
				{
					ComponentType.ReadOnly<SubLane>(),
					ComponentType.ReadOnly<Updated>()
				},
				None = new ComponentType[1]
				{
					ComponentType.ReadOnly<Deleted>()
				}
			});
			m_OwnerQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[1]
				{
					ComponentType.ReadOnly<SubLane>()
				},
				None = new ComponentType[4]
				{
					ComponentType.ReadOnly<Deleted>(),
					ComponentType.ReadOnly<Temp>(),
					ComponentType.ReadOnly<Lane>(),
					ComponentType.ReadOnly<LaneOverlap>()
				}
			});
			m_RestoreLaneQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[1]
				{
					ComponentType.ReadOnly<Lane>()
				},
				None = new ComponentType[3]
				{
					ComponentType.ReadOnly<Deleted>(),
					ComponentType.ReadOnly<Temp>(),
					ComponentType.ReadOnly<SecondaryLane>()
				}
			});
		}

		protected override void OnDestroy()
		{
			// ⚠ 稳定性关键一步（原生闪退自查，2026-09-30）：本系统的 job 读的是下面这些
			// Allocator.Persistent 容器（锚点网格 / 两份邻域集合 / 候选清单 / 诊断计数）。
			// ECS 不会替我们把 Dependency 等到零就进来，模组被卸载、退出到桌面、世界拆解都可能
			// 发生在"上一帧 OnUpdate 排了 job、这一帧就销毁"的窗口里；这时先 Dispose 就是
			// **释放仍在被 Burst job 读的内存** ⇒ 原生访问违例，而托管栈是空的（正是最难归因的那种崩）。
			WaitForJobs();
			// 审计是同步取结果的（无跨帧句柄），这里只需清掉待办状态。
			m_AuditPhase = kAuditIdle;
			m_AuditFrames = 0;
			if (m_AnchorGrid.IsCreated)
			{
				m_AnchorGrid.Dispose();
			}
			for (int i = 0; i < 2; i++)
			{
				if (m_VehicleSets[i].IsCreated)
				{
					m_VehicleSets[i].Dispose();
				}
			}
			if (m_Candidates.IsCreated)
			{
				m_Candidates.Dispose();
			}
			if (m_RepairCounters.IsCreated)
			{
				m_RepairCounters.Dispose();
			}
			if (m_GridStats.IsCreated)
			{
				m_GridStats.Dispose();
			}
			base.OnDestroy();
		}

		/// <summary>
		/// 换存档 / 换图 / 新建城市：世界里的实体会被整体重建，而**本系统实例不换**
		/// （`Game.GameSystemBase` 在 OnCreate 里把自己挂在 `GameManager.onGamePreload` 上，见反编译
		/// `Game/GameSystemBase.cs:27-29`）。以前没有重写这里 ⇒ 缓存的 `m_VehicleSets` /
		/// `m_AnchorGrid` / `m_Candidates` 全是**上一张图的 Entity id**：
		/// 新图里 index 会被复用，version/serial 变了就查不到（只是白跑），
		/// 但一旦某个旧 id 恰好命中新世界里另一个实体，我们就会把"车道"的判定套到它身上，
		/// 甚至对它 `SetBuffer&lt;LaneOverlap&gt;` ——给不该有这个缓冲的实体写 LaneOverlap 就是往别的系统的
		/// 内存布局里塞东西，后果是别处崩溃且托管栈为空。这里把整座缓存清干净，
		/// 让下一帧按新世界重建集合与锚点网格。
		/// </summary>
		protected override void OnGamePreload(Colossal.Serialization.Entities.Purpose purpose, Game.GameMode mode)
		{
			ResetWorldCaches();
			AccessAnarchyMod.log.Info($"World reset on game preload (purpose={purpose}, mode={mode}): cached vehicle sets and anchor grid cleared.");
		}

		/// <summary>丢掉所有与"上一张图的实体"绑定的缓存与派生状态。调用方保证主线程、无在飞 job
		/// （本方法自己先 `WaitForJobs()`）。</summary>
		private void ResetWorldCaches()
		{
			WaitForJobs();
			if (m_AnchorGrid.IsCreated)
			{
				m_AnchorGrid.Clear();
			}
			if (m_Candidates.IsCreated)
			{
				m_Candidates.Clear();
			}
			for (int i = 0; i < 2 && m_VehicleSets != null; i++)
			{
				if (m_VehicleSets[i].IsCreated)
				{
					m_VehicleSets[i].Clear();
				}
			}
			if (m_RepairCounters.IsCreated)
			{
				for (int i = 0; i < m_RepairCounters.Length; i++)
				{
					m_RepairCounters[i] = 0;
				}
			}
			if (m_GridStats.IsCreated)
			{
				for (int i = 0; i < m_GridStats.Length; i++)
				{
					m_GridStats[i] = 0;
				}
			}
			m_ActiveSetIndex = 0;
			m_Cursor = -1;
			m_SetsInitialized = false;
			m_BuildInProgress = false;
			m_SetsStale = true;
			m_InitialGlobalApplyDone = false;
			m_ReapplyCountdown = 0;
			m_ScopeObserved = false;
			m_LastMode = -1;
			m_WasEnabled = false;
			m_LoggedFirstRun = false;
			m_BaselineTaken = false;
			m_BaselineEntries = 0;
			m_BaselineCrosswalk = 0;
			m_AuditPhase = kAuditIdle;
			m_AuditFrames = 0;
			m_AuditChecks = 0;
			m_RestoreCheckGlobal = false;
			m_GateFrames = 0;
			m_GateLastOwners = 0;
			m_LastGateFrames = -1;
			m_LastGateLanes = -1;
			m_LastGateEntries = -1;
			m_LastGridSources = -1;
			m_LastGridAnchors = -1;
			m_LastGridCells = -1;
			m_LastGridDropped = -1;
			m_LastAuditSet = -1;
			m_LastAuditLanes = -1;
			m_LastAuditEntries = -1;
			m_LastAnchorCount = -1;
			m_LastCandidateCount = -1;
			m_GateSeenSinceRebuild = false;
			m_LastRebuildFrame = 0;
			m_RebuildRounds = 0;
			m_RebuildSkips = 0;
			m_LastLoggedRounds = -1;
			m_LastLoggedSkips = -1;
		}

		protected override void OnUpdate()
		{
			// 每一步开头先把游戏留在 Dependency 上的句柄吸收进自持链：第 2 步起游戏已经把
			// Dependency 清成 default 了，不吸就可能让上一步的 job 从此无人等待（见 m_JobChain 注释）。
			SyncJobChain();
			if (!m_KeyReported)
			{
				m_KeyReported = true;
				Setting.ReportKeyBindings("in-game");
			}
			Setting setting = Setting.Instance;
			bool enabled = setting != null && setting.Enabled;
			if (!enabled)
			{
				// 启用→关闭跳变：我们此前用 ECB.SetBuffer 删掉的 LaneOverlap 条目不会自动还原
				// （原版 LaneOverlapSystem 只在拥有者实体带 Updated 脏标记或读档时才重建 overlap）。
				// 触发一次原版重建，行人条目随之恢复。
				if (m_WasEnabled)
				{
					m_WasEnabled = false;
					// ⚠ `setting` 在这里**可以是 null**：上面的 `enabled` 就是按"null 也算关闭"写的
					// （AccessAnarchyMod.OnDispose 会把 Setting.Instance 置空，而 Game.UpdateAt 注册的系统
					// 不随模组卸载销毁，世界还会继续 tick 我们）。所以这一行不能再裸读 setting.Mode——
					// 抛 NRE 时 m_WasEnabled 已经是 false，这条沿一辈子只走这一次，后面的
					// RestoreVanillaOverlaps() 与八个状态复位全被跳过，正是"关了却永久残留"的形状。
					// Instance 没了就问不了当下模式，退回**我们上一次真正应用过的模式** m_LastMode。
					m_RestoreCheckGlobal = (setting != null ? setting.Mode : m_LastMode) == Setting.kModeGlobal;
					RestoreVanillaOverlaps();
					// 清空所有增量状态：下次启用时重新做全量 A + 重建邻域集合。
					m_InitialGlobalApplyDone = false;
					m_SetsInitialized = false;
					m_BuildInProgress = false;
					m_Cursor = -1;
					m_LastMode = -1;
					m_ReapplyCountdown = 0;
					m_ScopeObserved = false;
					m_SetsStale = true;
				}
				// 审计只在关闭态推进：启用态下我们本来就在删，测出的 0 会误判成"没恢复"，
				// 进而白跑一次全城 owner+lane Updated（v0.7.2 时代那种 churn）。
				TickRestoreAudit();
				return;
			}
			if (!m_WasEnabled)
			{
				// 关闭→启用跳变：状态已在关闭时重置，这里只记录，后续走首次全量 A；
				// 顺带撤销上一轮关闭时可能还挂着的恢复检查（刚关掉又立刻开启时不该再测）。
				m_WasEnabled = true;
				m_AuditPhase = kAuditIdle;
				m_AuditFrames = 0;
			}
			bool globalMode = setting.Mode == Setting.kModeGlobal;
			uint frameIndex = m_SimulationSystem.frameIndex;
			m_TotalPasses++;

			if (!m_LoggedFirstRun)
			{
				m_LoggedFirstRun = true;
				LogStatus("first pass", globalMode);
			}
			else if (m_TotalPasses % kHeartbeatInterval == 0)
			{
				LogStatus("heartbeat", globalMode);
				// 心跳时顺带把两个自证指标读掉并清零（内部会先 Complete 依赖，所以每 ~30 秒
				// 预期有一次极短的等待；稳态下没有长任务在飞，等待时间在亚毫秒级）。
				ReportRebuildDiagnostics(globalMode);
			}

			// 启用后第一遍此刻还没删过任何条目（本帧的删除要到帧末 ECB 回放才生效），
			// 扫一次当作"这个存档原版的行人条目数"基线：关闭后判恢复不再靠">0"这种
			// 会被横道条目糊弄过去的弱判据（v0.7.8）。系统实例随世界重建，基线只测一次。
			if (!m_BaselineTaken && ScanPedestrianOverlaps(out _, out int baselineEntries, out _, out int baselineCrosswalk))
			{
				m_BaselineEntries = baselineEntries;
				m_BaselineCrosswalk = baselineCrosswalk;
				m_BaselineTaken = true;
				AccessAnarchyMod.log.Info($"Baseline pedestrian overlaps: {baselineEntries} entries ({baselineCrosswalk} crosswalk).");
			}

			// 锚点网格 + 邻域集合：低频整轮重建。不再每帧增量扫全城做 IsNearAnchor。
			// v0.8.3 三合一：Include* 三个勾选已删，范围只会随「取消避让范围」这个模式下拉框变宽变窄。
			if (setting.Mode != m_LastMode)
			{
				// 只有"范围变窄"才需要还原重刷：变宽时多删几条就行，变窄时先前删过的范围外车道
				// 不会自己回来（v0.7.6 的同一个根因），必须让原版先重建一次再按新范围重删。
				// ⚠ 这条判据在 v0.8.2 之前写的是恒假死条件（m_LastMode == kModeGlobal && !toAccessMode），
				//    作者实测「从全部道路切回仅出入口后，人行横道再也不避让、只有关总开关才好」就是它
				//    （开发笔记 §⑨ 第 1 条）—— 别再改回去。
				bool narrowed = m_LastMode == Setting.kModeGlobal && setting.Mode != Setting.kModeGlobal;
				m_LastMode = setting.Mode;
				m_Cursor = -1;
				m_SetsInitialized = false;
				m_BuildInProgress = false;
				m_InitialGlobalApplyDone = false;
				m_SetsStale = true;
				if (narrowed && m_ScopeObserved)
				{
					TriggerVanillaOverlapRebuild("scope narrowed");
					m_ReapplyCountdown = kReapplyDelayFrames;
					return;
				}
				m_ScopeObserved = true;
			}

			// 等原版重建落地：这几帧既不删也不建集合，避免删完又被重建覆盖。
			if (m_ReapplyCountdown > 0)
			{
				m_ReapplyCountdown--;
				return;
			}
			m_ReapplyCountdown = 0;

			// v0.8.5 三条改动集中在这一段：
			// ① 原版闸门刚开过 ⇒ 不必等到下一个 512 步检查点，起一轮（但要隔至少 kGateRebuildMinGap
			//    步，否则连续施工会一轮接一轮）。v0.8.4 的变化探测器只数车道总数，"车道被重算过"
			//    这件事没参与判断，正是玩家说的"电梯/建筑状态变化后个别出入口又恢复避让"的成因之一
			//    （另一成因是反向写入，见 StripRebuiltLanesJob）。
			// ② 稳态空转的固定税：原来每一步都要取一次 EndFrameBarrier 命令缓冲、排一个查询为空的
			//    重刷 job、再把自己注册成 barrier 生产者。单次几微秒，但乘上每渲染帧 1–8 个模拟步
			//    就是长期白付；先判"这一步到底有没有活"，没活直接返回。
			// ③ 判活所需的三个查询空判断都用 IsEmptyIgnoreFilter（按 archetype 计数），成本可忽略。
			bool needAccessSets = !globalMode;
			bool gateActive = !m_RebuildOwnerQuery.IsEmptyIgnoreFilter;
			bool refreshActive = !m_RefreshQuery.IsEmptyIgnoreFilter;
			bool globalFirstPass = globalMode && !m_InitialGlobalApplyDone;
			bool gateUrgent = m_GateSeenSinceRebuild && frameIndex - m_LastRebuildFrame >= (uint)kGateRebuildMinGap;
			bool roundDue = needAccessSets && (m_BuildInProgress || !m_SetsInitialized || gateUrgent || ShouldRebuildAccessSets(frameIndex));
			if (!roundDue && !gateActive && !refreshActive && !globalFirstPass)
			{
				return;
			}

			// 命令缓冲**先取再干活**：SafeCommandBufferSystem 可能拒绝（见 TryCreateBarrierBuffer），
			// 而 AdvanceAccessSets 一旦把集合交换好就指望后面的删除批去落地。放在它前面，取不到就整步放弃，
			// 既不会留下"排了却没注册"的 job，也不会出现"集合已就绪但这一轮没人删"要等 34 秒的情况。
			if (!TryCreateBarrierBuffer(out EntityCommandBuffer.ParallelWriter ecb))
			{
				return;
			}

			bool setsJustReady = false;
			bool startRound = roundDue && !m_BuildInProgress;
			if (roundDue)
			{
				if (startRound)
				{
					// 起新一轮（旧活跃集在重建期间仍可继续用）。
					m_Cursor = -1;
					m_LastAnchorCount = m_AnchorQuery.CalculateEntityCount();
					m_LastCandidateCount = m_OverlapQuery.CalculateEntityCount();
					m_GateSeenSinceRebuild = false;
					m_LastRebuildFrame = frameIndex;
					m_RebuildRounds++;
				}
				setsJustReady = AdvanceAccessSets(setting, frameIndex);
			}

			// 干预点 A 调度（事件驱动，不再用 kReapplyInterval 定时器）：
			// 1. 全局模式世界加载后首次全量 A（m_OverlapQuery 全城 ScheduleParallel）
			// 2. access 模式集合刚建好时批量 A（StripAccessOverlapsFromSetJob 遍历车辆集合）
			// 3. 每帧事件驱动重刷（m_RefreshQuery 只匹配带 Updated 的车道，稳态 0 实体）
			//    LaneOverlapSystem 在 Modification4B 重建 overlap 后，Updated 仍在 → 当帧重删。
			// （ecb 已在本步开头取到，见上面 TryCreateBarrierBuffer 的注释）
			bool scheduled = false;

			// (0) v0.8.0 主机制：跟随原版重建闸门重删。
			// 原版 LaneOverlapSystem 在 Modification4B（本阶段之前）把这一批 owner 的 SubLane
			// 逐条 Clear 后重算 overlap，我们删掉的行人条目就是在这一刻回来的；它不给车道打 Updated，
			// 所以只看车道 Updated 的 (3) 收不到这批车道 → 出入口会在下一次集合整轮重建
			// （kSetRebuildCheckInterval=512 步一查 / kSetRebuildFallback=8192 步兜底，v0.8.4）之前一直恢复避让，这正是「个别停车场出入口仍会避让」；
			// 全局模式更糟：它没有周期性重删，条目一旦回来就永远回来。
			// 现在用与原版同一个闸门、同一批实体（owner 的 SubLane + Updated 节点的相邻 edge），
			// 当帧按同一套判定重删；没被改写过的车道被 HasPedestrianOverlap 直接跳过，不产生写入。
			if (!m_RebuildOwnerQuery.IsEmptyIgnoreFilter)
			{
				m_GateFrames++;
				// v0.8.5：只有"当前没有整轮在飞"时才把闸门事件记下来。一轮要跑
				// ceil(候选数/kSetBuildStep) 步（实机 128k 候选 ≈ 63 步），期间原版闸门几乎每步都开；
				// 若照原样在闸门块里无条件置位，本轮一结束 gateUrgent 立刻满足（63 ≈ kGateRebuildMinGap），
				// 于是施工期一轮接一轮地满负荷跑 —— 这正是 v0.8.4 用 512 步检查点压下去的开销。
				// 本轮在飞期间发生的重建由这一轮的集合批删 + 闸门重删一起收尾，不需要再记账。
				if (!m_BuildInProgress)
				{
					m_GateSeenSinceRebuild = true;
				}
				m_GateLastOwners = m_RebuildOwnerQuery.CalculateEntityCount();
				// 诊断计数按 chunk 独占槽位（会按需扩容，扩容点已在 WaitForJobs() 之后）。
				EnsureRepairCapacity(m_RebuildOwnerQuery.CalculateChunkCountWithoutFiltering());
				StripRebuiltLanesJob rebuildJob = new StripRebuiltLanesJob
				{
					m_EntityType = GetEntityTypeHandle(),
					m_SubLanes = GetBufferLookup<SubLane>(isReadOnly: true),
					m_ConnectedEdges = GetBufferLookup<ConnectedEdge>(isReadOnly: true),
					m_UpdatedData = GetComponentLookup<Updated>(isReadOnly: true),
					m_Overlaps = GetBufferLookup<LaneOverlap>(isReadOnly: true),
					m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
					m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
					m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
					m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
					m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_IncludeCrosswalks = globalMode ? 1 : 0,
					m_Ecb = ecb,
					m_RepairCounters = m_RepairCounters
				};
				m_JobChain = rebuildJob.ScheduleParallel(m_RebuildOwnerQuery, m_JobChain);
				scheduled = true;
			}

			// (1) 全局模式首次全量
			if (globalMode && !m_InitialGlobalApplyDone)
			{
				StripPedestrianOverlapsJob overlapJob = new StripPedestrianOverlapsJob
				{
					m_EntityType = GetEntityTypeHandle(),
					m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
					m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
					m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
					m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
					m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
					m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
					m_CarLaneType = GetComponentTypeHandle<CarLane>(isReadOnly: true),
					m_ParkingLaneType = GetComponentTypeHandle<ParkingLane>(isReadOnly: true),
					m_GarageLaneType = GetComponentTypeHandle<GarageLane>(isReadOnly: true),
					m_ConnectionLaneType = GetComponentTypeHandle<ConnectionLane>(isReadOnly: true),
					m_AreaLaneType = GetComponentTypeHandle<AreaLane>(isReadOnly: true),
					m_OverlapType = GetBufferTypeHandle<LaneOverlap>(isReadOnly: true),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = default,
					m_Mode = setting.Mode,
					m_IncludeCrosswalks = 1,
					m_Ecb = ecb
				};
				overlapJob.m_SortKeyBase = kSortKeyInitialPass;
				m_JobChain = overlapJob.ScheduleParallel(m_OverlapQuery, m_JobChain);
				m_InitialGlobalApplyDone = true;
				scheduled = true;
			}

			// (2) access 模式集合刚建好时批量 A
			if (!globalMode && setsJustReady && m_VehicleSets[m_ActiveSetIndex].Count > 0)
			{
				StripAccessOverlapsFromSetJob accessJob = new StripAccessOverlapsFromSetJob
				{
					m_Set = m_VehicleSets[m_ActiveSetIndex],
					m_Overlaps = GetBufferLookup<LaneOverlap>(isReadOnly: true),
					m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
					m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
					m_Ecb = ecb
				};
				accessJob.m_SortKeyBase = kSortKeyFromSet;
				m_JobChain = accessJob.Schedule(m_JobChain);
				scheduled = true;
			}

			// (3) 每帧事件驱动重刷：只处理带 Updated 的车道（LaneOverlapSystem 刚重建过 overlap 的）。
			// 全局模式：所有 Updated 车辆车道都删；access 模式：额外检查类型/集合/锚点邻域。
			// v0.8.5：外面已经用 IsEmptyIgnoreFilter 判过活，这里直接排；本步其余三段
			// （闸门重删 / 首次全量 / 集合批删）都在写同一批 LaneOverlap 缓冲，串在同一条
			// m_JobChain 上按顺序跑，不产生竞争。
			if (refreshActive)
			{
				StripPedestrianOverlapsJob refreshJob = new StripPedestrianOverlapsJob
				{
					m_EntityType = GetEntityTypeHandle(),
					m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
					m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
					m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
					m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
					m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
					m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
					m_CarLaneType = GetComponentTypeHandle<CarLane>(isReadOnly: true),
					m_ParkingLaneType = GetComponentTypeHandle<ParkingLane>(isReadOnly: true),
					m_GarageLaneType = GetComponentTypeHandle<GarageLane>(isReadOnly: true),
					m_ConnectionLaneType = GetComponentTypeHandle<ConnectionLane>(isReadOnly: true),
					m_AreaLaneType = GetComponentTypeHandle<AreaLane>(isReadOnly: true),
					m_OverlapType = GetBufferTypeHandle<LaneOverlap>(isReadOnly: true),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_IncludeCrosswalks = globalMode ? 1 : 0,
					m_Ecb = ecb
				};
				refreshJob.m_SortKeyBase = kSortKeyRefresh;
				m_JobChain = refreshJob.ScheduleParallel(m_RefreshQuery, m_JobChain);
				scheduled = true;
			}

			if (scheduled)
			{
				m_EndFrameBarrier.AddJobHandleForProducer(m_JobChain);
			}
		}

		/// <summary>把诊断计数数组扩到能容纳 <paramref name="chunkCount"/> 个 chunk（每块两格）。
		/// 扩容点一定在没有在飞 job 的时刻调用（OnUpdate 里排 gate job 之前，且自己再 Complete 一次兜底）。</summary>
		private void EnsureRepairCapacity(int chunkCount)
		{
			if (chunkCount <= m_RepairCounterChunks)
			{
				return;
			}
			WaitForJobs();
			if (m_RepairCounters.IsCreated)
			{
				m_RepairCounters.Dispose();
			}
			m_RepairCounterChunks = chunkCount + (chunkCount >> 2) + 16;
			m_RepairCounters = new NativeArray<int>(m_RepairCounterChunks * 2, Allocator.Persistent, NativeArrayOptions.ClearMemory);
		}

		/// <summary>
		/// 心跳自证日志（v0.8.0）。用户回报「某个出入口仍会避让」时先看这三行：
		/// ① Rebuild gate —— 原版这一阵子改写过 overlap 没有、我们跟上了多少；
		/// ② Anchor grid —— 锚点容量有没有被卡掉（卡掉就会有邻近车道判不出靠出入口）；
		/// ③ Access set audit —— 集合内的车辆车道此刻还留着几条行人条目。
		/// ①非零而③也非零 = 我们没跟全（漏口还在）；①=0 且③=0 却仍然避让 = 不是 LaneOverlap 这条通道，
		/// 而是类注释 v0.7.2 里保留的原版行为（行人就站在车道本体上、或行人静止不动被当成静态障碍）。
		/// v0.8.3 三合一：这里不再有第四行 `Scope census`（按勾选项普查的机器随勾选框一起删除）。
		/// </summary>
		private void ReportRebuildDiagnostics(bool globalMode)
		{
			// 读 job 写过的计数数组之前必须等在飞的 job 收尾（与 ScanPedestrianOverlaps 同一套做法）。
			WaitForJobs();
			int lanes = 0;
			int entries = 0;
			for (int i = 0; i < m_RepairCounterChunks; i++)
			{
				lanes += m_RepairCounters[i * 2];
				entries += m_RepairCounters[i * 2 + 1];
				m_RepairCounters[i * 2] = 0;
				m_RepairCounters[i * 2 + 1] = 0;
			}
			if (m_GateFrames != m_LastGateFrames || lanes != m_LastGateLanes || entries != m_LastGateEntries)
			{
				m_LastGateFrames = m_GateFrames;
				m_LastGateLanes = lanes;
				m_LastGateEntries = entries;
				AccessAnarchyMod.log.Info($"Rebuild gate: {m_GateFrames} frame(s) with vanilla LaneOverlap rebuild this period, last gate {m_GateLastOwners} owner(s); re-stripped {lanes} lane(s) / {entries} pedestrian entries (per-chunk counter slots: {m_RepairCounterChunks}).");
			}
			m_GateFrames = 0;
			m_GateLastOwners = 0;

			if (m_GridStats.IsCreated)
			{
				int sources = m_GridStats[kGridSources];
				int pedSkipped = m_GridStats[kGridPedSkipped];
				int anchors = m_GridStats[kGridAnchors];
				int cells = m_GridStats[kGridCells];
				int dropped = m_GridStats[kGridDropped];
				if (sources != m_LastGridSources || anchors != m_LastGridAnchors || cells != m_LastGridCells || dropped != m_LastGridDropped)
				{
					m_LastGridSources = sources;
					m_LastGridAnchors = anchors;
					m_LastGridCells = cells;
					m_LastGridDropped = dropped;
					string line = $"Anchor grid: {sources} vehicle source lane(s) -> {anchors} anchor point(s) in {cells} cell(s) (max {m_GridStats[kGridMaxCell]} per cell, gate {kAnchorPerCell}), {m_GridStats[kGridDeduped]} duplicate(s) merged, {pedSkipped} pedestrian lane(s) excluded, {dropped} dropped.";
					if (dropped > 0)
					{
						AccessAnarchyMod.log.Warn(line + " Lanes near a crowded access cluster may stay un-stripped - please send this line.");
					}
					else
					{
						// v0.8.4：容量截断本来是"没填满"的常态，玩家/作者看到的 WARN 应该是**这一句**而不是警告。
						AccessAnarchyMod.log.Info(line);
					}
				}
			}

			// v0.8.4 的自证行：整轮重建到底还在不在空转。稳态应当 rounds 不动、skipped 持续增长。
			// skipped 每个检查点（512 步）都会 +1，所以只在它攒够 64 次（≈1× 速度五分钟）或 rounds 变化时才写，
			// 免得这行本身变成新的刷屏源。
			if (m_RebuildRounds != m_LastLoggedRounds || m_RebuildSkips - m_LastLoggedSkips >= 64)
			{
				m_LastLoggedRounds = m_RebuildRounds;
				m_LastLoggedSkips = m_RebuildSkips;
				AccessAnarchyMod.log.Info($"Access-zone rebuild driver: {m_RebuildRounds} round(s) started, {m_RebuildSkips} check(s) skipped as unchanged (check every {kSetRebuildCheckInterval} steps, fallback every {kSetRebuildFallback}).");
			}

			if (!globalMode && m_SetsInitialized)
			{
				AuditAccessSet();
			}
		}

		/// <summary>邻域集合当前是不是干净的（成本 O(集合)，不是 O(全城)）。</summary>
		private void AuditAccessSet()
		{
			NativeHashSet<Entity> set = m_VehicleSets[m_ActiveSetIndex];
			if (!set.IsCreated || set.Count == 0)
			{
				return;
			}
			NativeArray<int> counters = new NativeArray<int>(2, Allocator.TempJob);
			AuditAccessSetJob auditJob = new AuditAccessSetJob
			{
				m_Set = set,
				m_Overlaps = GetBufferLookup<LaneOverlap>(isReadOnly: true),
				m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
				m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
				m_Counters = counters
			};
			// 传进自持链 m_JobChain（不是 Dependency —— 游戏多步模拟第 2 步起会把它清成 default，见字段注释）。
			// 排完立刻 Complete()：这是同步取结果的审计，不留跨步句柄。
			auditJob.Schedule(m_JobChain).Complete();
			int lanesWithEntries = counters[0];
			int entries = counters[1];
			counters.Dispose();
			if (set.Count != m_LastAuditSet || lanesWithEntries != m_LastAuditLanes || entries != m_LastAuditEntries)
			{
				m_LastAuditSet = set.Count;
				m_LastAuditLanes = lanesWithEntries;
				m_LastAuditEntries = entries;
				AccessAnarchyMod.log.Info($"Access set audit: {lanesWithEntries} of {set.Count} vehicle lanes still hold {entries} pedestrian overlap entries (0 = clean, crosswalk entries excluded by design).");
			}
		}

		private void LogStatus(string tag, bool globalMode)
		{
			// 刻意不算 CalculateEntityCount（大地图上很贵）；集合 Count 够诊断。
			AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem {tag}: pass {m_TotalPasses}, vehicle set {m_VehicleSets[m_ActiveSetIndex].Count}, mode={(globalMode ? "global" : "access-zones")}");
		}

		/// <summary>
		/// 启用→关闭跳变：让原版 LaneOverlapSystem 重建 LaneOverlap，恢复我们删掉的行人条目。
		/// 见类注释 v0.7.6：只给车道打 Updated 无效（原版的重建闸门和工作清单都按"拥有者实体"算），
		/// 所以优先反射置 LaneOverlapSystem.m_Loaded=true，走与读档完全相同的全量重建分支；
		/// 反射不可用时退化为「全部拥有者 + 车辆车道」打 Updated 的增量重建。
		/// 两条路径之后都排一次审计，把"行人条目到底回来了没有"写进日志。
		/// </summary>
		private void RestoreVanillaOverlaps()
		{
			TriggerVanillaOverlapRebuild("disabled");
			m_AuditChecks = 0;
			QueueRestoreCheck();
		}

		/// <summary>排一次恢复检查：延后 kRestoreCheckDelays[已做次数] 帧。</summary>
		private void QueueRestoreCheck()
		{
			int index = math.clamp(m_AuditChecks, 0, kRestoreCheckDelays.Length - 1);
			m_AuditPhase = kAuditWaitFrames;
			m_AuditFrames = kRestoreCheckDelays[index];
		}

		/// <summary>
		/// 请求一次原版 LaneOverlap 重建（关闭功能、或范围收窄后准备按新范围重删时调用）。
		/// 主路径：反射置 LaneOverlapSystem.m_Loaded=true；兜底：owner + 车道打 Updated。
		/// </summary>
		private void TriggerVanillaOverlapRebuild(string reason)
		{
			if (TryForceVanillaOverlapRebuild())
			{
				AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem {reason}: requested vanilla FULL LaneOverlap rebuild (LaneOverlapSystem.m_Loaded=true, same path as loading a save).");
			}
			else
			{
				AccessAnarchyMod.log.Warn($"AccessZoneOverlapSystem {reason}: full rebuild unavailable, falling back to marking network owners + vehicle lanes Updated.");
				ScheduleMarkUpdatedRestore();
			}
		}

		/// <summary>
		/// 检查不达标时的加力重试（v0.7.8）：第 1 次仍只再推一次 m_Loaded；
		/// 之后同时补打 owner+lane Updated——两套机制交替，避免"某一条被游戏吞掉"就永久卡死。
		/// </summary>
		private void RetryVanillaOverlapRebuild(int check)
		{
			bool forced = TryForceVanillaOverlapRebuild();
			if (check >= 2 || !forced)
			{
				ScheduleMarkUpdatedRestore();
			}
			AccessAnarchyMod.log.Warn($"Restore retry after check {check}: m_Loaded again={forced}, owner+lane Updated={(check >= 2 || !forced ? "yes" : "no")}, next check in {kRestoreCheckDelays[math.clamp(check, 0, kRestoreCheckDelays.Length - 1)]} frames.");
		}

		/// <summary>
		/// 反射把 LaneOverlapSystem.m_Loaded 置 true：它的 GetLoaded() 下一帧返回 true，
		/// 于是走 m_AllOwnersQuery（全城拥有者）+ m_UpdateAll=true 的读档同款全量重建，
		/// 不依赖 Updated 脏标记，也不受"节点未 Updated 则跳过跨拥有者 overlap"条件影响。
		/// 拿不到字段或系统不在本世界则返回 false（调用方走兜底）。
		/// </summary>
		private bool TryForceVanillaOverlapRebuild()
		{
			try
			{
				if (!s_LoadedFieldResolved)
				{
					s_LoadedFieldResolved = true;
					s_LoadedField = typeof(LaneOverlapSystem).GetField("m_Loaded", BindingFlags.Instance | BindingFlags.NonPublic);
					if (s_LoadedField == null)
					{
						AccessAnarchyMod.log.Warn("LaneOverlapSystem.m_Loaded field not found (game updated?) - owner-marking fallback will be used.");
					}
				}
				if (s_LoadedField == null)
				{
					return false;
				}
				LaneOverlapSystem laneOverlapSystem = World.GetExistingSystemManaged<LaneOverlapSystem>();
				if (laneOverlapSystem == null)
				{
					AccessAnarchyMod.log.Warn("LaneOverlapSystem not present in this world - owner-marking fallback will be used.");
					return false;
				}
				s_LoadedField.SetValue(laneOverlapSystem, true);
				return true;
			}
			catch (Exception ex)
			{
				AccessAnarchyMod.log.Warn("Forcing LaneOverlapSystem full rebuild failed: " + ex);
				return false;
			}
		}

		/// <summary>
		/// 兜底路径（v0.7.6）：给原版重建所需的两类实体打 Updated——
		/// 1) 拥有者实体（带 SubLane 缓冲的 edge/node/building，m_OwnerQuery）：满足
		///    LaneOverlapSystem 的闸门 m_UpdatedOwnersQuery=[SubLane,Updated,!Deleted]，
		///    并进入 UpdateLaneOverlapsJob 的工作清单（它按 m_SubLanes[owner] 逐条 Clear+重算）；
		///    节点被标记后 AddNonUpdatedEdgesJob 还会把相邻 edge 一起拉进清单。
		/// 2) 全部车道（m_RestoreLaneQuery，与 m_AllLanesQuery 同口径）：满足 m_UpdatedLanesQuery，
		///    让 CollectLaneDirectionsJob / UpdateLaneFlagsJob 把 CarLane Yield/Stop/Approach 等
		///    overlap 派生标记整趟刷新（只标车辆车道会漏掉行人道侧的标记）。
		/// 两个查询互斥（拥有者侧排除 Lane/LaneOverlap，车道侧必须有 Lane），
		/// 同一 ECB 不会对同一实体重复 AddComponent。
		/// 一次性全城重建的成本与世界加载相当，仅在开关跳变且主路径不可用时发生。
		/// </summary>
		private void ScheduleMarkUpdatedRestore()
		{
			if (!TryCreateBarrierBuffer(out EntityCommandBuffer.ParallelWriter ecb))
			{
				return;
			}
			MarkOwnersUpdatedJob ownerJob = new MarkOwnersUpdatedJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_UpdatedData = GetComponentLookup<Updated>(isReadOnly: true),
				m_SortKeyBase = kSortKeyRestore,
				m_Ecb = ecb
			};
			JobHandle ownerHandle = ownerJob.ScheduleParallel(m_OwnerQuery, m_JobChain);
			MarkLanesUpdatedJob laneJob = new MarkLanesUpdatedJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_UpdatedData = GetComponentLookup<Updated>(isReadOnly: true),
				// 两个 Mark job 的查询集互斥（拥有者侧排除 Lane，车道侧必须有 Lane），
				// 所以共用同一个基址不会在同一实体上产生两条 AddComponent。
				m_SortKeyBase = kSortKeyRestore,
				m_Ecb = ecb
			};
			JobHandle laneHandle = laneJob.ScheduleParallel(m_RestoreLaneQuery, m_JobChain);
			m_JobChain = JobHandle.CombineDependencies(ownerHandle, laneHandle);
			m_EndFrameBarrier.AddJobHandleForProducer(m_JobChain);
		}

		/// <summary>
		/// 全城扫一遍车辆车道的 LaneOverlap，统计指向行人道的条目数，并单列其中指向人行横道的部分。
		/// 分块并行 + 主线程同步求和（每轮开关跳变后才跑，成本可接受）。
		/// 返回 false = 没有匹配到车辆车道（无数据可判）。</summary>
		private bool ScanPedestrianOverlaps(out int lanes, out int entries, out int crosswalkLanes, out int crosswalkEntries)
		{
			lanes = 0;
			entries = 0;
			crosswalkLanes = 0;
			crosswalkEntries = 0;
			WaitForJobs();
			int chunkCount = m_OverlapQuery.CalculateChunkCountWithoutFiltering();
			if (chunkCount <= 0)
			{
				return false;
			}
			NativeArray<int> counters = new NativeArray<int>(chunkCount * kAuditCountersPerChunk, Allocator.TempJob);
			AuditPedestrianOverlapsJob auditJob = new AuditPedestrianOverlapsJob
			{
				m_OverlapType = GetBufferTypeHandle<LaneOverlap>(isReadOnly: true),
				m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
				m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
				m_Counters = counters
			};
			auditJob.ScheduleParallel(m_OverlapQuery, m_JobChain).Complete();
			for (int i = 0; i < chunkCount; i++)
			{
				int baseIndex = i * kAuditCountersPerChunk;
				lanes += counters[baseIndex];
				entries += counters[baseIndex + 1];
				crosswalkLanes += counters[baseIndex + 2];
				crosswalkEntries += counters[baseIndex + 3];
			}
			counters.Dispose();
			return true;
		}

		/// <summary>
		/// 关闭后的恢复检查（v0.7.8 重写）。v0.7.7 的判据是"车辆车道上还有任何行人条目就算恢复"，
		/// 实机被这个判据骗过两次：出入口模式我们从不动横道条目，所以哪怕原版重建完全没落地、
		/// 我们删掉的非横道条目一条都没回来，日志里横道那几万条也照样把判据判成"已恢复"，
		/// 于是不重试、不报警——表现就是"仅出入口模式下只要开启过就永远不避让"。
		/// 新判据看"我们一定删过的那一类"：出入口模式看非横道条目数，全局模式看横道条目数；
		/// 不达标就加力重试（RetryVanillaOverlapRebuild），最多 kRestoreMaxChecks 轮，每轮间隔逐次放大。
		/// </summary>
		private void TickRestoreAudit()
		{
			if (m_AuditPhase != kAuditWaitFrames)
			{
				return;
			}
			m_AuditFrames--;
			if (m_AuditFrames > 0)
			{
				return;
			}
			m_AuditPhase = kAuditIdle;
			m_AuditChecks++;
			if (!ScanPedestrianOverlaps(out int lanes, out int entries, out int crosswalkLanes, out int crosswalkEntries))
			{
				AccessAnarchyMod.log.Warn($"Restore check #{m_AuditChecks}: skipped, no vehicle lanes matched.");
				return;
			}
			int nonCrosswalk = entries - crosswalkEntries;
			// 主判据：这个模式下"我们一定删过的那一类"条目回来了没有——
			// 出入口模式横道条目从未被动过（v0.7.7 就是被这几万条横道条目糊弄成"已恢复"的），
			// 全局模式横道条目也删过，看横道回来即证明重建落地。
			bool classBack = m_RestoreCheckGlobal ? crosswalkEntries > 0 : nonCrosswalk > 0;
			// 有基线再加一道总量闸：条目数得回到基线的 8 成（留 2 成给期间修路/拆路）。
			bool enough = !m_BaselineTaken || m_BaselineEntries <= 0 || entries >= (m_BaselineEntries * 4) / 5;
			bool restored = classBack && enough;
			AccessAnarchyMod.log.Info($"Restore check #{m_AuditChecks} ({(m_RestoreCheckGlobal ? "global" : "access-zones")}): {lanes} vehicle lanes, {entries} pedestrian entries (crosswalk {crosswalkEntries}, non-crosswalk {nonCrosswalk}) vs baseline {m_BaselineEntries} (crosswalk {m_BaselineCrosswalk}) - {(restored ? "vanilla avoidance restored" : $"NOT restored (classBack={classBack} enough={enough})")}.");
			if (restored)
			{
				return;
			}
			if (m_AuditChecks >= kRestoreMaxChecks)
			{
				AccessAnarchyMod.log.Warn($"Restore failed after {m_AuditChecks} checks: LaneOverlapSystem never rebuilt the {(m_RestoreCheckGlobal ? "crosswalk" : "non-crosswalk")} pedestrian entries. Please send this log.");
				return;
			}
			RetryVanillaOverlapRebuild(m_AuditChecks);
			QueueRestoreCheck();
		}

		/// <summary>
		/// 邻域集合的低频整轮重建：先重建锚点网格，再分帧把候选筛进双缓冲集合。
		/// 返回 true = 本帧完成了整轮交换（活跃集刚可用，调用方应立刻补 A）。
		/// 重建期间旧活跃集继续有效；只有第一次完成前活跃集为空。
		/// </summary>
		/// <summary>要不要起新一轮"锚点网格 + 邻域集合"整轮重建。v0.8.4 性能修法的核心：
		/// 稳态城市里这一步原来每 2048 步无条件跑一次，跑的是全城候选的 32 米邻域扫描，
		/// 而实机日志证明那些周期里原版 overlap 一次都没变过（「Rebuild gate: 0 frame(s)」），
		/// 等于持续白烧一条 worker 线程，直接把每步模拟的时间预算吃掉一截 —— 玩家看到的
		/// "帧数没事但模拟速度变慢"正是这个形状。</summary>
		private bool ShouldRebuildAccessSets(uint frameIndex)
		{
			// 长兜底先答，避免在下面做两次计数查询（也保证兜底不会被检查节奏错过）。
			if (frameIndex - m_LastRebuildFrame >= (uint)kSetRebuildFallback)
			{
				return true;
			}
			if (frameIndex % kSetRebuildCheckInterval != 0)
			{
				return false;
			}
			if (m_GateSeenSinceRebuild)
			{
				return true;
			}
			int anchors = m_AnchorQuery.CalculateEntityCount();
			if (anchors != m_LastAnchorCount)
			{
				return true;
			}
			int candidates = m_OverlapQuery.CalculateEntityCount();
			if (candidates == m_LastCandidateCount)
			{
				m_RebuildSkips++;
				return false;
			}
			return true;
		}

		private bool AdvanceAccessSets(Setting setting, uint frameIndex)
		{
			// v0.8.3 三合一：锚点网格与邻域集合都是整片出入口区域一起收，不再有勾选掩码。
			// 模式变化由 OnUpdate 负责作废整轮（m_SetsStale / m_Cursor）。
			if (m_BuildInProgress && m_Cursor >= 0 && m_Cursor >= m_Candidates.Length)
			{
				// 本轮扫完：交换双缓冲，活跃集可用。
				// 先等上一帧的 BuildAccessSetsJob（写新活跃集）与重刷 job（读旧活跃集）收尾：
				// 下面的 Clear() 与 Count 直读都要求这些并发句柄已释放。
				WaitForJobs();
				m_ActiveSetIndex = 1 - m_ActiveSetIndex;
				m_SetsInitialized = true;
				m_BuildInProgress = false;
				// v0.8.5：整轮"跑完"才算上一轮结束 —— 重读时间戳与闸门账都从这里起算。
				// 只在起轮时打时间戳是不够的：一轮本身要几十步，闸门在这几十步里几乎每步都开，
				// 于是本轮一结束 gateUrgent 就立刻成立，施工期变成满负荷连续重建。
				m_LastRebuildFrame = frameIndex;
				m_GateSeenSinceRebuild = false;
				int buildIndex = 1 - m_ActiveSetIndex;
				m_VehicleSets[buildIndex].Clear();
				m_Cursor = m_Candidates.Length;
				return m_VehicleSets[m_ActiveSetIndex].Count > 0;
			}

			if (!m_BuildInProgress)
			{
				// 起新一轮。旧活跃集继续给判定用；新内容写入另一侧缓冲。
				WaitForJobs();

				if (m_SetsStale)
				{
					// 范围/模式变过：旧活跃集里的越界成员不能再被重刷 job 当依据。
					// 两侧一起清空，本轮重新收集；过渡期判定退化为"类型 + 锚点邻域"。
					m_VehicleSets[0].Clear();
					m_VehicleSets[1].Clear();
					m_SetsStale = false;
				}

				BuildAnchorGridJob buildJob = new BuildAnchorGridJob
				{
					m_Entities = m_AnchorQuery.ToEntityArray(Allocator.TempJob),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
					m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
					m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
					m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
					m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_Stats = m_GridStats
				};
				m_JobChain = buildJob.Schedule(m_JobChain);

				int buildSide = 1 - m_ActiveSetIndex;
				m_VehicleSets[buildSide].Clear();

				m_Candidates.Clear();
				// 快照用完立刻 Dispose：Allocator.Temp 虽然帧末会被框架收掉，但一轮重建就是
				// 「全城车辆车道数 × 4 字节」的量，显式释放更干净（也更早）。
				NativeArray<Entity> candidateSnapshot = m_OverlapQuery.ToEntityArray(Allocator.Temp);
				m_Candidates.AddRange(candidateSnapshot);
				candidateSnapshot.Dispose();
				m_Cursor = 0;
				m_BuildInProgress = true;
				return false;
			}

			int endIndex = math.min(m_Cursor + kSetBuildStep, m_Candidates.Length);
			if (endIndex <= m_Cursor)
			{
				return false;
			}
			BuildAccessSetsJob job = new BuildAccessSetsJob
			{
				m_Candidates = m_Candidates,
				m_Start = m_Cursor,
				m_End = endIndex,
				m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
				m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
				m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
				m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
				m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
				m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
				m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
				m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
				m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
				m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
				m_Grid = m_AnchorGrid,
				m_BuildVehicleSet = m_VehicleSets[1 - m_ActiveSetIndex]
			};
			m_JobChain = job.Schedule(m_JobChain);
			m_Cursor = endIndex;
			return false;
		}

		private static int2 GridKey(float3 position)
		{
			return (int2)math.floor(position.xz / kGridCellSize);
		}

		/// <summary>曲线是否邻近出入口锚点。v0.8.0 起采样点与 BuildAnchorGridJob 放锚点时用的
		/// 五个点一致（两端 + 两个四分位 + 弦中点）：原来这里只取 a/d/弦中点三个，
		/// 而长或弯的引道「弦中点」会明显偏离曲线本体，出入口正好落在偏离段上时这条车道
		/// 就既进不了邻域集合、也过不了逐帧锚点判定 → 永久漏删。
		/// v0.8.3：锚点不再带类别位（三类勾选合并成一个整体，见 <see cref="IsAccessOrFacilityLane"/>），
		/// 网格值退回 FixedList128Bytes&lt;float2&gt;，每格容量也随之回到 14。</summary>
		private static bool IsNearAnchor(Bezier4x3 bezier, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
		{
			if (IsNearAnchor(bezier.a, grid)
				|| IsNearAnchor(0.25f * (bezier.a + bezier.b + bezier.b + bezier.c), grid)
				|| IsNearAnchor(0.5f * (bezier.a + bezier.d), grid)
				|| IsNearAnchor(0.25f * (bezier.b + bezier.c + bezier.c + bezier.d), grid)
				|| IsNearAnchor(bezier.d, grid))
			{
				return true;
			}
			return false;
		}

		private static bool IsNearAnchor(float3 position, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
		{
			int2 key = GridKey(position);
			float radiusSq = kAccessRadius * kAccessRadius;
			for (int dy = -1; dy <= 1; dy++)
			{
				for (int dx = -1; dx <= 1; dx++)
				{
					if (grid.TryGetValue(key + new int2(dx, dy), out FixedList128Bytes<float2> anchors))
					{
						for (int i = 0; i < anchors.Length; i++)
						{
							float2 d = anchors[i] - position.xz;
							if (math.lengthsq(d) <= radiusSq)
							{
								return true;
							}
						}
					}
				}
			}
			return false;
		}

		/// <summary>
		/// 随机访问（不是 chunk 遍历）版本的「这条车道在不避让范围内吗」。三份判定必须语义一致：
		/// 这里、BuildAccessSetsJob、StripPedestrianOverlapsJob（全局模式走 chunk 版
		/// IsGlobalLaneByChunk，出入口模式调本方法）。不一致就会出现
		/// "进了集合却判不成范围内"这类互相漏口的现象。
		/// 全局模式：所有车辆车道（含指向人行横道的条目，见 v0.7.7）。
		/// 出入口模式（v0.8.3 三合一）：出入口/建筑内部道路类车道 / 邻域集合成员 / 锚点邻域，三选一。
		/// ⚠ 人行横道不受这里影响：出入口模式一律传 includeCrosswalks=0，指向人行横道的条目永不删除
		/// （v0.5 的横道误伤修复），所以普通道路上的行人过街照常避让——作者 2026-10-01 明确要求
		/// 合并三类后"出入口依然不要影响人行横道等正常道路的避让"，靠的就是这一条 + 邻近兜底只服务出入口。
		/// </summary>
		private static bool IsInNoYieldScope(Entity lane, int mode, NativeHashSet<Entity> vehicleSet,
			ComponentLookup<Curve> curveData, NativeHashMap<int2, FixedList128Bytes<float2>> grid,
			ComponentLookup<PedestrianLane> pedestrianData,
			ComponentLookup<GarageLane> garageData, ComponentLookup<ConnectionLane> connectionData,
			ComponentLookup<AreaLane> areaData, ComponentLookup<CarLane> carData, ComponentLookup<ParkingLane> parkingData,
			ComponentLookup<Owner> ownerData, ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData)
		{
			// v0.8.4（玩家性能反馈的根因修复）：带 PedestrianLane 的实体（人行道段、横道、
			// 建筑行人连道）**两种模式都不进范围**。车辆不读行人道自己的 LaneOverlap，
			// 改它既不产生任何"车不让人"的效果，又让候选集、邻域集合、锚点池全都膨胀一个量级，
			// 还顺手违背了 README 里"行人行走行为不受影响"的承诺。
			if (pedestrianData.HasComponent(lane))
			{
				return false;
			}
			if (mode == Setting.kModeGlobal)
			{
				if (carData.HasComponent(lane) || parkingData.HasComponent(lane) || garageData.HasComponent(lane)
					|| areaData.HasComponent(lane))
				{
					return true;
				}
				if (connectionData.TryGetComponent(lane, out ConnectionLane globalConn)
					&& (globalConn.m_Flags & kVehicleConnectionFlags) != 0)
				{
					return true;
				}
				return false;
			}
			if (IsAccessOrFacilityLane(lane, pedestrianData, garageData, connectionData, areaData, carData, parkingData,
				ownerData, edgeData, nodeData))
			{
				return true;
			}
			// 不属于出入口/内部道路的那批（就是普通市政道路）才吃两条邻近兜底：
			// 邻域集合成员，或曲线 32 米内有出入口锚点。
			if (vehicleSet.IsCreated && vehicleSet.Contains(lane))
			{
				return true;
			}
			if (curveData.TryGetComponent(lane, out Curve curve) && IsNearAnchor(curve.m_Bezier, grid))
			{
				return true;
			}
			return false;
		}

		/// <summary>
		/// v0.8.3 统一判据 + v0.8.4 只认车行 + v0.8.5 货运连道回到组件判定：这条车道属不属于
		/// "出入口与建筑内部道路"这一整片。
		/// 命中任一条即是：① GarageLane（车库/坡道）；② **带车行位**的 ConnectionLane（建筑、停车场与
		/// 外部道路之间那段出入连接道）；③ AreaLane（区域内部车道，含地面停车场铺面与场区道路）；
		/// ④ 建筑自有的车辆车道（CarLane/ParkingLane 且 Owner 既不是 Edge 也不是 Node，即归某个建筑/设施
		/// 而不是归路网，见 IsBuildingOwned）。带 `PedestrianLane` 的实体一律出局。
		/// ②没命中不代表出局——连道之外还带 ①③④ 任一的（典型是 `Inside|AllowCargo` 的装卸口连道）
		/// 仍由后面几条接住。
		///
		/// ⚠ v0.8.4 之前这里是 `connectionData.HasComponent(lane) → true`，不看 flags：原版给纯行人连道
		/// 只打 `Inside|Pedestrian`（RoadConnectionSystem.cs:1336-1351/1380-1388），人行道与横道本身是
		/// 带 `PedestrianLane` 的车道，而车道类型位互斥（Pathfind/LaneDataSystem.cs:16-131：一条车道只能有
		/// CarLane / ParkingLane / PedestrianLane / NavalLane 之一，Area/Connection/Track/Garage 是形状位），
		/// 所以"有 ConnectionLane 就是出入口"把整条街的行人连道都当成了车辆出入口——作者实机
		/// `Anchor sources: access=106283 parking=4823` 就是这件事的读数（11 万 vs 4.8 千）。
		/// 判"是不是车走的"用官方同一组位：<see cref="kVehicleConnectionFlags"/>，与全局模式 v0.5 起的写法一致。
		///
		/// 为什么不再分"停车场 / 建筑出入口 / 建筑内部道路"（两轮实机 + 作者 2026-10-01 决定，
		/// 证据全文在开发笔记 §⑨2 与 §⑪）：
		/// ① 停车场的车行大门在数据上与商店大门是同一种 ConnectionLane —— RoadConnectionSystem.cs:1356-1363
		///    那条 else 分支只给 Inside|Road、不给 ConnectionLaneFlags.Parking；而 OutsideConnectionSystem
		///    那批的 Owner 是 node 甚至不加 Owner（:601-655/:702-707）。沿 Owner 链找"停车设施建筑"
		///    （照 ParkingLaneDataSystem.cs:344-382 写的 m_ParkingSet 预计算，实机 parkingOwners=181 条确实认到了）
		///    仍然覆盖不到这些大门，作者实机第二次仍然看到"勾第二项把停车场一起关掉"。
		/// ② 更根本的是：出入口在几何上就是紧贴普通道路和彼此，"32 米邻近"这条兜底天生跨类，
		///    任何子类开关都会把相邻子类一起扫进来（第一次修复让锚点按类过滤后，作者实测仍然覆盖）。
		/// ⇒ 与其留三个互相覆盖的开关让玩家困惑，不如一个开关管住整片出入口区域；
		///    需要更细粒度的用户，走 v0.9.0 的「自定义标记」（逐个点选，语义天然无歧义）。
		/// </summary>
		private static bool IsAccessOrFacilityLane(Entity lane, ComponentLookup<PedestrianLane> pedestrianData,
			ComponentLookup<GarageLane> garageData, ComponentLookup<ConnectionLane> connectionData,
			ComponentLookup<AreaLane> areaData,
			ComponentLookup<CarLane> carData, ComponentLookup<ParkingLane> parkingData,
			ComponentLookup<Owner> ownerData, ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData)
		{
			if (pedestrianData.HasComponent(lane))
			{
				return false;
			}
			if (connectionData.TryGetComponent(lane, out ConnectionLane accessConn)
				&& (accessConn.m_Flags & kVehicleConnectionFlags) != 0)
			{
				return true;
			}
			// v0.8.5：连道但**没有**那三位之一，不能就此判定"不是出入口"。
			// `RouteConnectionType.Cargo` 的建筑连道只打 `Inside|AllowCargo`
			// （RoadConnectionSystem.cs:1354，仓库/商超的装卸口、货台那一类），
			// 原来这里直接 `return false` 把整类货运用途的车行连道挡在出入口模式之外；
			// 而全局模式是先看 CarLane/ParkingLane/GarageLane/AreaLane 组件、**后**才看连道位
			// （见 IsInNoYieldScope 的 global 分支），同一批车道在两条模式下给出不同答案——
			// "有些停车场/仓库生效、有些不生效"正好是这个形状。落到下面的组件判定即可：
			// 纯行人连道已经在第一句被 PedestrianLane 挡掉，不会因此混进来。
			if (garageData.HasComponent(lane) || areaData.HasComponent(lane))
			{
				return true;
			}
			if ((carData.HasComponent(lane) || parkingData.HasComponent(lane))
				&& IsBuildingOwned(lane, ownerData, edgeData, nodeData))
			{
				return true;
			}
			return false;
		}

		/// <summary>全局模式的 chunk 级快速类型判定：任何车辆车道（CarLane/ParkingLane/GarageLane/AreaLane，
		/// 以及带 Road/Track/Parking  flags 的 ConnectionLane）一律在范围内。
		/// 出入口模式不走这里——那份判定要逐条车道看组件、邻域集合与锚点距离，
		/// 统一在 <see cref="IsInNoYieldScope"/> 里（需要逐车道的 ComponentLookup，没法用 chunk flag 代替）。</summary>
		private static bool IsGlobalLaneByChunk(bool chunkHasCarLane, bool chunkHasParking, bool chunkHasGarage,
			bool chunkHasConnection, bool chunkHasArea, NativeArray<ConnectionLane> connectionArray, int i)
		{
			if (chunkHasCarLane || chunkHasParking || chunkHasGarage || chunkHasArea)
			{
				return true;
			}
			if (chunkHasConnection && (connectionArray[i].m_Flags & (ConnectionLaneFlags.Road | ConnectionLaneFlags.Track | ConnectionLaneFlags.Parking)) != 0)
			{
				return true;
			}
			return false;
		}

		private static bool IsBuildingOwned(Entity lane, ComponentLookup<Owner> ownerData,
			ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData)
		{
			if (!ownerData.TryGetComponent(lane, out Owner owner) || owner.m_Owner == Entity.Null)
			{
				return false;
			}
			Entity ownerEntity = owner.m_Owner;
			return !edgeData.HasComponent(ownerEntity) && !nodeData.HasComponent(ownerEntity);
		}

		/// <summary>该 overlap 对端是否人行横道（审计分项用）。</summary>
		private static bool IsCrosswalkLane(Entity other, ComponentLookup<PedestrianLane> pedestrianLaneData)
		{
			if (pedestrianLaneData.TryGetComponent(other, out PedestrianLane ped))
			{
				return (ped.m_Flags & PedestrianLaneFlags.Crosswalk) != 0;
			}
			return false;
		}

		/// <summary>overlap 条目指向的是否为"行人道"。
		/// includeCrosswalks == 0 时人行横道（Crosswalk flag）算"不是行人道"，即条目保留：
		/// 出入口模式下这是 v0.5 的横道误伤修复（横道天然紧邻出入口）；
		/// 全局模式传 1 —— "全部道路"就要连横道也不让，见 v0.7.7。</summary>
		private static bool IsPedestrianLaneEntity(Entity other, ComponentLookup<PedestrianLane> pedestrianLaneData,
			ComponentLookup<ConnectionLane> connectionLaneData, int includeCrosswalks)
		{
			if (pedestrianLaneData.TryGetComponent(other, out PedestrianLane ped))
			{
				if ((ped.m_Flags & PedestrianLaneFlags.Crosswalk) != 0)
				{
					return includeCrosswalks != 0;
				}
				return true;
			}
			if (connectionLaneData.TryGetComponent(other, out ConnectionLane conn)
				&& (conn.m_Flags & ConnectionLaneFlags.Pedestrian) != 0
				&& (conn.m_Flags & (ConnectionLaneFlags.Road | ConnectionLaneFlags.Track | ConnectionLaneFlags.Parking)) == 0)
			{
				return true;
			}
			return false;
		}

		/// <summary>关闭功能兜底路径：给拥有者实体（带 SubLane 缓冲的 edge/node/building）打 Updated 脏标记，
		/// 这才是原版 LaneOverlapSystem 重建闸门与工作清单所要求的实体（v0.7.6）。
		/// 已有 Updated 的实体跳过——ECB 帧末回放时重复 AddComponent 不是我们要赌的语义。</summary>
		[BurstCompile]
		private struct MarkOwnersUpdatedJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public ComponentLookup<Updated> m_UpdatedData;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kSortKeyRestore）。</summary>
			public int m_SortKeyBase;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				for (int i = 0; i < chunk.Count; i++)
				{
					if (!m_UpdatedData.HasComponent(entities[i]))
					{
						m_Ecb.AddComponent<Updated>(m_SortKeyBase + unfilteredChunkIndex, entities[i]);
					}
				}
			}
		}

		/// <summary>兜底路径车道侧：给车道（含行人道，m_RestoreLaneQuery）打 Updated，
		/// 满足原版 m_UpdatedLanesQuery 的 flags/方向两趟；单靠它开不了重建闸门，见类注释 v0.7.6。</summary>
		[BurstCompile]
		private struct MarkLanesUpdatedJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public ComponentLookup<Updated> m_UpdatedData;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kSortKeyRestore）。</summary>
			public int m_SortKeyBase;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				for (int i = 0; i < chunk.Count; i++)
				{
					if (!m_UpdatedData.HasComponent(entities[i]))
					{
						m_Ecb.AddComponent<Updated>(m_SortKeyBase + unfilteredChunkIndex, entities[i]);
					}
				}
			}
		}

		/// <summary>
		/// 收集出入口锚点（连接道/车库坡道/区域车道的曲线控制点）到网格。
		/// 五个采样点与判定侧 <see cref="IsNearAnchor(Bezier4x3,NativeHashMap{int2,FixedList128Bytes{float2}})"/> 一致。
		///
		/// v0.8.3 三合一：这里**不再按勾选项过滤**（原 Setting.IncludeMask 分支随三个勾选框一起删除）。
		/// 历史原因记在这里以免再犯：v0.8.0~0.8.2 试图按"停车场 / 建筑出入口 / 内部道路"分别收录锚点，
		/// 但大门道本身在数据上分不开、32 米邻域兜底又天生跨类，作者三轮实测都是"互相覆盖"，
		/// 所以锚点池重新合成一份，判定侧也只有"在不在出入口范围内"一个问题。
		/// </summary>
		[BurstCompile]
		private struct BuildAnchorGridJob : IJob
		{
			[ReadOnly]
			[DeallocateOnJobCompletion]
			public NativeArray<Entity> m_Entities;

			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<GarageLane> m_GarageLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			[ReadOnly]
			public ComponentLookup<AreaLane> m_AreaLaneData;

			[ReadOnly]
			public ComponentLookup<CarLane> m_CarLaneData;

			[ReadOnly]
			public ComponentLookup<ParkingLane> m_ParkingLaneData;

			[ReadOnly]
			public ComponentLookup<Owner> m_OwnerData;

			[ReadOnly]
			public ComponentLookup<Edge> m_EdgeData;

			[ReadOnly]
			public ComponentLookup<Node> m_NodeData;

			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			/// <summary>诊断输出槽（见 kGridStatsSlots）。单线程 job，整轮重建时一次性赋值。</summary>
			public NativeArray<int> m_Stats;

			private int m_Dropped;
			private int m_MaxCell;
			private int m_Anchors;
			private int m_Cells;
			private int m_Sources;
			private int m_PedSkipped;
			private int m_Deduped;

			public void Execute()
			{
				m_Grid.Clear();
				m_Dropped = 0;
				m_MaxCell = 0;
				m_Anchors = 0;
				m_Cells = 0;
				m_Sources = 0;
				m_PedSkipped = 0;
				m_Deduped = 0;
				for (int i = 0; i < m_Entities.Length; i++)
				{
					Entity lane = m_Entities[i];
					if (m_PedestrianLaneData.HasComponent(lane))
					{
						m_PedSkipped++;
						continue;
					}
					if (!IsAccessOrFacilityLane(lane, m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData,
						m_AreaLaneData, m_CarLaneData, m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData))
					{
						continue;
					}
					m_Sources++;
					if (!m_CurveData.TryGetComponent(lane, out Curve curve))
					{
						continue;
					}
					Bezier4x3 bezier = curve.m_Bezier;
					AddAnchor(bezier.a);
					AddAnchor(0.25f * (bezier.a + bezier.b + bezier.b + bezier.c));
					AddAnchor(0.5f * (bezier.a + bezier.d));
					AddAnchor(0.25f * (bezier.b + bezier.c + bezier.c + bezier.d));
					AddAnchor(bezier.d);
				}
				if (m_Stats.IsCreated)
				{
					m_Stats[kGridDropped] = m_Dropped;
					m_Stats[kGridMaxCell] = m_MaxCell;
					m_Stats[kGridAnchors] = m_Anchors;
					m_Stats[kGridCells] = m_Cells;
					m_Stats[kGridSources] = m_Sources;
					m_Stats[kGridPedSkipped] = m_PedSkipped;
					m_Stats[kGridDeduped] = m_Deduped;
				}
			}

			/// <summary>往网格放一个出入口锚点。v0.8.3 三合一后锚点不再带类别位（三个勾选项合并成一个，
			/// 见 <see cref="IsAccessOrFacilityLane"/>），网格值回到 `float2`，闸门回到 14。
			/// 截断点是 <see cref="kAnchorPerCell"/>（我们自己设的闸门，容器本身还能再多一格），
			/// 被丢掉的锚点会让紧邻那条出入口的普通道路判不出"靠出入口太近"→ 漏删，
			/// 所以心跳那行 `Anchor grid: …` 会报 `… dropped`（stats[kGridDropped]），且仍然有截断时才是 WARN。</summary>
			private void AddAnchor(float3 position)
			{
				int2 key = GridKey(position);
				if (!m_Grid.TryGetValue(key, out FixedList128Bytes<float2> list))
				{
					list = default;
				}
				float2 point = position.xz;
				for (int i = 0; i < list.Length; i++)
				{
					if (math.lengthsq(list[i] - point) <= kAnchorDedupDistSq)
					{
						m_Deduped++;
						return;
					}
				}
				if (list.Length >= kAnchorPerCell)
				{
					m_Dropped++;
					return;
				}
				if (list.Length == 0)
				{
					m_Cells++;
				}
				list.Add(position.xz);
				m_Anchors++;
				if (list.Length > m_MaxCell)
				{
					m_MaxCell = list.Length;
				}
				m_Grid[key] = list;
			}
		}

		/// <summary>分帧构建邻域车辆集合：候选（m_OverlapQuery 实体）中类型出入口车道或邻近引道入集。
		/// access 模式的 A 只遍历此集合，不再对全城 chunk 迭代。</summary>
		[BurstCompile]
		private struct BuildAccessSetsJob : IJob
		{
			[ReadOnly]
			public NativeList<Entity> m_Candidates;

			public int m_Start;

			public int m_End;

			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<GarageLane> m_GarageLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			[ReadOnly]
			public ComponentLookup<AreaLane> m_AreaLaneData;

			[ReadOnly]
			public ComponentLookup<CarLane> m_CarLaneData;

			[ReadOnly]
			public ComponentLookup<ParkingLane> m_ParkingLaneData;

			[ReadOnly]
			public ComponentLookup<Owner> m_OwnerData;

			[ReadOnly]
			public ComponentLookup<Edge> m_EdgeData;

			[ReadOnly]
			public ComponentLookup<Node> m_NodeData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			public NativeHashSet<Entity> m_BuildVehicleSet;

			public void Execute()
			{
				int end = math.min(m_End, m_Candidates.Length);
				int start = math.max(0, m_Start); // 现在所有调用点都保证 >=0，但 -1 漏进来就是 m_Candidates[-1] 越界读
				for (int i = start; i < end; i++)
				{
					Entity lane = m_Candidates[i];
					// 行人道候选直接跳过：它们既不进集合，也不做那次 5 采样 × 9 邻格的锚点距离测试
					// （v0.8.4 玩家反馈里最贵的那一遍扫描，本来有将近一半花在不产生任何效果的行人道上）。
					if (m_PedestrianLaneData.HasComponent(lane))
					{
						continue;
					}
					bool isAccessLane = IsAccessOrFacilityLane(lane, m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData,
						m_AreaLaneData, m_CarLaneData, m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData);
					bool inScope = isAccessLane;
					if (!inScope)
					{
						// 只有普通道路车道（不属于出入口/内部道路那一片）才吃"紧邻出入口锚点"这条兜底。
						inScope = m_CurveData.TryGetComponent(lane, out Curve curve) && IsNearAnchor(curve.m_Bezier, m_Grid);
					}
					if (inScope)
					{
						m_BuildVehicleSet.Add(lane);
					}
				}
			}

			/// <summary>直接委托给系统级实现，避免各 job 里再抄一份采样点列表
			/// （v0.8.0 之前这三份 wrapper 各自写死 3 个采样点，与放锚点的 5 个点不一致）。</summary>
			private static bool IsNearAnchor(Bezier4x3 bezier, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
			{
				return AccessZoneOverlapSystem.IsNearAnchor(bezier, grid);
			}
		}

		/// <summary>
		/// v0.8.0 新增（停车场出入口仍会避让的根因修复）：跟随原版重建闸门的主动重删。
		///
		/// 遍历对象与原版 LaneOverlapSystem.UpdateLaneOverlapsJob 完全一致——
		/// 本帧带 Updated 的 owner 的 SubLane 缓冲，外加 AddNonUpdatedEdgesJob
		/// （LaneOverlapSystem.cs:26-54）会被拉进原版清单的那批「自身未 Updated 的相邻 edge」的 SubLane。
		/// 原版对这批实体是 `bufferData.Clear()` 后整表重算（:1077 → :1400/:1420），
		/// 我们删掉的行人条目就是在 Modification4B 那一刻回来的，而它不会给车道打 Updated，
		/// 所以只认车道 Updated 的旧路径收不到 → 那些出入口在下一次集合整轮重建前恢复避让。
		///
		/// 判据本身沿用与 (3) 相同的三重测试；没被原版改写过（或改写后确实没有行人条目）的车道
		/// 在行人条目计数为 0 时直接返回，不产生任何 ECB 写入，所以不会退回 v0.7.1 那种每帧写战。
		/// </summary>
		[BurstCompile]
		private struct StripRebuiltLanesJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public BufferLookup<SubLane> m_SubLanes;

			[ReadOnly]
			public BufferLookup<ConnectedEdge> m_ConnectedEdges;

			[ReadOnly]
			public ComponentLookup<Updated> m_UpdatedData;

			[ReadOnly]
			public BufferLookup<LaneOverlap> m_Overlaps;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			[ReadOnly]
			public ComponentLookup<GarageLane> m_GarageLaneData;

			[ReadOnly]
			public ComponentLookup<AreaLane> m_AreaLaneData;

			[ReadOnly]
			public ComponentLookup<CarLane> m_CarLaneData;

			[ReadOnly]
			public ComponentLookup<ParkingLane> m_ParkingLaneData;

			[ReadOnly]
			public ComponentLookup<Owner> m_OwnerData;

			[ReadOnly]
			public ComponentLookup<Edge> m_EdgeData;

			[ReadOnly]
			public ComponentLookup<Node> m_NodeData;

			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			[ReadOnly]
			public NativeHashSet<Entity> m_AccessVehicleSet;

			public int m_Mode;

			public int m_IncludeCrosswalks;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			[NativeDisableParallelForRestriction]
			public NativeArray<int> m_RepairCounters;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				NativeArray<Entity> owners = chunk.GetNativeArray(m_EntityType);
				// sortKey 与 (1)(2)(3) 那批用 unfilteredChunkIndex 的 job 隔开一个固定偏移：
				// 两条路径可能对同一条车道同时 SetBuffer，内容都是从同一份快照算出来的幂等结果，
				// 错开键位只是为了回放顺序可读，不承担正确性。
				int sortKey = kGateSortKeyBase + unfilteredChunkIndex;
				int slot = unfilteredChunkIndex * 2;
				for (int i = 0; i < chunk.Count; i++)
				{
					Entity owner = owners[i];
					if (m_SubLanes.TryGetBuffer(owner, out DynamicBuffer<SubLane> subLanes))
					{
						for (int j = 0; j < subLanes.Length; j++)
						{
							StripLane(subLanes[j].m_SubLane, sortKey, slot);
						}
					}
					if (!m_ConnectedEdges.TryGetBuffer(owner, out DynamicBuffer<ConnectedEdge> connectedEdges))
					{
						continue;
					}
					for (int k = 0; k < connectedEdges.Length; k++)
					{
						Entity neighbor = connectedEdges[k].m_Edge;
						// 自身已 Updated 的相邻 edge 会在自己的 chunk 记录里被处理，跳过以免重复。
						if (m_UpdatedData.HasComponent(neighbor))
						{
							continue;
						}
						if (!m_SubLanes.TryGetBuffer(neighbor, out DynamicBuffer<SubLane> neighborSubLanes))
						{
							continue;
						}
						for (int j = 0; j < neighborSubLanes.Length; j++)
						{
							StripLane(neighborSubLanes[j].m_SubLane, sortKey, slot);
						}
					}
				}
			}

			/// <summary>本帧被原版重算过的车道：先沿"反向条目"走一跳，再处理它自己。</summary>
			private void StripLane(Entity lane, int sortKey, int slot)
			{
				// v0.8.5（玩家反馈"部分停车场生效、部分不生效"的根因修复）。原版
				// UpdateLaneOverlapsJob 对**不同拥有者**的车道对不只写自己这一侧
				// （LaneOverlapSystem.cs:1401 `overlaps1.Add(...)`），还会把反向条目排队给
				// ApplyExtraOverlapsJob（:1404-1420 入队 → :915-918 `m_Overlaps[item.m_Entity].Add(...)`），
				// 写进**对面那条车道自己的缓冲**。对面车道的拥有者这一帧并没有 Updated，所以它既不在
				// 本 job 的扫描集合里（我们只跟着原主的 SubLane + 相邻 edge），也不会被 v0.8.4 之后
				// 的"条件重建"覆盖到——建筑状态变化/电梯一开只 Update 建筑那一侧，路口那条机动车道
				// 刚被原版写回的行人条目就一直留着 ⇒ 那条出入口又恢复避让。
				// 反向条目与正向条目是成对产生的，所以从"本帧被重算的行人道"出发沿它自己的
				// LaneOverlap 走一跳，取到的正是这批被漏掉的车辆车道（行人道缓冲里列的就是与它
				// 重叠的车辆车道，原版自己也是这么读的：TrafficLightInitializationSystem.cs:521-523）。
				if (m_PedestrianLaneData.HasComponent(lane)
					&& m_Overlaps.TryGetBuffer(lane, out DynamicBuffer<LaneOverlap> pedOverlaps))
				{
					for (int j = 0; j < pedOverlaps.Length; j++)
					{
						Entity other = pedOverlaps[j].m_Other;
						if (other == Entity.Null)
						{
							// 原版同样要先挡 null：Entity.Null 的 index 是 0，直接查会误碰 0 号实体。
							continue;
						}
						StripTarget(other, sortKey, slot);
					}
				}
				StripTarget(lane, sortKey, slot);
			}

			private void StripTarget(Entity lane, int sortKey, int slot)
			{
				if (!IsInNoYieldScope(lane, m_Mode, m_AccessVehicleSet, m_CurveData, m_Grid,
					m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData, m_ParkingLaneData,
					m_OwnerData, m_EdgeData, m_NodeData))
				{
					return;
				}
				if (!m_Overlaps.TryGetBuffer(lane, out DynamicBuffer<LaneOverlap> buffer) || buffer.Length == 0)
				{
					return;
				}
				int pedestrian = 0;
				for (int j = 0; j < buffer.Length; j++)
				{
					if (IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, m_IncludeCrosswalks))
					{
						pedestrian++;
					}
				}
				if (pedestrian == 0)
				{
					// 绝大多数车道停在这里就走：原版这一帧没改写它，或改写完本来就没有行人条目。
					return;
				}
				// SetBuffer 是整表替换，所以保留项必须按原序全部补写回去。
				DynamicBuffer<LaneOverlap> replacement = m_Ecb.SetBuffer<LaneOverlap>(sortKey, lane);
				for (int j = 0; j < buffer.Length; j++)
				{
					if (!IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, m_IncludeCrosswalks))
					{
						replacement.Add(buffer[j]);
					}
				}
				m_RepairCounters[slot]++;
				m_RepairCounters[slot + 1] += pedestrian;
			}
		}

		/// <summary>心跳自证用：统计邻域集合内的车辆车道当前还残留多少条行人 overlap 条目。
		/// 出入口模式恒传 includeCrosswalks=0（横道条目我们本来就不动，算进去会永远非 0）。</summary>
		[BurstCompile]
		private struct AuditAccessSetJob : IJob
		{
			[ReadOnly]
			public NativeHashSet<Entity> m_Set;

			[ReadOnly]
			public BufferLookup<LaneOverlap> m_Overlaps;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			public NativeArray<int> m_Counters;

			public void Execute()
			{
				foreach (Entity lane in m_Set)
				{
					if (!m_Overlaps.TryGetBuffer(lane, out DynamicBuffer<LaneOverlap> buffer) || buffer.Length == 0)
					{
						continue;
					}
					int hit = 0;
					for (int j = 0; j < buffer.Length; j++)
					{
						if (IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, 0))
						{
							hit++;
						}
					}
					if (hit > 0)
					{
						m_Counters[0]++;
						m_Counters[1] += hit;
					}
				}
			}
		}

		/// <summary>access 模式干预点 A：只遍历邻域车辆集合，删除指向行人道的 overlap 条目。
		/// 不再 ScheduleParallel 全城 m_OverlapQuery——那是 v0.7.0 性能重构后残留的 O(全城) 热点。</summary>
		[BurstCompile]
		private struct StripAccessOverlapsFromSetJob : IJob
		{
			[ReadOnly]
			public NativeHashSet<Entity> m_Set;

			[ReadOnly]
			public BufferLookup<LaneOverlap> m_Overlaps;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kSortKeyFromSet）。</summary>
			public int m_SortKeyBase;

			public void Execute()
			{
				if (m_Set.Count == 0)
				{
					return;
				}
				int sortKey = m_SortKeyBase;
				foreach (Entity lane in m_Set)
				{
					if (!m_Overlaps.TryGetBuffer(lane, out DynamicBuffer<LaneOverlap> buffer) || buffer.Length == 0)
					{
						continue;
					}
					if (!HasPedestrianOverlap(buffer))
					{
						continue;
					}
					DynamicBuffer<LaneOverlap> replacement = m_Ecb.SetBuffer<LaneOverlap>(sortKey++, lane);
					for (int j = 0; j < buffer.Length; j++)
					{
						// 本 job 只在出入口模式跑：横道条目一律保留（v0.5 误伤修复）。
						if (!IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, 0))
						{
							replacement.Add(buffer[j]);
						}
					}
				}
			}

			private bool HasPedestrianOverlap(DynamicBuffer<LaneOverlap> buffer)
			{
				for (int i = 0; i < buffer.Length; i++)
				{
					if (IsPedestrianLaneEntity(buffer[i].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, 0))
					{
						return true;
					}
				}
				return false;
			}
		}

		/// <summary>
		/// 干预点 A：删除车辆车道 LaneOverlap 缓冲中指向行人道的条目。
		/// 全局模式：所有车辆车道都删（含指向人行横道的条目，m_IncludeCrosswalks=1）。
		/// access 模式：类型判定 / 邻域集合成员 / 锚点邻域三选一，横道条目一律保留。
		/// 既用于世界加载后的首次全量（m_OverlapQuery），也用于每帧事件驱动重刷（m_RefreshQuery）。
		/// </summary>
		[BurstCompile]
		private struct StripPedestrianOverlapsJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			[ReadOnly]
			public ComponentLookup<GarageLane> m_GarageLaneData;

			[ReadOnly]
			public ComponentLookup<Owner> m_OwnerData;

			[ReadOnly]
			public ComponentLookup<Edge> m_EdgeData;

			[ReadOnly]
			public ComponentLookup<Node> m_NodeData;

			[ReadOnly]
			public ComponentTypeHandle<CarLane> m_CarLaneType;

			[ReadOnly]
			public ComponentTypeHandle<ParkingLane> m_ParkingLaneType;

			[ReadOnly]
			public ComponentTypeHandle<GarageLane> m_GarageLaneType;

			[ReadOnly]
			public ComponentTypeHandle<ConnectionLane> m_ConnectionLaneType;

			[ReadOnly]
			public ComponentTypeHandle<AreaLane> m_AreaLaneType;

			[ReadOnly]
			public BufferTypeHandle<LaneOverlap> m_OverlapType;

			/// <summary>access 模式事件驱动重刷用：曲线数据 + 锚点网格，做 IsNearAnchor 判定。
			/// 捕捉刚建好、还没进邻域集合但已邻近锚点的车道。</summary>
			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			[ReadOnly]
			public NativeHashSet<Entity> m_AccessVehicleSet;

			[ReadOnly]
			public ComponentLookup<CarLane> m_CarLaneData;

			[ReadOnly]
			public ComponentLookup<ParkingLane> m_ParkingLaneData;

			[ReadOnly]
			public ComponentLookup<AreaLane> m_AreaLaneData;

			public int m_Mode;

			/// <summary>是否连人行横道（PedestrianLaneFlags.Crosswalk）的条目一起删：
			/// 全局模式 = 1（"全部道路"含横道），出入口模式 = 0（保留 v0.5 的横道误伤修复）。</summary>
			public int m_IncludeCrosswalks;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kGateSortKeyBase / kSortKeyRefresh 等常量的注释）：
			/// 同一条车道一帧内可能被多批命令整表覆盖，基址互不重叠才能保证"后跑的、快照更新的"赢。</summary>
			public int m_SortKeyBase;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				bool chunkHasCarLane = chunk.Has(ref m_CarLaneType);
				bool chunkHasParking = chunk.Has(ref m_ParkingLaneType);
				bool chunkHasGarage = chunk.Has(ref m_GarageLaneType);
				bool chunkHasConnection = chunk.Has(ref m_ConnectionLaneType);
				bool chunkHasArea = chunk.Has(ref m_AreaLaneType);
				if (!chunkHasCarLane && !chunkHasParking && !chunkHasGarage && !chunkHasConnection && !chunkHasArea)
				{
					return;
				}
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				BufferAccessor<LaneOverlap> overlaps = chunk.GetBufferAccessor(ref m_OverlapType);
				NativeArray<ConnectionLane> connectionArray = chunkHasConnection ? chunk.GetNativeArray(ref m_ConnectionLaneType) : default;

				for (int i = 0; i < chunk.Count; i++)
				{
					Entity lane = entities[i];
					// v0.8.4：行人道实体（人行道段/横道/建筑行人连道）不是我们要改的"持有者"。
					// 全局模式以前只靠 chunk 类型位过滤，AreaLane 会把行人广场路径一起捞进来；
					// 这里一次前置排除，两条模式共用同一个口径（与 IsInNoYieldScope 内那道守卫一致）。
					if (m_PedestrianLaneData.HasComponent(lane))
					{
						continue;
					}
					bool inScope;
					if (m_Mode == Setting.kModeGlobal)
					{
						// 全局模式：chunk 级类型快判（全城车辆车道一律删，不看邻域）。
						inScope = IsGlobalLaneByChunk(chunkHasCarLane, chunkHasParking, chunkHasGarage,
							chunkHasConnection, chunkHasArea, connectionArray, i);
					}
					else
					{
						// 出入口模式三重判定：出入口/内部道路类型 / 邻域集合成员 / 锚点邻域。
						// 与 BuildAccessSetsJob、StripRebuiltLanesJob 共用 IsInNoYieldScope，防止三份判定再度漂移。
						inScope = IsInNoYieldScope(lane, m_Mode, m_AccessVehicleSet, m_CurveData, m_Grid,
							m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData, m_ParkingLaneData,
							m_OwnerData, m_EdgeData, m_NodeData);
					}
					if (!inScope)
					{
						continue;
					}
					DynamicBuffer<LaneOverlap> buffer = overlaps[i];
					if (!HasPedestrianOverlap(buffer))
					{
						continue;
					}
					DynamicBuffer<LaneOverlap> replacement = m_Ecb.SetBuffer<LaneOverlap>(m_SortKeyBase + unfilteredChunkIndex, lane);
					for (int j = 0; j < buffer.Length; j++)
					{
						if (!IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, m_IncludeCrosswalks))
						{
							replacement.Add(buffer[j]);
						}
					}
				}
			}

			private bool HasPedestrianOverlap(DynamicBuffer<LaneOverlap> buffer)
			{
				for (int i = 0; i < buffer.Length; i++)
				{
					if (IsPedestrianLaneEntity(buffer[i].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, m_IncludeCrosswalks))
					{
						return true;
					}
				}
				return false;
			}
		}

		/// <summary>关闭恢复自检（v0.7.6，诊断用）：统计 m_OverlapQuery 匹配的车辆车道里仍有几条
		/// 指向行人道的 LaneOverlap 条目，并单列其中指向人行横道的那部分（v0.7.7 起全局模式
		/// 连横道条目也删，所以"横道条目回来了"才是原版真正恢复的证据）。
		/// 每块写 kAuditCountersPerChunk 个 int 避免原子争用，主线程求和。
		/// 总数 &gt; 0 = 原版重建已把行人条目写回；== 0 = 仍处于被剥离状态。</summary>
		[BurstCompile]
		private struct AuditPedestrianOverlapsJob : IJobChunk
		{
			[ReadOnly]
			public BufferTypeHandle<LaneOverlap> m_OverlapType;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			[NativeDisableParallelForRestriction]
			public NativeArray<int> m_Counters;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				BufferAccessor<LaneOverlap> overlaps = chunk.GetBufferAccessor(ref m_OverlapType);
				int lanes = 0;
				int entries = 0;
				int crosswalkLanes = 0;
				int crosswalkEntries = 0;
				for (int i = 0; i < chunk.Count; i++)
				{
					DynamicBuffer<LaneOverlap> buffer = overlaps[i];
					int hit = 0;
					int crosswalkHit = 0;
					for (int j = 0; j < buffer.Length; j++)
					{
						// 审计要的是"原版重建有没有发生过"，所以横道条目也计入（includeCrosswalks=1）：
						// 全局模式下我们连横道都删，只数非横道会误判。
						if (IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, 1))
						{
							hit++;
							if (IsCrosswalkLane(buffer[j].m_Other, m_PedestrianLaneData))
							{
								crosswalkHit++;
							}
						}
					}
					if (hit > 0)
					{
						lanes++;
						entries += hit;
					}
					if (crosswalkHit > 0)
					{
						crosswalkLanes++;
						crosswalkEntries += crosswalkHit;
					}
				}
				int baseIndex = unfilteredChunkIndex * kAuditCountersPerChunk;
				m_Counters[baseIndex] = lanes;
				m_Counters[baseIndex + 1] = entries;
				m_Counters[baseIndex + 2] = crosswalkLanes;
				m_Counters[baseIndex + 3] = crosswalkEntries;
			}
		}
	}
}

