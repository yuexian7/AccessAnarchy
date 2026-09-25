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
	/// - 集合重建还带 Dependency.Complete() + 全城 ToEntityArray，周期性卡主线程。
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
	/// 3. 邻域集合改为低频整轮重建（kSetRebuildInterval），去掉每帧增量 IsNearAnchor；
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
	/// 只能等下一次集合整轮重建（kSetRebuildInterval=2048 帧 ≈ 34 秒）由集合内批量重删捡回来；
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
	/// v0.8.0 第二轮：「不避让范围」三个勾选框此前完全无效（作者实机反馈：关任何一个都不改变行为）。
	/// 根因是锚点网格不分类型：BuildAnchorGridJob 把所有 Garage/Connection/Area 车道的曲线都收进网格，
	/// 而出入口模式的判据是「类型 / 集合成员 / 锚点邻域」三选一 —— 即使按类型排除了某一类，
	/// 那一类旁边的其它类锚点照样让邻近车辆车道命中「锚点邻域」，于是开关形同虚设。
	/// 现在：① 三个开关重定义为互不重叠的三类（停车场 / 建筑与外部道路的车辆出入口 / 建筑内部道路，
	/// 见 LaneCategory），② 锚点网格与邻域集合都按当下 IncludeMask 过滤后才收录，
	/// ③ 三份类型判定收敛到 LaneCategory + IsInNoYieldScope 一处，避免再次各写一份而漂移。
	/// "停车场"按所属建筑带不带 ParkingFacility/CarParkingFacility 判定，不按车道组件判定
	/// （停车场门口那条连接道和商店门口那条是同一个 ConnectionLane），所以勾第二条不会连带停车场。
	/// 开关变化仍走 v0.7.6/0.7.8 的「先让原版整表重建、再按新范围重删」路径（narrowed 分支），
	/// 所以关掉的类别会在 kReapplyDelayFrames(120 帧) 后恢复原版避让，不会留下永久剥离态。
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
		/// <summary>锚点网格 + 邻域集合的整轮重建间隔（模拟帧）。连接道/道路只在修路建拆时变化。</summary>
		private const int kSetRebuildInterval = 2048;

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

		/// <summary>重删计数数组：64 个槽 × 2（该槽内「修好的车道数」「删掉的行人条目数」）。
		/// 按 unfilteredChunkIndex &amp; 63 取槽，槽位撞车时计数偏低——它只是诊断用的下界，
		/// 不参与任何行为判断，所以不做原子操作（Burst 里加锁/原子的写法风险更大）。</summary>
		private const int kRepairSlots = 64;

		/// <summary>锚点网格诊断槽：0=被容量上限丢掉的锚点数，1=单格最多锚点数，2=锚点总数，3=占用格数。</summary>
		private const int kGridStatsSlots = 4;

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
		/// 置位后在下一轮集合起始处（已 Dependency.Complete，无在飞 job）把两侧一起清空，
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
		private int m_LastIncludeMask = -1;

		private bool m_LoggedFirstRun;
		private ulong m_TotalPasses;

		/// <summary>重删诊断计数（见 kRepairSlots）：由 StripRebuiltLanesJob 写，心跳时读完清零。</summary>
		private NativeArray<int> m_RepairCounters;

		/// <summary>锚点网格诊断计数（见 kGridStatsSlots）：BuildAnchorGridJob 单线程写，心跳时读。</summary>
		private NativeArray<int> m_GridStats;

		/// <summary>本心跳周期内原版重建闸门被打开的帧数，以及最后一帧的 owner 数（主线程计数，无竞态）。</summary>
		private int m_GateFrames;
		private int m_GateLastOwners;

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
			m_RepairCounters = new NativeArray<int>(kRepairSlots * 2, Allocator.Persistent, NativeArrayOptions.ClearMemory);
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

		protected override void OnUpdate()
		{
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
					m_RestoreCheckGlobal = setting.Mode == Setting.kModeGlobal;
					RestoreVanillaOverlaps();
					// 清空所有增量状态：下次启用时重新做全量 A + 重建邻域集合。
					m_InitialGlobalApplyDone = false;
					m_SetsInitialized = false;
					m_BuildInProgress = false;
					m_Cursor = -1;
					m_LastMode = -1;
					m_LastIncludeMask = -1;
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
			// 模式/范围开关变化时立刻作废，并走「先让原版重建、再按新范围重删」一轮。
			int includeMask = setting.IncludeMask;
			if (setting.Mode != m_LastMode || includeMask != m_LastIncludeMask)
			{
				// 只有"范围变窄"才需要还原重刷：变宽时多删几条就行，变窄时先前删过的范围外车道
				// 不会自己回来（v0.7.6 的同一个根因），必须让原版先重建一次再按新范围重删。
				// 注意 Include* 只在全局模式之外有意义（全局分支压根不看它们），
				// 所以全局模式下改 Include* 不算收窄，别白跑一次全城重建 + 空窗。
				bool toAccessMode = setting.Mode != Setting.kModeGlobal;
				bool narrowed = (m_LastMode == Setting.kModeGlobal && !toAccessMode)
					|| (toAccessMode && m_LastIncludeMask >= 0 && (m_LastIncludeMask & ~includeMask) != 0);
				m_LastMode = setting.Mode;
				m_LastIncludeMask = includeMask;
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

			bool needAccessSets = !globalMode;
			bool setsJustReady = false;
			if (needAccessSets && (m_BuildInProgress || !m_SetsInitialized || frameIndex % kSetRebuildInterval == 0))
			{
				if (!m_BuildInProgress && frameIndex % kSetRebuildInterval == 0)
				{
					// 到点强制起新一轮（旧活跃集在重建期间仍可继续用）。
					m_Cursor = -1;
				}
				setsJustReady = AdvanceAccessSets(setting, frameIndex);
			}

			// 干预点 A 调度（事件驱动，不再用 kReapplyInterval 定时器）：
			// 1. 全局模式世界加载后首次全量 A（m_OverlapQuery 全城 ScheduleParallel）
			// 2. access 模式集合刚建好时批量 A（StripAccessOverlapsFromSetJob 遍历车辆集合）
			// 3. 每帧事件驱动重刷（m_RefreshQuery 只匹配带 Updated 的车道，稳态 0 实体）
			//    LaneOverlapSystem 在 Modification4B 重建 overlap 后，Updated 仍在 → 当帧重删。
			EntityCommandBuffer.ParallelWriter ecb = m_EndFrameBarrier.CreateCommandBuffer().AsParallelWriter();
			bool scheduled = false;

			// (0) v0.8.0 主机制：跟随原版重建闸门重删。
			// 原版 LaneOverlapSystem 在 Modification4B（本阶段之前）把这一批 owner 的 SubLane
			// 逐条 Clear 后重算 overlap，我们删掉的行人条目就是在这一刻回来的；它不给车道打 Updated，
			// 所以只看车道 Updated 的 (3) 收不到这批车道 → 出入口会在下一次集合整轮重建
			// （kSetRebuildInterval=2048 帧 ≈ 34 秒）之前一直恢复避让，这正是「个别停车场出入口仍会避让」；
			// 全局模式更糟：它没有周期性重删，条目一旦回来就永远回来。
			// 现在用与原版同一个闸门、同一批实体（owner 的 SubLane + Updated 节点的相邻 edge），
			// 当帧按同一套判定重删；没被改写过的车道被 HasPedestrianOverlap 直接跳过，不产生写入。
			if (!m_RebuildOwnerQuery.IsEmptyIgnoreFilter)
			{
				m_GateFrames++;
				m_GateLastOwners = m_RebuildOwnerQuery.CalculateEntityCount();
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
					m_CarFacilityData = GetComponentLookup<CarParkingFacility>(isReadOnly: true),
					m_ParkingFacilityData = GetComponentLookup<ParkingFacility>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_IncludeMask = includeMask,
					m_IncludeCrosswalks = globalMode ? 1 : 0,
					m_Ecb = ecb,
					m_RepairCounters = m_RepairCounters
				};
				Dependency = rebuildJob.ScheduleParallel(m_RebuildOwnerQuery, Dependency);
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
					m_CarFacilityData = GetComponentLookup<CarParkingFacility>(isReadOnly: true),
					m_ParkingFacilityData = GetComponentLookup<ParkingFacility>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = default,
					m_Mode = setting.Mode,
					m_IncludeMask = includeMask,
					m_IncludeCrosswalks = 1,
					m_Ecb = ecb
				};
				Dependency = overlapJob.ScheduleParallel(m_OverlapQuery, Dependency);
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
				Dependency = accessJob.Schedule(Dependency);
				scheduled = true;
			}

			// (3) 每帧事件驱动重刷：只处理带 Updated 的车道（LaneOverlapSystem 刚重建过 overlap 的）。
			// 稳态下 m_RefreshQuery 匹配 0 实体，job 空转成本 ≈ 0。
			// 全局模式：所有 Updated 车辆车道都删；access 模式：额外检查类型/集合/锚点邻域。
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
					m_CarFacilityData = GetComponentLookup<CarParkingFacility>(isReadOnly: true),
					m_ParkingFacilityData = GetComponentLookup<ParkingFacility>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_IncludeMask = includeMask,
					m_IncludeCrosswalks = globalMode ? 1 : 0,
					m_Ecb = ecb
				};
				Dependency = refreshJob.ScheduleParallel(m_RefreshQuery, Dependency);
				scheduled = true;
			}

			if (scheduled)
			{
				m_EndFrameBarrier.AddJobHandleForProducer(Dependency);
			}
		}

		/// <summary>
		/// 心跳自证日志（v0.8.0）。用户回报「某个出入口仍会避让」时先看这三行：
		/// ① Rebuild gate —— 原版这一阵子改写过 overlap 没有、我们跟上了多少；
		/// ② Anchor grid —— 锚点容量有没有被卡掉（卡掉就会有邻近车道判不出靠出入口）；
		/// ③ Access set audit —— 集合内的车辆车道此刻还留着几条行人条目。
		/// ①非零而③也非零 = 我们没跟全（漏口还在）；①=0 且③=0 却仍然避让 = 不是 LaneOverlap 这条通道，
		/// 而是类注释 v0.7.2 里保留的原版行为（行人就站在车道本体上、或行人静止不动被当成静态障碍）。
		/// </summary>
		private void ReportRebuildDiagnostics(bool globalMode)
		{
			// 读 job 写过的计数数组之前必须等在飞的 job 收尾（与 ScanPedestrianOverlaps 同一套做法）。
			Dependency.Complete();
			int lanes = 0;
			int entries = 0;
			for (int i = 0; i < kRepairSlots; i++)
			{
				lanes += m_RepairCounters[i * 2];
				entries += m_RepairCounters[i * 2 + 1];
				m_RepairCounters[i * 2] = 0;
				m_RepairCounters[i * 2 + 1] = 0;
			}
			AccessAnarchyMod.log.Info($"Rebuild gate: {m_GateFrames} frame(s) with vanilla LaneOverlap rebuild this period, last gate {m_GateLastOwners} owner(s); re-stripped {lanes} lane(s) / {entries} pedestrian entries (lower bound, {kRepairSlots} slots).");
			m_GateFrames = 0;
			m_GateLastOwners = 0;

			int dropped = m_GridStats.IsCreated ? m_GridStats[0] : 0;
			if (dropped > 0)
			{
				AccessAnarchyMod.log.Warn($"Anchor grid capacity hit: {dropped} anchor point(s) dropped last grid rebuild (max {m_GridStats[1]} per cell, {m_GridStats[2]} anchors in {m_GridStats[3]} cells) - lanes near a crowded access cluster may stay un-stripped. Please send this line.");
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
			// 把 Dependency 传进去（而不是 Schedule()）：这样 ECS 安全检查知道这次读挂在本系统的
			// 依赖链上，与 ScanPedestrianOverlaps 里 ScheduleParallel(q, Dependency) 同一套做法。
			auditJob.Schedule(Dependency).Complete();
			int lanesWithEntries = counters[0];
			int entries = counters[1];
			counters.Dispose();
			AccessAnarchyMod.log.Info($"Access set audit: {lanesWithEntries} of {set.Count} vehicle lanes still hold {entries} pedestrian overlap entries (0 = clean, crosswalk entries excluded by design).");
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
			EntityCommandBuffer.ParallelWriter ecb = m_EndFrameBarrier.CreateCommandBuffer().AsParallelWriter();
			MarkOwnersUpdatedJob ownerJob = new MarkOwnersUpdatedJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_UpdatedData = GetComponentLookup<Updated>(isReadOnly: true),
				m_Ecb = ecb
			};
			JobHandle ownerHandle = ownerJob.ScheduleParallel(m_OwnerQuery, Dependency);
			MarkLanesUpdatedJob laneJob = new MarkLanesUpdatedJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_UpdatedData = GetComponentLookup<Updated>(isReadOnly: true),
				m_Ecb = ecb
			};
			JobHandle laneHandle = laneJob.ScheduleParallel(m_RestoreLaneQuery, Dependency);
			Dependency = JobHandle.CombineDependencies(ownerHandle, laneHandle);
			m_EndFrameBarrier.AddJobHandleForProducer(Dependency);
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
			Dependency.Complete();
			int chunkCount = m_OverlapQuery.CalculateChunkCount();
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
			auditJob.ScheduleParallel(m_OverlapQuery, Dependency).Complete();
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
		private bool AdvanceAccessSets(Setting setting, uint frameIndex)
		{
			// 锚点网格与邻域集合都按当下勾选掩码收集；掩码一变，OnUpdate 会作废整轮并排重建。
			int includeMask = setting.IncludeMask;
			if (m_BuildInProgress && m_Cursor >= 0 && m_Cursor >= m_Candidates.Length)
			{
				// 本轮扫完：交换双缓冲，活跃集可用。
				// 先等上一帧的 BuildAccessSetsJob（写新活跃集）与重刷 job（读旧活跃集）收尾：
				// 下面的 Clear() 与 Count 直读都要求这些并发句柄已释放。
				Dependency.Complete();
				m_ActiveSetIndex = 1 - m_ActiveSetIndex;
				m_SetsInitialized = true;
				m_BuildInProgress = false;
				int buildIndex = 1 - m_ActiveSetIndex;
				m_VehicleSets[buildIndex].Clear();
				m_Cursor = m_Candidates.Length;
				return m_VehicleSets[m_ActiveSetIndex].Count > 0;
			}

			if (!m_BuildInProgress)
			{
				// 起新一轮。旧活跃集继续给判定用；新内容写入另一侧缓冲。
				Dependency.Complete();

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
					m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
					m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
					m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
					m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
					m_CarFacilityData = GetComponentLookup<CarParkingFacility>(isReadOnly: true),
					m_ParkingFacilityData = GetComponentLookup<ParkingFacility>(isReadOnly: true),
					m_IncludeMask = includeMask,
					m_Grid = m_AnchorGrid,
					m_Stats = m_GridStats
				};
				Dependency = buildJob.Schedule(Dependency);

				int buildSide = 1 - m_ActiveSetIndex;
				m_VehicleSets[buildSide].Clear();

				m_Candidates.Clear();
				m_Candidates.AddRange(m_OverlapQuery.ToEntityArray(Allocator.Temp));
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
				m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
				m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
				m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
				m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
				m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
				m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
				m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
				m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
				m_CarFacilityData = GetComponentLookup<CarParkingFacility>(isReadOnly: true),
				m_ParkingFacilityData = GetComponentLookup<ParkingFacility>(isReadOnly: true),
				m_IncludeMask = includeMask,
				m_Grid = m_AnchorGrid,
				m_BuildVehicleSet = m_VehicleSets[1 - m_ActiveSetIndex]
			};
			Dependency = job.Schedule(Dependency);
			m_Cursor = endIndex;
			return false;
		}

		private static int2 GridKey(float3 position)
		{
			return (int2)math.floor(position.xz / kGridCellSize);
		}

		/// <summary>曲线是否邻近任何出入口锚点。v0.8.0 起采样点与 BuildAnchorGridJob 放锚点时用的
		/// 五个点一致（两端 + 两个四分位 + 弦中点）：原来这里只取 a/d/弦中点三个，
		/// 而长或弯的引道「弦中点」会明显偏离曲线本体，出入口正好落在偏离段上时这条车道
		/// 就既进不了邻域集合、也过不了逐帧锚点判定 → 永久漏删。</summary>
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

		/// <summary>
		/// 随机访问（不是 chunk 遍历）版本的「这条车道在不避让范围内吗」。三份判定必须语义一致：
		/// 这里、BuildAccessSetsJob.IsAccessType、StripPedestrianOverlapsJob（全局模式走 chunk 版
		/// IsGlobalLaneByChunk，出入口模式调本方法）。不一致就会出现
		/// "进了集合却判不成范围内"这类互相漏口的现象。
		/// 全局模式：所有车辆车道（含指向人行横道的条目，见 v0.7.7）。
		/// 出入口模式：类型属于某个已勾选类别 / 邻域集合成员 / 锚点邻域，三选一。
		/// </summary>
		private static bool IsInNoYieldScope(Entity lane, int mode, int includeMask, NativeHashSet<Entity> vehicleSet,
			ComponentLookup<Curve> curveData, NativeHashMap<int2, FixedList128Bytes<float2>> grid,
			ComponentLookup<GarageLane> garageData, ComponentLookup<ConnectionLane> connectionData,
			ComponentLookup<AreaLane> areaData, ComponentLookup<CarLane> carData, ComponentLookup<ParkingLane> parkingData,
			ComponentLookup<Owner> ownerData, ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData,
			ComponentLookup<CarParkingFacility> carFacilityData, ComponentLookup<ParkingFacility> facilityData)
		{
			if (mode == Setting.kModeGlobal)
			{
				if (carData.HasComponent(lane) || parkingData.HasComponent(lane) || garageData.HasComponent(lane)
					|| areaData.HasComponent(lane))
				{
					return true;
				}
				if (connectionData.TryGetComponent(lane, out ConnectionLane globalConn)
					&& (globalConn.m_Flags & (ConnectionLaneFlags.Road | ConnectionLaneFlags.Track | ConnectionLaneFlags.Parking)) != 0)
				{
					return true;
				}
				return false;
			}
			int category = LaneCategory(lane, garageData, connectionData, areaData, carData, parkingData,
				ownerData, edgeData, nodeData, carFacilityData, facilityData);
			if (category != 0 && (includeMask & category) != 0)
			{
				return true;
			}
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
		/// 车道属于三类中的哪一类（Setting.kCatParking / kCatBuildingAccess / kCatBuildingInternal，0=不属于）。
		///
		/// 停车场这一维**不能**靠车道组件区分：地面停车场的出入口连接道和商店的出入连接道是同一个东西
		/// （RoadConnectionSystem.cs:1356-1363 给两者的 flags 都是 Inside|Road），
		/// ConnectionLaneFlags.Parking 标的是车位/自行车架那种 berth 链接（:1370-1391），
		/// 停车场大门口那条路反而没有它。所以看**所属建筑**：
		/// 建筑出入车道的 Owner 直接就是建筑实体（RoadConnectionSystem.cs:1332 `new Owner(building)`），
		/// 而只有停车场/停车楼这类建筑才带 Game.Buildings.ParkingFacility / CarParkingFacility
		/// ——反证在 ParkingLaneDataSystem.cs:405-418：没有 ParkingFacilityData 的建筑（商店/办公/工业）
		/// 会退化到用 BuildingPropertyData.m_SpaceMultiplier 或 WorkplaceData.m_MaxWorkers/20 估容量，
		/// 说明游戏自己就是拿这个组件区分"是不是停车场"。
		/// 这样勾「建筑与外部道路车辆出入口」不会连带把停车场出入口也取消避让（作者 2026-09-25 的硬要求）。
		/// </summary>
		private static int LaneCategory(Entity lane, ComponentLookup<GarageLane> garageData,
			ComponentLookup<ConnectionLane> connectionData, ComponentLookup<AreaLane> areaData,
			ComponentLookup<CarLane> carData, ComponentLookup<ParkingLane> parkingData,
			ComponentLookup<Owner> ownerData, ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData,
			ComponentLookup<CarParkingFacility> carFacilityData, ComponentLookup<ParkingFacility> facilityData)
		{
			bool isGarage = garageData.HasComponent(lane);
			bool isConnection = connectionData.HasComponent(lane);
			bool isArea = areaData.HasComponent(lane);
			bool isCarOrParking = carData.HasComponent(lane) || parkingData.HasComponent(lane);
			if (isGarage)
			{
				return Setting.kCatParking;
			}
			if (IsParkingFacilityOwned(lane, ownerData, carFacilityData, facilityData))
			{
				if (isConnection || isArea || isCarOrParking)
				{
					return Setting.kCatParking;
				}
				return 0;
			}
			if (isConnection)
			{
				return Setting.kCatBuildingAccess;
			}
			if (isArea)
			{
				return Setting.kCatBuildingInternal;
			}
			if (isCarOrParking && IsBuildingOwned(lane, ownerData, edgeData, nodeData))
			{
				return Setting.kCatBuildingInternal;
			}
			return 0;
		}

		/// <summary>车道所属实体是不是停车场/停车楼建筑。</summary>
		private static bool IsParkingFacilityOwned(Entity lane, ComponentLookup<Owner> ownerData,
			ComponentLookup<CarParkingFacility> carFacilityData, ComponentLookup<ParkingFacility> facilityData)
		{
			if (!ownerData.TryGetComponent(lane, out Owner owner) || owner.m_Owner == Entity.Null)
			{
				return false;
			}
			return carFacilityData.HasComponent(owner.m_Owner) || facilityData.HasComponent(owner.m_Owner);
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

		/// <summary>全局模式的 chunk 级快速类型判定：任何车辆车道（CarLane/ParkingLane/GarageLane/AreaLane，
		/// 以及带 Road/Track/Parking  flags 的 ConnectionLane）一律在范围内。
		/// 出入口模式不走这里——那份判定要看三类勾选与所属建筑是不是停车场，
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

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				for (int i = 0; i < chunk.Count; i++)
				{
					if (!m_UpdatedData.HasComponent(entities[i]))
					{
						m_Ecb.AddComponent<Updated>(unfilteredChunkIndex, entities[i]);
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

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				for (int i = 0; i < chunk.Count; i++)
				{
					if (!m_UpdatedData.HasComponent(entities[i]))
					{
						m_Ecb.AddComponent<Updated>(unfilteredChunkIndex, entities[i]);
					}
				}
			}
		}

		/// <summary>
		/// 收集出入口锚点（连接道/车库坡道/区域车道的曲线控制点）到网格。
		/// 五个采样点与判定侧 <see cref="IsNearAnchor(Bezier4x3, NativeHashMap{int2, FixedList128Bytes{float2}})"/> 一致。
		///
		/// v0.8.0 关键修正：锚点按 Setting.IncludeMask 过滤后才入网格。
		/// 之前这个网格不分类型地收下所有 Garage/Connection/Area 车道的曲线，而出入口模式的判定是
		/// 「类型 / 集合成员 / 锚点邻域」三选一——勾掉任何一类，邻近的其它类锚点照样命中锚点邻域，
		/// 于是三个勾选框实测完全无效（作者 2026-09-25 反馈）。勾选变化会走
		/// OnUpdate 的 includeMask 分支重排整轮重建，所以这里按当下掩码收集即可。
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
			public ComponentLookup<CarParkingFacility> m_CarFacilityData;

			[ReadOnly]
			public ComponentLookup<ParkingFacility> m_ParkingFacilityData;

			public int m_IncludeMask;

			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			/// <summary>诊断输出槽（见 kGridStatsSlots）。单线程 job，整轮重建时一次性赋值。</summary>
			public NativeArray<int> m_Stats;

			private int m_Dropped;
			private int m_MaxCell;
			private int m_Anchors;
			private int m_Cells;

			public void Execute()
			{
				m_Grid.Clear();
				m_Dropped = 0;
				m_MaxCell = 0;
				m_Anchors = 0;
				m_Cells = 0;
				for (int i = 0; i < m_Entities.Length; i++)
				{
					Entity lane = m_Entities[i];
					int category = LaneCategory(lane, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData,
						m_CarLaneData, m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData,
						m_CarFacilityData, m_ParkingFacilityData);
					if (category == 0 || (m_IncludeMask & category) == 0)
					{
						continue;
					}
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
					m_Stats[0] = m_Dropped;
					m_Stats[1] = m_MaxCell;
					m_Stats[2] = m_Anchors;
					m_Stats[3] = m_Cells;
				}
			}

			/// <summary>FixedList128Bytes 装满 15 个 float2 就到 128 字节上限（留一个槽给长度前缀 → 14），
			/// 所以下面的截断不是可调参数而是容器硬限制。大停车场里 AreaLane 密布，一个 32×32 米格子
			/// 超过 14 个采样点是会发生的，而被丢掉的锚点会让邻近车道判不出「靠出入口太近」→ 漏删。
			/// v0.8.0 先把丢掉的量测出来（心跳日志 Anchor grid capacity 那行），拿到实机数据再决定
			/// 换成不设上限的结构（CSR 或 MultiHashMap），不在没有证据时改判定半径。</summary>
			private void AddAnchor(float3 position)
			{
				int2 key = GridKey(position);
				if (!m_Grid.TryGetValue(key, out FixedList128Bytes<float2> list))
				{
					list = default;
				}
				if (list.Length >= 14)
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
			public ComponentLookup<CarParkingFacility> m_CarFacilityData;

			[ReadOnly]
			public ComponentLookup<ParkingFacility> m_ParkingFacilityData;

			/// <summary>Setting.IncludeMask：只把被勾选类别的车道收进邻域集合。</summary>
			public int m_IncludeMask;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			public NativeHashSet<Entity> m_BuildVehicleSet;

			public void Execute()
			{
				int end = math.min(m_End, m_Candidates.Length);
				for (int i = m_Start; i < end; i++)
				{
					Entity lane = m_Candidates[i];
					bool isAccessType = IsAccessType(lane);
					bool nearAnchor = m_CurveData.TryGetComponent(lane, out Curve curve) && IsNearAnchor(curve.m_Bezier, m_Grid);
					if (isAccessType || nearAnchor)
					{
						m_BuildVehicleSet.Add(lane);
					}
				}
			}

			/// <summary>本候选车道的类别有没有被勾选。与 IsInNoYieldScope 的类型分支同一套判据。</summary>
			private bool IsAccessType(Entity lane)
			{
				int category = LaneCategory(lane, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData,
					m_CarLaneData, m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData,
					m_CarFacilityData, m_ParkingFacilityData);
				return category != 0 && (m_IncludeMask & category) != 0;
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

			[ReadOnly]
			public ComponentLookup<CarParkingFacility> m_CarFacilityData;

			[ReadOnly]
			public ComponentLookup<ParkingFacility> m_ParkingFacilityData;

			public int m_Mode;

			/// <summary>Setting.IncludeMask：三类不避让范围的勾选掩码。</summary>
			public int m_IncludeMask;

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
				int sortKey = 1000000 + unfilteredChunkIndex;
				int slot = (unfilteredChunkIndex % kRepairSlots) * 2;
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

			private void StripLane(Entity lane, int sortKey, int slot)
			{
				if (!IsInNoYieldScope(lane, m_Mode, m_IncludeMask, m_AccessVehicleSet, m_CurveData, m_Grid,
					m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData, m_ParkingLaneData,
					m_OwnerData, m_EdgeData, m_NodeData, m_CarFacilityData, m_ParkingFacilityData))
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

			public void Execute()
			{
				if (m_Set.Count == 0)
				{
					return;
				}
				int sortKey = 0;
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

			[ReadOnly]
			public ComponentLookup<CarParkingFacility> m_CarFacilityData;

			[ReadOnly]
			public ComponentLookup<ParkingFacility> m_ParkingFacilityData;

			public int m_Mode;

			/// <summary>Setting.IncludeMask：三类不避让范围的勾选掩码（出入口模式用；全局模式不看）。</summary>
			public int m_IncludeMask;

			/// <summary>是否连人行横道（PedestrianLaneFlags.Crosswalk）的条目一起删：
			/// 全局模式 = 1（"全部道路"含横道），出入口模式 = 0（保留 v0.5 的横道误伤修复）。</summary>
			public int m_IncludeCrosswalks;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

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
					bool inScope;
					if (m_Mode == Setting.kModeGlobal)
					{
						// 全局模式：chunk 级类型快判（全城车辆车道一律删，不分三类）。
						inScope = IsGlobalLaneByChunk(chunkHasCarLane, chunkHasParking, chunkHasGarage,
							chunkHasConnection, chunkHasArea, connectionArray, i);
					}
					else
					{
						// 出入口模式三重判定：已勾选类别的类型 / 邻域集合成员 / 锚点邻域。
						// 与 BuildAccessSetsJob、StripRebuiltLanesJob 共用 IsInNoYieldScope，防止三份判定再度漂移。
						inScope = IsInNoYieldScope(lane, m_Mode, m_IncludeMask, m_AccessVehicleSet, m_CurveData, m_Grid,
							m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData, m_ParkingLaneData,
							m_OwnerData, m_EdgeData, m_NodeData, m_CarFacilityData, m_ParkingFacilityData);
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
					DynamicBuffer<LaneOverlap> replacement = m_Ecb.SetBuffer<LaneOverlap>(unfilteredChunkIndex, lane);
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
