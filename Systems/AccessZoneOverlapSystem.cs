using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using Colossal.Collections;
using Colossal.Mathematics;
using Game;
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

		private SimulationSystem m_SimulationSystem;
		private EndFrameBarrier m_EndFrameBarrier;
		private EntityQuery m_OverlapQuery;
		private EntityQuery m_AnchorQuery;

		/// <summary>事件驱动重刷查询：匹配刚被 LaneOverlapSystem 重建过 overlap 的车道（带 Updated 标记）。
		/// Updated 是单帧脏标记（CleanUpSystem 帧末移除），LaneOverlapSystem 在 Modification4B 阶段
		/// 重建 overlap 后，本系统在 GameSimulation 阶段仍能看到 Updated → 当帧重删，零延迟。
		/// 稳态下匹配 0 实体，成本 ≈ 0。</summary>
		private EntityQuery m_RefreshQuery;

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

		/// <summary>上次观察到的作用模式/范围开关，变化时强制重建集合。</summary>
		private int m_LastMode = -1;
		private int m_LastIncludeMask = -1;

		private bool m_LoggedFirstRun;
		private ulong m_TotalPasses;

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
			base.OnDestroy();
		}

		protected override void OnUpdate()
		{
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
			int includeMask = (setting.IncludeGarageLanes ? 1 : 0)
				| (setting.IncludeConnectionLanes ? 2 : 0)
				| (setting.IncludeParkingLotLanes ? 4 : 0);
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
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = default,
					m_Mode = setting.Mode,
					m_IncludeGarage = setting.IncludeGarageLanes ? 1 : 0,
					m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
					m_IncludeParkingLots = setting.IncludeParkingLotLanes ? 1 : 0,
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
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_IncludeGarage = setting.IncludeGarageLanes ? 1 : 0,
					m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
					m_IncludeParkingLots = setting.IncludeParkingLotLanes ? 1 : 0,
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
					m_Grid = m_AnchorGrid
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
				m_IncludeGarage = setting.IncludeGarageLanes ? 1 : 0,
				m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
				m_IncludeParkingLots = setting.IncludeParkingLotLanes ? 1 : 0,
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

		/// <summary>曲线是否邻近任何出入口锚点（端点+中点的 3×3 邻域 + 距离校验）。</summary>
		private static bool IsNearAnchor(Bezier4x3 bezier, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
		{
			if (IsNearAnchor(bezier.a, grid) || IsNearAnchor(bezier.d, grid) || IsNearAnchor(0.5f * (bezier.a + bezier.d), grid))
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

		private static bool IsAccessLaneByType(int mode, int includeGarage, int includeConnections, int includeParkingLots,
			bool chunkHasCarLane, bool chunkHasParking, bool chunkHasGarage, bool chunkHasConnection, bool chunkHasArea,
			NativeArray<ConnectionLane> connectionArray, int i, Entity lane, ComponentLookup<Owner> ownerData,
			ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData)
		{
			if (mode == Setting.kModeGlobal)
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
			if (includeGarage != 0 && chunkHasGarage)
			{
				return true;
			}
			if (includeConnections != 0 && chunkHasConnection)
			{
				return true;
			}
			if (includeParkingLots != 0 && chunkHasArea)
			{
				return true;
			}
			if (includeParkingLots != 0 && (chunkHasCarLane || chunkHasParking) && !chunkHasGarage && !chunkHasConnection
				&& IsBuildingOwned(lane, ownerData, edgeData, nodeData))
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

		/// <summary>收集出入口锚点（连接道/车库坡道/区域车道的曲线控制点）到网格。</summary>
		[BurstCompile]
		private struct BuildAnchorGridJob : IJob
		{
			[ReadOnly]
			[DeallocateOnJobCompletion]
			public NativeArray<Entity> m_Entities;

			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			public void Execute()
			{
				m_Grid.Clear();
				for (int i = 0; i < m_Entities.Length; i++)
				{
					if (!m_CurveData.TryGetComponent(m_Entities[i], out Curve curve))
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
			}

			private void AddAnchor(float3 position)
			{
				int2 key = GridKey(position);
				if (!m_Grid.TryGetValue(key, out FixedList128Bytes<float2> list))
				{
					list = default;
				}
				if (list.Length >= 14)
				{
					return;
				}
				list.Add(position.xz);
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

			public int m_IncludeGarage;

			public int m_IncludeConnections;

			public int m_IncludeParkingLots;

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

			private bool IsAccessType(Entity lane)
			{
				if (m_IncludeGarage != 0 && m_GarageLaneData.HasComponent(lane))
				{
					return true;
				}
				if (m_IncludeConnections != 0 && m_ConnectionLaneData.HasComponent(lane))
				{
					return true;
				}
				if (m_IncludeParkingLots != 0)
				{
					if (m_AreaLaneData.HasComponent(lane))
					{
						return true;
					}
					if ((m_CarLaneData.HasComponent(lane) || m_ParkingLaneData.HasComponent(lane))
						&& !m_GarageLaneData.HasComponent(lane) && !m_ConnectionLaneData.HasComponent(lane)
						&& IsBuildingOwned(lane))
					{
						return true;
					}
				}
				return false;
			}

			private bool IsBuildingOwned(Entity lane)
			{
				if (!m_OwnerData.TryGetComponent(lane, out Owner owner) || owner.m_Owner == Entity.Null)
				{
					return false;
				}
				Entity ownerEntity = owner.m_Owner;
				return !m_EdgeData.HasComponent(ownerEntity) && !m_NodeData.HasComponent(ownerEntity);
			}

			private static bool IsNearAnchor(Bezier4x3 bezier, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
			{
				if (AccessZoneOverlapSystem.IsNearAnchor(bezier.a, grid) || AccessZoneOverlapSystem.IsNearAnchor(bezier.d, grid)
					|| AccessZoneOverlapSystem.IsNearAnchor(0.5f * (bezier.a + bezier.d), grid))
				{
					return true;
				}
				return false;
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

			public int m_Mode;

			public int m_IncludeGarage;

			public int m_IncludeConnections;

			public int m_IncludeParkingLots;

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
					bool isAccess;
					if (m_Mode == Setting.kModeGlobal)
					{
						isAccess = IsAccessLaneByType(m_Mode, m_IncludeGarage, m_IncludeConnections, m_IncludeParkingLots,
							chunkHasCarLane, chunkHasParking, chunkHasGarage, chunkHasConnection, chunkHasArea,
							connectionArray, i, lane, m_OwnerData, m_EdgeData, m_NodeData);
					}
					else
					{
						// access 模式三重判定：类型出入口 / 邻域集合成员 / 锚点邻域（捕捉新建道路）。
						isAccess = IsAccessLaneByType(m_Mode, m_IncludeGarage, m_IncludeConnections, m_IncludeParkingLots,
							chunkHasCarLane, chunkHasParking, chunkHasGarage, chunkHasConnection, chunkHasArea,
							connectionArray, i, lane, m_OwnerData, m_EdgeData, m_NodeData)
							|| m_AccessVehicleSet.Contains(lane)
							|| (m_CurveData.TryGetComponent(lane, out Curve curve) && IsNearAnchorLocal(curve.m_Bezier, m_Grid));
					}
					if (!isAccess)
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

			private static bool IsNearAnchorLocal(Bezier4x3 bezier, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
			{
				if (AccessZoneOverlapSystem.IsNearAnchor(bezier.a, grid) || AccessZoneOverlapSystem.IsNearAnchor(bezier.d, grid)
					|| AccessZoneOverlapSystem.IsNearAnchor(0.5f * (bezier.a + bezier.d), grid))
				{
					return true;
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
