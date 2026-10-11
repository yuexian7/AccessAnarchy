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
	///    每 kSetRebuildCheckInterval 步看一眼"车道数变了吗 / 原版闸门开过吗"，最长 kSetRebuildFallback
	///    步兜底一次，且 v0.8.9 起兜底还要求墙钟过 kFallbackMinSeconds=30 秒），
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
	/// 在本周期开过，最迟 kSetRebuildCheckInterval=512 步起一轮；完全没动静时靠 kSetRebuildFallback=8192 步
	/// 兜底，v0.8.9 起还要墙钟过 kFallbackMinSeconds=30 秒才算到点）
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
	/// v0.8.6（玩家评论"it doesn't seem to work for all intersections, even though there is no difference
	/// in the way it's built" + 作者 2026-10-06「仔细排查是否如此并修复」「（建筑/资产门口的）横道要删也可以的，
	/// 不要影响道路上的横道就行」）两件事：
	/// ① **真漏口**：车行连道的判定位漏了 `AllowCargo`。`RouteConnectionType.Cargo` 的连道只打
	///    `Inside|AllowCargo`（<c>RoadConnectionSystem.cs:1354</c>），而两条模式的"是不是车辆车道"都只认
	///    Road/Track/Parking 三位（<see cref="IsGlobalLaneByChunk"/> 与 <see cref="IsInNoYieldScope"/> 的全局分支），
	///    于是货台/装卸口那条连道**两种模式下都不在范围内**——v0.8.5 给出入口分支补的是"没命中这三位就继续
	///    按 GarageLane/AreaLane/建筑自有车道这些组件判"，而连道是否带车行类型组件取决于共用的连道 prefab，
	///    判据不该压在一个本机推不出结论的假设上 ⇒ 这一版把 flags 判据补全，并用下面的 `Overlap coverage`
	///    实机行核对这条连道到底有没有被剥。
	///    现统一走 <see cref="IsVehicleConnection"/>，并**必须**同时排除 `Pedestrian` 位：
	///    室外行人连道带的正是 `Pedestrian|AllowMiddle|AllowCargo`（<c>OutsideConnectionSystem.cs:622</c>），
	///    只认 AllowCargo 会把人行道当车行道删。五种 connection type 共用同一个连道 prefab
	///    （`m_ConnectionPrefabs[0]`），人车之别只在 flags，所以这条判据没有第二条路可走。
	/// ② **横道条目按持有者分类**，不再按模式一刀切：出入口模式下只有当**持有者本身是出入口/建筑内部道路**
	///    （<see cref="IsAccessOrFacilityLane"/>）时才删指向横道的条目，靠"邻域集合/32 米邻近"进范围的那些
	///    普通市政道路与路口车道一律保留横道条目 ⇒ 资产门口那条横道不再卡车，马路上的横道照常避让。
	///    `m_IncludeCrosswalks` 这个"按模式取常量"的字段因此删除，四处剥离点（闸门重删 / 首次全量 /
	///    事件重刷 / 集合批删）与 `AuditAccessSetJob` 全部改调 <see cref="CrosswalkScopeFor"/>，杜绝再各自写死一份。
	///    ⚠ 这一档"算不算设施"用的是比 <see cref="IsAccessOrFacilityLane"/> **更窄**的
	///    <see cref="IsFacilityCrosswalkHolder"/>：连道必须带 `Inside`（全库唯一写入点
	///    `RoadConnectionSystem.cs:1336`），否则图边连接道（`OutsideConnectionSystem.cs:603,611,615`
	///    的 `Start|Outside|Road`）也算设施，会把市政路口的横道避让一起带走。
	///    另一个审计 job `AuditPedestrianOverlapsJob` 仍然恒传 includeCrosswalks=1——它数的是
	///    "原版到底有没有把行人条目写回来"，与我们的删除口径无关，写成按模式判就废了这个基线。
	///
	/// 关于"某些交叉口没生效"的定性（作者要的答案）：全局模式下路口车道的行人条目**本来就在删除范围内**，
	/// 单靠代码推不出"漏了某类路口"的形状，所以这一版把 <see cref="AuditPedestrianOverlapsJob"/> 从 4 个计数
	/// 扩到 10 个（新增"车辆持有者 / 其中归路网（edge|node）所有"两组），并在心跳里加一行
	/// `Overlap coverage`。判读：路口车道残留条目=0 却仍然停车 ⇒ 不是 LaneOverlap 这条通道，而是
	/// 信号灯（<c>CarLaneSpeedIterator.cs:138-149</c> 读 LaneSignalType.Stop）、停车/让行标志位
	/// （CarLaneFlags.Stop/Yield）、前车Blocking与路口不占用（reservation/CheckOverlapSpace）、
	/// 行人**本体**站在本车道上（:1105→:1125 的 LaneObject 通道）、以及车辆压到行人道上时被限到 20 km/h
	/// （:194-196 `math.min(m_PrefabCar.m_MaxSpeed, 5.555556f)`）——这五类都不是"避让行人"的数据，本模组不改。
	/// ⚠ v0.8.7 更正：这一版实机（作者 57 km 图，2026-10-07 14:46-15:14 日志）证明"条目=0"这个读数本身
	/// **不可信**——全局模式只在 `m_CoverageDue`（首次全量后第一次心跳）与"闸门开过"时扫全城，而该局
	/// 闸门全程 0 frame(s)，于是那行 0 只代表切模式后 4 秒的状态；之后 15 分钟里条目回来了也没人再看。
	/// 真正的漏口与修法见下面 v0.8.7 段。
	///
	/// v0.8.7（作者实机两条：①「仅出入口时模组会生效，开启全局后这些设施出入口反倒不生效了」
	/// ②「有些人行横道确实没生效…附近没有出入口，另外两边的人行横道都生效，就这一边没生效」）：
	/// **根因是两种模式的重剥节奏不对等，而不对等的源头是"暂停期间本系统根本不跑"**。取证（1.6.0f1）：
	/// <list type="number">
	/// <item>原版重建挂在 <c>MainLoop</c> 驱动的链上：<c>SystemOrder.cs:60</c> 把 <c>ModificationSystem</c>
	/// 注册在 <c>SystemUpdatePhase.MainLoop</c>，它在 OnUpdate 里连跑 Modification1…ModificationEnd
	/// （<c>Common/ModificationSystem.cs:19-26</c>），<c>LaneOverlapSystem</c> 就在 Modification4B
	/// （<c>SystemOrder.cs:178</c>）；<c>GameManager.UpdateWorld()</c>（<c>SceneFlow/GameManager.cs:2386-2393</c>，
	/// 由 <c>Update()</c> 的 <c>:709</c> 调）每**渲染帧**跑一次 MainLoop，只看 <c>shouldUpdateWorld</c>，
	/// 不看模拟速度。</item>
	/// <item>本系统挂在 <c>GameSimulation</c>，而 <c>SimulationSystem.OnUpdate</c>
	/// （反编译件不在 dump 里，本轮 ilspycmd 现读并存为 <c>research/Game.Simulation.SimulationSystem.cs</c>）
	/// 写的是 <c>if (selectedSpeed == 0f) { num = 0; }</c>（<c>:221-224</c>）…
	/// <c>if (num != 0) { for (i &lt; num) { frameIndex++; … Update(GameSimulation, …) } }</c>（<c>:274-289</c>）
	/// ⇒ **暂停（速度 0）时 GameSimulation 一步都不跑，<c>frameIndex</c> 也不涨**。而 <c>ToolSystem</c> 同样在
	/// MainLoop（<c>SystemOrder.cs:58</c>）：暂停中施工照样给 edge/node/建筑打 <c>Updated</c>，下一渲染帧
	/// Modification4B 照样整表 Clear+重算，帧末 <c>Cleanup</c>（也在渲染帧循环里，
	/// <c>GameManager.PostUpdateWorld:2395-2401</c>）把 <c>Updated</c> 摘掉。</item>
	/// <item>⇒ 这些重建对我们**完全隐形**：闸门 <c>m_RebuildOwnerQuery</c> 只在 GameSimulation 求值。
	/// 作者这一局日志就是铁证：<c>Rebuild gate: 0 frame(s)</c> 从 14:46:24 到 15:14 一次没变（该值一变就会
	/// 再打一行），可出入口模式集合内的残留条目却在 0 ↔ 71/80 ↔ 72/82 ↔ 13/13 ↔ 122/195 之间来回跳
	/// ——条目确实在回来，而探测器一次都没响。</item>
	/// <item>为什么只有全局模式"反过来不生效"：出入口模式每完成一轮整轮重建，都会在交换点批量重剥整个
	/// 邻域集合（<see cref="AdvanceAccessSets"/> 返回 true → 干预点 (2)），所以暂停期丢的条目最迟下一轮
	/// （512 步一查 / 8192 步兜底）就被捡回来；**全局模式只有一次 (1) 全量**（<c>m_InitialGlobalApplyDone</c>
	/// 只在模式切换/重新启用/读档时复位），之后只剩"闸门重删"和"车道 Updated 重刷"两个事件钩子，而这两个
	/// 钩子按上一条根本不会响 ⇒ 一次暂停期的重建 = 永久回到原版。作者看到的"出入口生效、全局反倒不生效"
	/// 与"同一个路口两边生效、某一边不生效"是同一个洞的两种形状：差别不在车道类型，而在**那条车道有没有在
	/// 最后一次全局剥离之后被原版重算过**。</item>
	/// </list>
	/// 修法三条，都不增加稳态开销：
	/// ① <see cref="DetectBlindWindow"/>：用"渲染帧涨得比模拟步多"量出暂停/0 倍速窗口
	/// （<c>UnityEngine.Time.frameCount</c> 对 <c>SimulationSystem.frameIndex</c>，余量
	/// <see cref="kBlindFrameMargin"/> 帧），命中就把 <c>m_GateSeenSinceRebuild</c> 与
	/// <c>m_GlobalSweepSignal</c> 一起置位——出入口模式据此立刻起一轮，全局模式据此起一次重剥。
	/// ② <see cref="ShouldSweepGlobal"/>：全局模式补上与出入口模式同一套调度（信号优先、512 步看一眼
	/// 候选数变没变、8192 步兜底），跑的是同一个 <see cref="StripPedestrianOverlapsJob"/> 全城批删；
	/// 稳态一张没动过的图只有兜底那一次，而且它只读不写（没有行人条目的车道在计数为 0 时直接返回）。
	/// ⚠ **v0.8.9 修正上一条**：作者实机日志（2026-10-07 17:02–17:03，全局模式）里 `Global sweep driver`
	/// 的 sweep 数从 3 涨到 5、间隔只有 8~19 秒，每次还替换 153 条车道的缓冲区
	/// （`re-stripped 153 SetBuffer write(s) / 193 pedestrian entries`），而覆盖读数一直是
	/// `0 of 6745` / `0 of 6739` 之间跳——城市结构没变，判据却把"±几条"和"新铺一条路"同等看待。
	/// 当时我把这个频次归给候选数抖动，**同一天的 v0.8.9 日志否证了归因**：那局
	/// `overlap entities 6751 -> 6751`（Δ 全程 0）、34 个检查点全部 skipped，两次 sweep 之间正好是
	/// 8192 步 ≈ 36 秒 ⇒ 起 sweep 的是兜底，而兜底只用步数量计量，在步率高的那局会被压到 8~19 秒
	/// （见 <see cref="kFallbackMinSeconds"/>）。两条都改：门槛挡住噪声（证据就是这局 34 次 skipped），
	/// 墙钟下限挡住兜底被加速；基线同时改成每个检查点前移，比的是"这一段的变化量"而不是
	/// "与上一次起轮的总量之差"。出入口模式的锚点数判据保持原样，
	/// 所以新建筑的出入口仍然当检查点就生效，没有牺牲 v0.8.5/v0.8.6 的响应性。
	/// ③ 度量补口：全局覆盖率改成**每次重剥之后都扫**（<c>m_CoverageDue</c> 由 (1) 与 sweep 共同置位），
	/// 新增一行 `Global sweep driver` 自证（跑了几次、探测到几次暂停窗口、这一阵子又剥掉几条），并让全城
	/// 审计在真数到残留条目时顺带采 3 条样本（持有者类型位 + 对端 flags + 曲线中点坐标，槽位见
	/// <see cref="kAuditSampleCount"/> 与 <see cref="kAuditSamplePerChunk"/>）——下次再报"某一边不生效"，
	/// 日志能直接说出是哪条车道、指向哪种
	/// 对端，不再靠肉眼猜。⚠ 样本里的"持有者类型位"同时能回答 v0.8.6 那个本机推不出的问题：连道到底
	/// 带不带车行类型组件。
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
	/// 横道仍然只在全局分支参与（当时是 `m_IncludeCrosswalks = globalMode ? 1 : 0`，出入口模式一条不动；
	/// v0.8.6 起这个字段没了，改由 CrosswalkScopeFor 按持有者判，见上方 v0.8.6 ②）。
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
	///
	/// v0.8.10（稳定性/性能四项加固 + 一处覆盖补齐）：
	/// <list type="number">
	/// <item>**恢复不再要求玩家退出重进存档**（见 TriggerVanillaOverlapRebuild 的三档阶梯）。
	/// 起因是 v0.8.9 的恢复只有两条路：①反射置 `LaneOverlapSystem.m_Loaded=true`（便宜，
	/// 但依赖一个**私有字段名**，游戏一更新就可能失灵）；②给**全城** owner + **全部**车道
	/// 打 `Updated`（不依赖私有名字，但那是几十万次 archetype 迁移，与世界加载同量级，
	/// 而五轮重试最多能连来 4 次）。现在插入第三档「定向标记」：剥离 job 在确认要写
	/// `SetBuffer` 时把那条车道 Enqueue 进 <see cref="m_WrittenLanes"/>，恢复时只给
	/// 这些车道的 **owner** 打脏（闸门 `m_UpdatedOwnersQuery=[SubLane,Updated,!Deleted]`
	/// 只要 owner 带 Updated，开销与我们改过的车道数成正比，通常几百条）。
	/// 此外 ①加了自检（<see cref="ProbeLoadedFlag"/>：置位后跨一个**渲染帧**读回，
	/// 原版 `GetLoaded()` 消费掉即复位为 false；读到 true 说明这条路已经失灵，
	/// 后续重试直接跳过）；②封顶（<see cref="kRestoreMaxChecks"/> 轮里最重的全城档只跑一次）；
	/// ③五轮不达标不再放弃，挂上 <see cref="kRestoreBgDelaySteps"/> 步一次、最多
	/// <see cref="kRestoreBgMaxRetries"/> 次的**后台有限重试**（<see cref="TickBackgroundRestore"/>），
	/// 每轮立刻扫一遍判定，所以玩家不用退出游戏也能自愈。</item>
	/// <item>**诊断由两遍全城扫描并成一遍**（<see cref="RunCoverageScan"/>）。出入口模式的
	/// `AuditAccessSetJob`（O(集合) 随机访问，实机 6.5 万条 ⇒ 主线程阻塞 15~40 ms）与全局模式的
	/// `Overlap coverage`（O(全城)，10~30 ms）过去是两个独立动作，同一次心跳付两笔。
	/// 现在由 <see cref="AuditPedestrianOverlapsJob"/> 在同一遍里算两个口径（每条车辆车道
	/// 多一次哈希查找），阻塞减半，且两个数字从此同源、不会再互相矛盾。</item>
	/// <item>**暂停窗口探测器加限速**（<see cref="DetectBlindWindow"/>）：判据是"跳过多少帧"，
	/// 而寻路积压与 FrameRate 偏好也会把每帧步数夹到 0（那本来不是暂停），
	/// 于是在拥堵的大城里会贴着 kGateRebuildMinGap 起整轮重建、占空比约 25%。
	/// 现在命中后还要求距上次起轮/起重剥至少 kSetRebuildCheckInterval（512）步，占空比减半。</item>
	/// <item>**轨道车道纳入全局模式**（作者 2026-10-11 口径：全局＝所有会避让行人的地方都不避让）。
	/// 反编译核实这不是"本来就有"：`TrainLaneSpeedIterator` 走的是与汽车同一条通道
	/// （LaneOverlap → 对端 LaneObject → 带 `Creature` 就 `CheckPedestrian`，
	/// TrainLaneSpeedIterator.cs:466-468/:512），而 `Game.Prefabs.TrackLane.GetArchetypeComponents`
	/// 确实贡献 `LaneObject`+`LaneOverlap` ⇒ 电车/列车在原版里本来就会为行人减速。
	/// 于是把 `TrackLane` 加进 <see cref="m_OverlapQuery"/>/<see cref="m_RefreshQuery"/> 的 Any、
	/// <see cref="IsGlobalLaneByChunk"/> 与 <see cref="IsInNoYieldScope"/> 的全局分支。
	/// **出入口模式不含轨道**：轨道永远不是建筑出入口，也不该因为"离某个出入口 32 米"
	/// 就把一整条铁路干线带进来（那会牵连全城道口），所以
	/// <see cref="BuildAccessSetsJob"/> 里也提前跳过它们，不让它们白跑锚点距离测试。</item>
	/// </list>
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
		/// 模式或范围切换）时重建，剩下的用这个长兜底保证极端漏检也能自愈。⚠ 兜底另有墙钟下限
		/// <see cref="kFallbackMinSeconds"/>：步数量在高速档会被同比压缩，光靠步数定不出"最长多少秒一次"。</summary>
		private const int kSetRebuildFallback = 8192;

		/// <summary>v0.8.9：全局模式"车道数变了就重剥"的**物质性门槛**（绝对条数）。
		/// 出处是作者实机日志（2026-10-07 17:02–17:03，v0.8.7，全局模式）里 `Global sweep driver`
		/// 的 sweep 数从 3 涨到 5、间隔只有 8~19 秒，而覆盖读数一直是
		/// `vehicle lanes 0 of 6745 hold 0` / `0 of 6739 hold 0`——城市结构没变、计数在来回抖几条，
		/// 判据却把"±几条"和"新铺了一条路"同等看待。⚠ 那条日志后来被 v0.8.9 的一局否证了**归因**：
		/// 这局 `overlap entities 6751 -> 6751`（Δ 全程 0）、34 个检查点全跳过，8~19 秒那个频次
		/// 对得上的是 <see cref="kSetRebuildFallback"/> 步兜底（见 <see cref="kFallbackMinSeconds"/>）。
		/// 门槛本身仍然要加：它把"计数噪声"这条路径关掉的证据是这局的 34 次 skipped，
		/// 而真正的新路/新建筑（动辄上千条车道）照常立刻触发；小变化交给兜底。</summary>
		private const int kSweepCountMinDelta = 256;

		/// <summary>v0.8.9：同上门槛的相对部分（百分比）。大城市里 2% 就是几千条，小城市里靠绝对值 256 兜住，
		/// 两个条件任一满足即视为"城市结构变了"。</summary>
		private const int kSweepCountMinPercent = 2;

		/// <summary>v0.8.9：出入口模式里"候选数变了就起一轮"的同类门槛（绝对条数 / 百分比）。
		/// 阈值比全局模式低一档，因为出入口的一轮是摊到 ~63 步里跑的、单轮成本低，
		/// 而锚点数那条判据仍然原样保留（新建筑必然改锚点，不受这里影响）。</summary>
		private const int kRebuildCandidateMinDelta = 64;

		/// <summary>v0.8.9：<see cref="kRebuildCandidateMinDelta"/> 的相对部分。</summary>
		private const int kRebuildCandidateMinPercent = 1;

		/// <summary>v0.8.9：最长兜底**另外**要求的墙钟秒数（步数与墙钟两个条件都到才算到点）。
		/// 兜底原本只用步数量，而"一步"值多少秒随玩家的模拟速度档和帧率变，所以"最长兜底周期"
		/// 在墙上并不是一个固定的量：
		/// ① 作者实机 2026-10-07 23:02–23:19（v0.8.9，全局模式）日志里每 1800 步 ≈ 7.8~8.0 秒
		///   ⇒ ≈225~230 步/秒 ⇒ 8192 步 ≈ 36 秒，两次全城重剥之间正是这个长度（23:04:19 首剥、
		///   兜底二剥落在 pass ≈8193，23:05:18 那次心跳报出 sweep=2），而同一段里
		///   `overlap entities 6751 -> 6751`（Δ=0）、34 个检查点全被跳过 ⇒ **起 sweep 的是兜底，
		///   不是候选数门槛**（我上一轮把 8~19 秒的频次归给"候选数抖动"是错的，这份日志把它否掉了）；
		/// ② 上一局 17:02–17:03（v0.8.7）实测的 sweep 间隔是 8~19 秒，但那版两条触发路径都没有门槛、
		///   日志也已被后续会话覆盖，**无法归因到兜底**；能确定的是：只用步数量时，这个"最长周期"
		///   在墙上的长度完全由步率决定，玩家开高速档就会被同比压缩。
		/// 30 秒夹在中间：①那局的 36 秒照常放行，步率再高也不会把全城重剥压到半分钟以内。
		/// 只夹兜底这一条；闸门、暂停窗口（`m_GlobalSweepSignal`）、锚点持续变化、候选数变够量
		/// 这四条事件路径都不受它影响。</summary>
		private const float kFallbackMinSeconds = 30f;

		/// <summary>v0.8.7：判定"有一段渲染帧本系统根本没跑"的余量（帧）。
		/// 判据量的是**相邻两次我们自己的步进之间**的差值（时间戳每步都复位），所以它等于
		/// "连续多少渲染帧没有产出任何模拟步"。正常推进时这个数是 0 或 1；能连续 8 帧以上不产步的只有：
		/// ①暂停 / 0 倍速（<c>SimulationSystem.cs:221-224</c> 直接把步数置 0）；
		/// ②寻路积压把 <c>num</c> 夹到 0（同文件 <c>:240-243</c> <c>pendingSimulationFrame</c>，
		/// 以及 <c>:246-252</c> 的 FrameRate 偏好夹，最后 <c>:256</c> 的下界是 0 而不是 1）。
		/// ②看着像误报，其实不是：这些帧里 <c>MainLoop</c> 照样在跑、原版照样可能重算 overlap，
		/// 而我们同样一步都没看见 ⇒ 补剥是应该做的。真正要防的是"每帧都扫全城"，那由
		/// <see cref="kGateRebuildMinGap"/> 与"信号在起轮时清零"两层夹住（见 <see cref="ShouldSweepGlobal"/>）。
		/// 取 8 帧（约 0.13 秒）而不是 1 帧，是为了不让高刷屏上的常态 0 步帧把扫描频率顶满。</summary>
		private const int kBlindFrameMargin = 8;

		/// <summary>v0.8.7：全城审计发现"该剥却没剥干净"的车辆车道时，最多打几条样本进日志。
		/// 采样本身是**每个 chunk 一格**（见 <see cref="kAuditSamplePerChunk"/>），主线程按顺序取前
		/// <see cref="kAuditSampleCount"/> 个非空槽——不用原子计数器抢槽，因为抢不到就会写出
		/// "A 车道的持有者配 B 车道的对端"这种混样，而这行是要给作者当坐标飞过去看的。</summary>
		private const int kAuditSampleCount = 3;

		/// <summary>每个 chunk 的样本槽宽度（int）：残留行人条目数（0 = 本 chunk 没有样本）、
		/// 持有者 index/version、对端 index/version、持有者类型位、对端
		/// <c>PedestrianLane.m_Flags</c>、对端 <c>ConnectionLane.m_Flags</c>、持有者曲线中点 x/z（米，取整）。
		/// 数组长度 = chunk 数 × 本值，与 <see cref="kAuditCountersPerChunk"/> 同量级（TempJob，扫完即释）。</summary>
		private const int kAuditSamplePerChunk = 10;

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

		/// <summary>v0.8.10：恢复检查全部不达标后的**后台有限重试**间隔（模拟步，≈1× 速度 24 秒）。
		/// 为什么要它：v0.8.10 之前五轮不达标就只打一行 WARN 然后放弃，世界停在"永久剥离态"，
		/// 玩家唯一的自救办法是退出重进存档。现在把廉价的恢复组合（反射 + 定向标记）挂在一个长周期上
		/// 继续试，玩家不用退出游戏也能自愈。频率刻意压低：每轮只多付一次全城扫描（大城 10~30 ms），
		/// 24 秒一次远低于可感知阈值；试满 <see cref="kRestoreBgMaxRetries"/> 次仍不成功才彻底放弃。</summary>
		private const int kRestoreBgDelaySteps = 5400;

		/// <summary>v0.8.10：后台有限重试的次数上限。超过就彻底放弃并明确告知"重进存档即恢复"。</summary>
		private const int kRestoreBgMaxRetries = 5;

		/// <summary>v0.8.10："我们到底改过哪些车道"清单的**软上限**（条数，不是字节）。
		/// 这个清单只服务恢复时的定向标记，塞爆了也不会算错——恢复判据本来就是"审计扫出来的条目数"，
		/// 漏几条只会让定向标记少覆盖几片，下一轮加重机制照样兜住。设上限只是防止一次暴力的全城重剥
		/// 把队列撑到内存里去（131072 × 8 B ≈ 1 MB）。</summary>
		private const int kWrittenLaneCap = 131072;

		/// <summary>v0.8.10：反射恢复路径的自检三态。
		/// v0.8.10 之前"反射置 m_Loaded"失败与否只能靠猜（拿不到字段时报一行 WARN，字段还在但被游戏
		/// 改名/换类型时连 WARN 都没有）。现在置位后隔**一个渲染帧**读回一次：原版 GetLoaded() 消费掉它会
		/// 把字段复位成 false（LaneOverlapSystem.cs 反编译核实），所以读到 false = 这条路真的走通了，
		/// 读到 true = 没人消费 ⇒ 判定失效，后续重试直接跳过这条路，改走定向标记。</summary>
		private const int kLoadedProbeIdle = 0;
		private const int kLoadedProbeWait = 1;
		private const int kLoadedProbeAlive = 2;
		private const int kLoadedProbeDead = 3;

		/// <summary>范围收窄后等多久再按新范围重删。必须给足整帧边界：v0.7.7 复核确认
		/// GameSimulation 一帧渲染内可连追多步模拟，而 LaneOverlapSystem 在 MainLoop 的
		/// Modification4B，等待太短会出现"我们先重删、原版重建随后把它冲掉"，
		/// 表现就是切了范围没反应。代价是切换后约 1-2 秒才收敛。</summary>
		private const int kReapplyDelayFrames = 120;

		/// <summary>审计状态机：闲置 / 等帧（到点后同步跑完，不跨帧）。</summary>
		private const int kAuditIdle = 0;
		private const int kAuditWaitFrames = 1;

		/// <summary>审计计数每块占几个 int（v0.8.6 从 4 扩到 10，v0.8.10 再扩到 12）。
		/// 0~1 号是"全城任何持有者"的口径，与 v0.7.8 起的基线/恢复判据完全兼容；
		/// 4~9 号是 v0.8.6 为「交叉口到底有没有被剥干净」新增的**按持有者分类**口径：
		/// 车辆持有者（我们该管的）与其中归路网的（edge/node 所有的市政道路与路口车道）各数一遍，
		/// 连"扫到了多少条这样的车道"一起存，不然光看残留数没法判分母。
		/// 10~11 号是 v0.8.10 把 <see cref="AuditAccessSetJob"/> 合并进来后多出来的两格：
		/// 出入口模式下，**属于邻域集合**的车辆车道各自残留多少条（集合判定与残留计数同源）。
		/// 每块写满 12 个 int 而不是用原子计数，避免 chunk 间争用（主线程求和）。</summary>
		private const int kAuditCountersPerChunk = 12;

		private const int kAuditLanes = 0;
		private const int kAuditEntries = 1;
		private const int kAuditCrosswalkLanes = 2;
		private const int kAuditCrosswalkEntries = 3;
		private const int kAuditVehicleLanes = 4;
		private const int kAuditVehicleEntries = 5;
		private const int kAuditRoadLanes = 6;
		private const int kAuditRoadEntries = 7;
		private const int kAuditVehicleScanned = 8;
		private const int kAuditRoadScanned = 9;

		/// <summary>v0.8.10：出入口模式的邻域集合成员在同一遍扫描里的残留读数（与 kAuditVehicleLanes 同批
		/// 算出，只是多一次集合查找）。合进来之前它由 <see cref="AuditAccessSetJob"/> **单独**扫一遍集合
		/// （IJob 逐实体随机读缓冲，作者实机 6.5 万条 ⇒ 主线程阻塞 15~40 ms），而同一时刻全局模式那边也在
		/// 扫全城 ⇒ 一次心跳付两笔全量扫描。现在合成一笔，两个口径同源，也再没有"两边数字互相矛盾"的可能。</summary>
		private const int kAuditSetLanes = 10;
		private const int kAuditSetEntries = 11;

		/// <summary>重删计数的**初始** chunk 槽位数（每块两格：该 chunk 的「SetBuffer 写入次数」
		/// 「删掉的行人条目数」。v0.8.6 复核：同一条车道可能被"反向一跳"和它自己的 owner 两路各取到一次，
		/// 所以第一格是**写入次数**而不是不同车道数——日志文案已照此改写；两次写入内容取自同一份只读快照，
		/// 幂等，不影响正确性）。
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

		/// <summary>v0.8.10：定向恢复（MarkUpdatedTargetedJob）的基址。它写的是 `AddComponent&lt;Updated&gt;`，
		/// 与全量兜底那两批不同框，也刻意不和它们同帧出现（定向是第一选择，全量只在定向也不够时才上），
		/// 但仍然给独立键空间：ECB 的 SetBuffer/AddComponent 回放顺序一旦和预期不符，症状是"该回来的没回来"，
		/// 而这种 bug 在现场是查不出来的。</summary>
		private const int kSortKeyRestoreTargeted = 8000000;

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
		/// ⚠ 它既**不是**"不是出入口"的判据，也不是"不是车行道"的判据：
		/// `RouteConnectionType.Cargo` 的连道只有 Inside|AllowCargo（:1354），三位一个都不带，
		/// 判"是不是车走的"请一律用 <see cref="IsVehicleConnection"/>，不要就地比较这三位。</summary>
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

		/// <summary>v0.8.6 的 Overlap coverage 行：整行文本去重（这行一次要印六个数字，
		/// 逐个数存"上一次的值"要开六个字段，不如直接把上次那行存下来比一次）。
		/// 只有真的跑了全城扫描时才会构造这行，稳态每 ~30 秒最多一次，成本可忽略。</summary>
		private string m_LastCoverageLine;

		/// <summary>v0.8.6：首次全量剥离跑完后，欠读者一次全城覆盖率读数（下次心跳补）。
		/// 之后只有"原版改写过 overlap"才再扫，见 ReportRebuildDiagnostics。</summary>
		private bool m_CoverageDue;

		/// <summary>上一轮整轮重建开始时看到的两批实体数，以及"之后原版重建闸门有没有开过"。
		/// 这是 v0.8.4 的变化探测器：施工/拆建必然改动带 LaneOverlap 的车道数或带曲线的出入口车道数，
		/// 而任何一次原版重算 overlap 都会被 m_RebuildOwnerQuery 抓到。</summary>
		private int m_LastAnchorCount = -1;

		/// <summary>v0.8.9：上一个检查点看到的锚点数（-1 = 无待确认的变化）。锚点数被车位/临时车道
		/// 来回抖动时，这个值让"起轮"必须等同一个新数字连续出现两次（见 <see cref="ShouldRebuildAccessSets"/>）。</summary>
		private int m_AnchorSeenPrev = -1;
		private int m_LastCandidateCount = -1;
		private bool m_GateSeenSinceRebuild;
		private uint m_LastRebuildFrame;

		/// <summary>v0.8.7 暂停/0 倍速探测器用的两个时间戳：上一次 OnUpdate 看到的模拟步号与渲染帧号。
		/// 判据与理由见 <see cref="DetectBlindWindow"/>；<c>m_LastStepRenderFrame &lt; 0</c> = 本局还没跑过。</summary>
		private uint m_LastStepFrameIndex;
		private int m_LastStepRenderFrame = -1;

		/// <summary>本局探测到的"我们没跑而原版可能在改"窗口次数（只用于心跳自证）。</summary>
		private int m_BlindWindows;

		/// <summary>v0.8.7：全局模式的重剥信号——暂停窗口（或候选数变化之外的其它成因）攒下来了，
		/// 下一次 <see cref="ShouldSweepGlobal"/> 就直接放行。起一轮重剥后清零。</summary>
		private bool m_GlobalSweepSignal;

		private uint m_LastSweepFrame;

		/// <summary>v0.8.9：与 <see cref="m_LastSweepFrame"/>/<see cref="m_LastRebuildFrame"/> 配对的**墙钟**基准，
		/// 用于 <see cref="kFallbackMinSeconds"/>（与 m_LastRebuildTime 同一条理由：步数量在高速档会被压缩）。</summary>
		private float m_LastSweepTime;
		private float m_LastRebuildTime;
		private int m_LastSweepCandidates = -1;

		/// <summary>v0.8.9：最近两次 512 步检查点数到的"带 LaneOverlap 的实体数"，只为把门槛读数打进日志。</summary>
		private int m_SweepCountPrev = -1;
		private int m_SweepCountNow = -1;
		private int m_GlobalSweeps;
		private int m_GlobalSweepSkips;

		/// <summary>v0.8.10：本局**真正被我们改写过 LaneOverlap** 的车道实体。
		///
		/// 为什么要有它：恢复（关开关 / 收窄范围）过去只有两条路——反射让原版全量重建（便宜但依赖一个
		/// 私有字段名，游戏一更新就可能失灵），以及给**全城** owner + **全部**车道打 Updated（不依赖任何私有
		/// 名字，但那是几十万次 archetype 迁移，与世界加载同量级，最坏会连来 4 次）。第三条路就是这份清单：
		/// 只给"我们确实动过的那几百条车道"的 owner 打脏，剩下的让原版按自己的增量路径重建。
		/// 成本：每个剥离 job 在确认要写 SetBuffer 时顺手 Enqueue 一次（NativeQueue.ParallelWriter，
		/// 线程安全），稳态一次都不写（没有行人条目的车道在计数为 0 时就返回了）。
		///
		/// 生命周期：**只在出入口模式起新一轮时清空**（那里紧跟一次 WaitForJobs，见
		/// <see cref="ClearWrittenLanes"/> 的并发约束）；全局模式那条全城重剥路径**不清**
		/// （它前面没有 WaitForJobs，硬清就会与刚排出的 job 的 Enqueue 撞车），清单自然累积。
		/// 累积无害：恢复判据是审计扫出来的条目数，多几条记录只是多标几个 owner；真正的上限由
		/// <see cref="kWrittenLaneCap"/> 兜底。换图时由 ResetWorldCaches 清（它开头同样先 WaitForJobs）。
		/// 恢复时排空交给 <see cref="MarkUpdatedTargetedJob"/>。跨帧有效性问题：里面的实体可能已被游戏删掉，
		/// 所以下游 job 必须先判 `m_UpdatedData.HasComponent`/Owner 是否存在（MarkUpdatedTargetedJob 有这道守卫）。
		private NativeQueue<Entity> m_WrittenLanes;

		/// <summary>v0.8.10：反射恢复路径的自检状态（见 kLoadedProbe*）。</summary>
		private int m_LoadedProbe;

		/// <summary>v0.8.10：置位 m_Loaded 时的渲染帧号。
		/// ⚠ 必须等**下一个渲染帧**再读回：原版的 GetLoaded() 在 MainLoop 里每个渲染帧消费一次，
		/// 而置位发生在 GameSimulation 步内——同一个渲染帧里后续还有 1~7 个模拟步，
		/// 那时原版还没跑到，m_Loaded 仍然是 true，早读会误判成"这条路失效"。</summary>
		private int m_LoadedProbeFrame = -1;

		/// <summary>v0.8.10：上次定向恢复标记了多少个目标（只打进日志）。</summary>
		private int m_LastRestoreTargets;

		/// <summary>v0.8.10：五轮恢复检查失败后的后台重试倒计时（模拟步；0 = 没有在排的重试）。</summary>
		private int m_RestoreBgCountdown;

		/// <summary>v0.8.10：已经做过几次后台重试。</summary>
		private int m_RestoreBgRetries;

		/// <summary>全局重剥的诊断计数（每 chunk 两格：SetBuffer 写入次数、删掉的行人条目数），
		/// 与 <see cref="m_RepairCounters"/> 同构但必须分开：闸门重删与全城重剥在同一步里都会排，
		/// 共用数组会把两个成因糊成同一个数（v0.8.6 那条"混合了行人持有者所以证明不了任何事"的教训）。
		/// 只有 (1)/sweep 那一批拿得到它，事件重刷那一批传的是未创建的默认值。</summary>
		private NativeArray<int> m_SweepCounters;
		private int m_SweepCounterChunks;

		private int m_LastLoggedSweeps = -1;
		private int m_LastLoggedBlindWindows = -1;
		private int m_LastLoggedSweepWrites = -1;
		private int m_LastLoggedSweepEntries = -1;

		/// <summary>整轮重建的实际发生次数与被"没变化"挡掉的检查次数（只用于心跳自证）。
		/// 稳态城市里应当看到 rounds 不再增长、skipped 持续增长——这就是 v0.8.4 那条修复的证据。</summary>
		private int m_RebuildRounds;
		private int m_RebuildSkips;

		/// <summary>v0.8.9：上一次出入口审计看到过的轮次号。审计现在只在"有新情况"时跑
		/// （见 <see cref="ReportRebuildDiagnostics"/>），刚起完一轮也必须读一次，否则那轮的成效没有读数。</summary>
		private int m_LastAuditedRounds = -1;
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

		/// <summary>v0.8.10：丢掉"我们改过哪些车道"的旧账。
		/// ⚠ 只能在**没有在飞 job**的时刻调用（唯一调用点在 AdvanceAccessSets 起新一轮处，
		/// 紧跟它自己的 WaitForJobs()，且在任何新 job 排出去之前）——
		/// `NativeQueue.Clear()` 与 `ParallelWriter.Enqueue` 并发执行就是写坏队列内存。
		/// 换图时的清理由 ResetWorldCaches 负责（它开头同样先 WaitForJobs）。</summary>
		private void ClearWrittenLanes()
		{
			if (m_WrittenLanes.IsCreated && m_WrittenLanes.Count > 0)
			{
				m_WrittenLanes.Clear();
			}
		}

		/// <summary>v0.8.10：软上限兜底（每个模拟步一次，O(1)）。
		/// 稳定状态下这份清单每 ~36 秒会被出入口模式的整轮重建清空一次（兜底周期），
		/// 所以正常远到不了上限；这里只防"连绵施工 + 集合一直不起轮"这种极端情况把队列撑大。
		/// 丢掉几条记录无伤大雅：定向标记少覆盖几片，下一轮恢复检查会判成"没恢复"，
		/// 然后按 <see cref="RetryVanillaOverlapRebuild"/> 的阶梯加力——判据始终是审计扫出来的条目数，
		/// 不是这份清单。⚠ Count 在有在飞 job 时读取是安全的（job 只 Enqueue，不改已排的槽）。</summary>
		private void TrimWrittenLanes()
		{
			if (!m_WrittenLanes.IsCreated || m_WrittenLanes.Count <= kWrittenLaneCap)
			{
				return;
			}
			AccessAnarchyMod.log.Warn($"Tracked stripped-lane list exceeded {kWrittenLaneCap} entries - dropping the oldest records (restore still verifies by counting entries, so this only costs one extra recovery attempt).");
			m_WrittenLanes.Clear();
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
			m_SweepCounterChunks = kRepairSlots;
			m_SweepCounters = new NativeArray<int>(m_SweepCounterChunks * 2, Allocator.Persistent, NativeArrayOptions.ClearMemory);
			m_GridStats = new NativeArray<int>(kGridStatsSlots, Allocator.Persistent, NativeArrayOptions.ClearMemory);
			m_WrittenLanes = new NativeQueue<Entity>(Allocator.Persistent);
			m_OverlapQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[1]
				{
					ComponentType.ReadOnly<LaneOverlap>()
				},
				// v0.8.10：加 TrackLane（电车/火车轨道）。原版 TrainLaneSpeedIterator 同样
				// 走 LaneOverlap → LaneObject → Creature 这条通道为行人减速，而
				// Game.Prefabs.TrackLane.GetArchetypeComponents 贡献 LaneObject+LaneOverlap
				// ⇒ 它们和汽车一样"看得见行人"，全局模式必须一并纳入（自行车道本来就走 CarLane，
				// v0.8.4 起就在范围内）。
				Any = new ComponentType[6]
				{
					ComponentType.ReadOnly<CarLane>(),
					ComponentType.ReadOnly<ParkingLane>(),
					ComponentType.ReadOnly<GarageLane>(),
					ComponentType.ReadOnly<ConnectionLane>(),
					ComponentType.ReadOnly<AreaLane>(),
					ComponentType.ReadOnly<TrackLane>()
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
				// v0.8.10：与 m_OverlapQuery 同口径（含 TrackLane，见那里的反编译依据）。
				Any = new ComponentType[6]
				{
					ComponentType.ReadOnly<CarLane>(),
					ComponentType.ReadOnly<ParkingLane>(),
					ComponentType.ReadOnly<GarageLane>(),
					ComponentType.ReadOnly<ConnectionLane>(),
					ComponentType.ReadOnly<AreaLane>(),
					ComponentType.ReadOnly<TrackLane>()
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
			for (int i = 0; i < 2 && m_VehicleSets != null; i++)
			{
				if (m_VehicleSets[i].IsCreated)
				{
					m_VehicleSets[i].Dispose();
				}
			}
			if (m_WrittenLanes.IsCreated)
			{
				m_WrittenLanes.Dispose();
			}
			if (m_Candidates.IsCreated)
			{
				m_Candidates.Dispose();
			}
			if (m_RepairCounters.IsCreated)
			{
				m_RepairCounters.Dispose();
			}
			if (m_SweepCounters.IsCreated)
			{
				m_SweepCounters.Dispose();
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
			if (m_SweepCounters.IsCreated)
			{
				for (int i = 0; i < m_SweepCounters.Length; i++)
				{
					m_SweepCounters[i] = 0;
				}
			}
			if (m_GridStats.IsCreated)
			{
				for (int i = 0; i < m_GridStats.Length; i++)
				{
					m_GridStats[i] = 0;
				}
			}
			// v0.8.10：换图后上一张图的"改过哪些车道"全部作废（index 会被新世界复用，留着只会把
			// 无关实体标成脏）。自检与后台重试状态一起复位，让新图至少各打一行自证。
			if (m_WrittenLanes.IsCreated)
			{
				m_WrittenLanes.Clear();
			}
			m_LoadedProbe = kLoadedProbeIdle;
			m_LoadedProbeFrame = -1;
			m_RestoreBgCountdown = 0;
			m_RestoreBgRetries = 0;
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
			m_LastCoverageLine = null;
			m_CoverageDue = false;
			m_LastAnchorCount = -1;
			m_AnchorSeenPrev = -1;
			m_LastCandidateCount = -1;
			m_GateSeenSinceRebuild = false;
			m_LastRebuildFrame = 0;
			m_LastRebuildTime = 0f;
			m_RebuildRounds = 0;
			m_RebuildSkips = 0;
			m_LastAuditedRounds = -1;
			m_LastLoggedRounds = -1;
			m_LastLoggedSkips = -1;
			// v0.8.7：全局重剥与暂停窗口探测器。探测器的一次性基准（m_LastStepRenderFrame = -1）
			// 必须一起复位，否则换图后的第一步会把"上一张图到最后一次步进之间"的渲染帧差
			// 读成一个假的暂停窗口。计数复位到 -1 是为了让新图至少各写一行自证。
			m_LastStepFrameIndex = 0;
			m_LastStepRenderFrame = -1;
			m_BlindWindows = 0;
			m_GlobalSweepSignal = false;
			m_LastSweepFrame = 0;
			m_LastSweepTime = 0f;
			m_LastSweepCandidates = -1;
			m_GlobalSweeps = 0;
			m_GlobalSweepSkips = 0;
			m_SweepCountPrev = -1;
			m_SweepCountNow = -1;
			m_LastLoggedSweeps = -1;
			m_LastLoggedBlindWindows = -1;
			m_LastLoggedSweepWrites = -1;
			m_LastLoggedSweepEntries = -1;
		}

		protected override void OnUpdate()
		{
			// 每一步开头先把游戏留在 Dependency 上的句柄吸收进自持链：第 2 步起游戏已经把
			// Dependency 清成 default 了，不吸就可能让上一步的 job 从此无人等待（见 m_JobChain 注释）。
			SyncJobChain();
			// v0.8.10：O(1) 的软上限兜底（见 TrimWrittenLanes）。
			TrimWrittenLanes();
			if (!m_KeyReported)
			{
				m_KeyReported = true;
				Setting.ReportKeyBindings("in-game");
			}
			Setting setting = Setting.Instance;
			bool enabled = setting != null && setting.Enabled;
			// v0.8.7：先量"这一段渲染帧里我们到底跑没跑"，再决定这一帧有没有活。
			// 放在 enabled 判断之前，是为了让时间戳在关闭态也跟着走——否则关掉模组再打开，
			// 关闭期间攒下的渲染帧差会被读成一个假的暂停窗口。
			DetectBlindWindow();
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
					// 关闭态不需要补剥（恢复沿会把条目全部还给原版），把信号一起丢掉，
					// 免得"重新启用"的第一步就因为关闭期间的渲染帧差去扫全城。
					m_GlobalSweepSignal = false;
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
				// v0.8.10：后台恢复重试也是关闭态的东西，重新启用就撤掉。
				m_RestoreBgCountdown = 0;
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
			if (!m_BaselineTaken && ScanPedestrianOverlaps(out AuditTotals baseline, out _, collectSamples: false, vehicleSet: m_VehicleSets[m_ActiveSetIndex]))
			{
				m_BaselineEntries = baseline.m_Entries;
				m_BaselineCrosswalk = baseline.m_CrosswalkEntries;
				m_BaselineTaken = true;
				AccessAnarchyMod.log.Info($"Baseline pedestrian overlaps: {baseline.m_Entries} entries ({baseline.m_CrosswalkEntries} crosswalk) on {baseline.m_Lanes} lane(s); vehicle holders {baseline.m_VehicleScanned} ({baseline.m_RoadScanned} street/intersection).");
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
					TriggerVanillaOverlapRebuild("scope narrowed", allowTargetedFallback: false);
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
			//
			// v0.8.7 把这一段从"全局模式只有一次全量"改成"两种模式都有条件重剥"：
			// 原来 `globalFirstPass` 是全局模式唯一的批量剥离机会（m_InitialGlobalApplyDone 只在模式
			// 切换/重新启用/读档时复位），之后只剩闸门与车道 Updated 两个事件钩子——而这两个钩子在
			// 暂停期间根本不响（见类注释 v0.8.7）。现在多一个 globalSweep，判据与出入口模式的
			// ShouldRebuildAccessSets 同一套（信号优先 / 512 步看候选数 / 8192 步兜底）。
			bool needAccessSets = !globalMode;
			bool gateActive = !m_RebuildOwnerQuery.IsEmptyIgnoreFilter;
			bool refreshActive = !m_RefreshQuery.IsEmptyIgnoreFilter;
			bool globalFirstPass = globalMode && !m_InitialGlobalApplyDone;
			bool globalSweep = globalMode && !globalFirstPass && ShouldSweepGlobal(frameIndex);
			bool gateUrgent = m_GateSeenSinceRebuild && frameIndex - m_LastRebuildFrame >= (uint)kGateRebuildMinGap;
			bool roundDue = needAccessSets && (m_BuildInProgress || !m_SetsInitialized || gateUrgent || ShouldRebuildAccessSets(frameIndex));
			if (!roundDue && !gateActive && !refreshActive && !globalFirstPass && !globalSweep)
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
					m_AnchorSeenPrev = -1;
					m_LastCandidateCount = m_OverlapQuery.CalculateEntityCount();
					m_GateSeenSinceRebuild = false;
					m_LastRebuildFrame = frameIndex;
					m_LastRebuildTime = UnityEngine.Time.realtimeSinceStartup;
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
					m_TrackLaneData = GetComponentLookup<TrackLane>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_Ecb = ecb,
					m_RepairCounters = m_RepairCounters,
					m_Written = m_WrittenLanes.AsParallelWriter()
				};
				m_JobChain = rebuildJob.ScheduleParallel(m_RebuildOwnerQuery, m_JobChain);
				scheduled = true;
			}

			// (1) 全局模式全量剥离：世界加载/切模式后的首次全量 + v0.8.7 起的条件重剥（sweep）。
			// 这是全局模式唯一的"批量"路径，v0.8.6 及以前它一辈子只跑一次，所以任何一次
			// 原版重算（尤其暂停期间发生的、我们探测器看不见的那些）都会把结果永久留在原版状态。
			// 现在与出入口模式共用同一套节流：信号（暂停窗口）优先、512 步看候选数变化、8192 步兜底。
			// 成本：稳态一张没动过的图只有兜底那一次；扫描本身只读，没有行人条目的车道在
			// HasPedestrianOverlap 处直接返回，不产生任何 ECB 写入（与 (0) 同一个不退回写战的论证）。
			if (globalMode && (globalFirstPass || globalSweep))
			{
				// 计数数组按 chunk 独占槽位；扩容点内部会先 WaitForJobs()（见 EnsureSweepCapacity）。
				EnsureSweepCapacity(m_OverlapQuery.CalculateChunkCountWithoutFiltering());
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
					m_TrackLaneType = GetComponentTypeHandle<TrackLane>(isReadOnly: true),
					m_OverlapType = GetBufferTypeHandle<LaneOverlap>(isReadOnly: true),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_TrackLaneData = GetComponentLookup<TrackLane>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = default,
					m_Mode = setting.Mode,
					m_StripCounters = m_SweepCounters,
					m_Written = m_WrittenLanes.AsParallelWriter(),
					m_Ecb = ecb
				};
				overlapJob.m_SortKeyBase = kSortKeyInitialPass;
				m_JobChain = overlapJob.ScheduleParallel(m_OverlapQuery, m_JobChain);
				m_InitialGlobalApplyDone = true;
				// 本批删除要到帧末 ECB 回放才落地，所以读数不在这里排，留给下一次心跳
				// （心跳间隔 1800 步，远大于"回放完成"所需的 1 帧）。
				m_CoverageDue = true;
				// v0.8.7：这一轮重剥的账从这里起算（与出入口模式在"起轮"处记时间戳/候选数是同一套做法：
				// 间隔量在**轮次发生**上，不是量在轮次结束上——这里一轮就是一次 job，没有跨步过程）。
				m_GlobalSweeps++;
				m_LastSweepFrame = frameIndex;
				m_LastSweepTime = UnityEngine.Time.realtimeSinceStartup;
				m_LastSweepCandidates = m_OverlapQuery.CalculateEntityCount();
				m_GlobalSweepSignal = false;
				// v0.8.9：闸门标记在这里消费掉（全局模式没有"起轮"这个概念，不清就永远卡着
				// ShouldSweepGlobal 的那条让路判据）。出入口模式在起轮处清的是同一个标记。
				m_GateSeenSinceRebuild = false;
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
					m_GarageLaneData = GetComponentLookup<GarageLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
					m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
					m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
					m_Mode = setting.Mode,
					m_Ecb = ecb,
					m_Written = m_WrittenLanes.AsParallelWriter()
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
					m_TrackLaneType = GetComponentTypeHandle<TrackLane>(isReadOnly: true),
					m_OverlapType = GetBufferTypeHandle<LaneOverlap>(isReadOnly: true),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_CarLaneData = GetComponentLookup<CarLane>(isReadOnly: true),
					m_ParkingLaneData = GetComponentLookup<ParkingLane>(isReadOnly: true),
					m_AreaLaneData = GetComponentLookup<AreaLane>(isReadOnly: true),
					m_TrackLaneData = GetComponentLookup<TrackLane>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_AccessVehicleSet = globalMode ? default : m_VehicleSets[m_ActiveSetIndex],
					m_Mode = setting.Mode,
					m_Ecb = ecb,
					m_Written = m_WrittenLanes.AsParallelWriter()
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

		/// <summary>v0.8.7：量出"这一段渲染帧里本系统一次都没跑"的窗口，并把两种模式的重剥信号一起点上。
		///
		/// 为什么需要它（作者实机两条反馈的根因，取证见类注释 v0.8.7 段）：原版
		/// <c>LaneOverlapSystem</c> 在 <c>MainLoop → ModificationSystem → Modification4B</c>
		/// （<c>SystemOrder.cs:58</c> + <c>:178</c>），每**渲染帧**跑一次；本系统在
		/// <c>GameSimulation</c>，而 <c>SimulationSystem.OnUpdate</c> 在 <c>selectedSpeed == 0f</c> 时
		/// 把步数 <c>num</c> 直接置 0、整段跳过 ⇒ **暂停期间原版照样重建 overlap，我们一步都不跑**。
		/// 更糟的是这条沿上没有任何东西能事后查到：<c>Updated</c> 在同一渲染帧的 <c>Cleanup</c>
		/// （<c>GameManager.PostUpdateWorld</c>，同样每渲染帧跑）就被摘掉了，所以我们恢复运行的第一步
		/// 看到的闸门是空的——这就是那局日志里 <c>Rebuild gate: 0 frame(s)</c> 与"条目明明回来了"
		/// 能同时成立的原因。
		///
		/// 判据只用两个现成的计数器：<c>UnityEngine.Time.frameCount</c>（渲染帧，暂停也涨）与
		/// <c>SimulationSystem.frameIndex</c>（模拟步，只有真跑了才涨）。时间戳每步复位，所以差值量的就是
		/// "连续多少渲染帧没产出任何模拟步"；累积到 <see cref="kBlindFrameMargin"/> 帧即说明中间有一段
		/// "只跑 MainLoop 不跑 GameSimulation"的时间，原版有充分机会改写 overlap。
		/// ⚠ 别把这条读成"只有暂停才会命中"：寻路积压与 FrameRate 偏好也能把每帧步数夹到 0
		/// （<c>research/Game.Simulation.SimulationSystem.cs:240-256</c>），那时原版照样在每帧重建，
		/// 所以命中不是误报，只是频率会高一些——上限由下面的两个节流夹住。
		/// 命中后不自己动数据，只置两个既有信号位：出入口模式走 <c>gateUrgent</c> 起一轮整轮重建
		/// （交换点的集合批删会重剥），全局模式走 <see cref="ShouldSweepGlobal"/> 起一次全城重剥。
		/// 两者都各自带节流（<see cref="kGateRebuildMinGap"/> / 起轮后清信号），不会变成每步扫描。</summary>
		private void DetectBlindWindow()
		{
			uint frameIndex = m_SimulationSystem.frameIndex;
			int renderFrame = UnityEngine.Time.frameCount;
			if (m_LastStepRenderFrame >= 0)
			{
				int skipped = renderFrame - m_LastStepRenderFrame - (int)(frameIndex - m_LastStepFrameIndex);
				// v0.8.10：命中之后再加一道"距上次起轮/起重剥至少隔 kSetRebuildCheckInterval 步"的限速。
				// 原来的判据只看"跳过多少帧"，而寻路积压与 FrameRate 偏好也能把每帧步数夹到 0
				// （SimulationSystem.cs:240-256），那本来不是暂停，却一样会让这里置位 ⇒ 出入口模式会
				// 贴着 kGateRebuildMinGap（256 步）的上限反复起整轮，一轮 ≈63 步 ⇒ 占空比约 25%。
				// 加到 512 步后占空比减半，而真正的暂停窗口恢复后最多晚 ~2.3 秒被接住，玩家感知不到。
				uint floor = math.max(m_LastRebuildFrame, m_LastSweepFrame);
				bool cooled = floor == 0u || frameIndex - floor >= (uint)kSetRebuildCheckInterval;
				if (skipped >= kBlindFrameMargin && cooled)
				{
					m_BlindWindows++;
					m_GateSeenSinceRebuild = true;
					m_GlobalSweepSignal = true;
				}
			}
			m_LastStepFrameIndex = frameIndex;
			m_LastStepRenderFrame = renderFrame;
		}

		/// <summary>v0.8.7：全局模式要不要再起一次全城重剥。判据与 <see cref="ShouldRebuildAccessSets"/>
		/// 逐条对齐，因为两者的病因相同（"原版改写过，我们没跟上"）：
		/// ① 有信号（暂停窗口）就直接放行，但要隔够 <see cref="kGateRebuildMinGap"/> 步，
		/// 免得连续暂停/连续施工把全城扫描变成满负荷；② 每 <see cref="kSetRebuildCheckInterval"/> 步
		/// 看一眼候选车道数（两次 CalculateEntityCount，按 archetype 计数，成本可忽略）；
		/// ③ 完全没动静时 <see cref="kSetRebuildFallback"/> 步兜底一次。
		/// ⚠ 这里**不**把真实闸门（<c>m_GateSeenSinceRebuild</c>）当触发器：闸门开的时候 (0) 那批
		/// 已经按同一批实体重剥过了，再扫全城就是 v0.8.4 压下去的那笔重复开销。全局重剥只补
		/// "闸门看不见的窗口"和"车道数变了"这两种情况。</summary>
		private bool ShouldSweepGlobal(uint frameIndex)
		{
			// 兜底先答：它本身就远大于最小间隔，不需要再夹。⚠ v0.8.9：步数到了还要墙钟也够
			// kFallbackMinSeconds——步数量在高速档会被同比压缩（见常量注释的实机读数）。
			if (frameIndex - m_LastSweepFrame >= (uint)kSetRebuildFallback
				&& UnityEngine.Time.realtimeSinceStartup - m_LastSweepTime >= kFallbackMinSeconds)
			{
				return true;
			}
			// 其余触发一律隔够 kGateRebuildMinGap。这里必须夹：一轮 sweep 是**一次**全城 chunk
			// 迭代（不像出入口模式那样分 63 步摊开），施工期候选数每个 512 步检查点都在变，
			// 不夹间隔就等于每 4 秒满负荷扫一次全城——正是 v0.8.5 被审查代理抓出来的那个回归形状。
			if (frameIndex - m_LastSweepFrame < (uint)kGateRebuildMinGap)
			{
				return false;
			}
			if (m_GlobalSweepSignal)
			{
				return true;
			}
			if (frameIndex % kSetRebuildCheckInterval != 0)
			{
				return false;
			}
			// v0.8.9（独立复核 #4）：本周期原版闸门开过 ⇒ 干预点 (0) 已经按同一批实体当帧重删过了，
			// 再扫全城就是把同一份活干第二遍（正是 v0.8.4 压下去的那笔重复开销；注释一直这么写、
			// 代码没真做）。闸门标记在**起 sweep 时**清（见上面全局剥离那段），所以这里只是"这轮先让路"，
			// 不会把它永久卡死；暂停窗口走的是 m_GlobalSweepSignal，在上面已经答掉，不受这条影响。
			// ⚠ 必须放在检查点判据**之后**：本方法每个模拟步都会被调用，放在前面会让 skipped
			// 这个读数每步 +1，把"每个检查点跳过一次"的含义冲淡掉。
			if (m_GateSeenSinceRebuild)
			{
				m_GlobalSweepSkips++;
				return false;
			}
			// v0.8.9：门槛比的是"这一个检查点间隔内的净变化"，基线每个检查点都前移。旧写法拿
			// "上一次起 sweep 时的数量"当基线，于是 ±几条的车位/临时车道抖动（实机 6745 ↔ 6739）
			// 与"新铺了一条路"完全等价。⚠ 那局 8~19 秒的 sweep 间隔本身不是这条判据造成的
			// （见上面兜底那段：v0.8.9 日志里 Δ=0、34 次 skipped，是 8192 步兜底在高步率下的长度）。
			int now = m_OverlapQuery.CalculateEntityCount();
			m_SweepCountPrev = m_LastSweepCandidates;
			m_SweepCountNow = now;
			m_LastSweepCandidates = now;
			if (m_SweepCountPrev < 0)
			{
				return true;
			}
			int delta = now >= m_SweepCountPrev ? now - m_SweepCountPrev : m_SweepCountPrev - now;
			if (delta >= kSweepCountMinDelta || (m_SweepCountPrev > 0 && delta * 100 >= m_SweepCountPrev * kSweepCountMinPercent))
			{
				return true;
			}
			m_GlobalSweepSkips++;
			return false;
		}

		/// <summary>把全局重剥的诊断计数数组扩到能容纳 <paramref name="chunkCount"/> 个 chunk
		/// （每块两格）。与 <see cref="EnsureRepairCapacity"/> 同构，只在真要扩容时 WaitForJobs()。</summary>
		private void EnsureSweepCapacity(int chunkCount)
		{
			if (chunkCount <= m_SweepCounterChunks)
			{
				return;
			}
			WaitForJobs();
			if (m_SweepCounters.IsCreated)
			{
				m_SweepCounters.Dispose();
			}
			m_SweepCounterChunks = chunkCount + (chunkCount >> 2) + 16;
			m_SweepCounters = new NativeArray<int>(m_SweepCounterChunks * 2, Allocator.Persistent, NativeArrayOptions.ClearMemory);
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
		/// 心跳自证日志（v0.8.0）。用户回报「某个出入口仍会避让」时先看这五行：
		/// ① Rebuild gate —— 原版这一阵子改写过 overlap 没有、我们跟上了多少；
		/// ② Anchor grid —— 锚点容量有没有被卡掉（卡掉就会有邻近车道判不出靠出入口）；
		/// ③ Access set audit —— 集合内的车辆车道此刻还留着几条行人条目；
		/// ④ Overlap coverage（v0.8.6，仅全局模式）—— 全城车辆持有者 / 其中路网与路口持有者
		/// 此刻还留着几条行人条目；
		/// ⑤ Global sweep driver（v0.8.7，仅全局模式）—— 全城重剥跑了几次、其中几次是被
		/// "暂停窗口"探测器叫起来的、这一阵子又叫走了多少条行人条目。
		/// ①非零而③也非零 = 我们没跟全（漏口还在）；①=0 且③④都=0 却仍然避让 = 不是 LaneOverlap 这条通道，
		/// 而是类注释 v0.7.2 与本文件 v0.8.6 段列出的那五类原版行为（信号灯、停车/让行标志位、
		/// 路口被前车占满、行人本体站在车道上、车压在行人道上被限速）。
		/// ⚠ ④紧跟在一次重建之后可能读到短暂的残留值：原版在 Modification4B 重写、我们在同一个模拟步
		/// 排删、帧末才回放，所以判"漏口"要看连续两次数值一致，单次非零不算。
		/// ⚠ v0.8.7 起 ①=0 **不再等于"原版没改写过"**——暂停期间原版照样改写而我们整段不跑，
		/// 那次数的只是"我们醒着时看到的闸门"。要看穿这段窗口读 ⑤ 的 blind window 计数。
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
			// v0.8.7：全局重剥自己的两格计数（闸门重删用的是上面那份，两个成因不能混）。
			int sweepWrites = 0;
			int sweepEntries = 0;
			for (int i = 0; i < m_SweepCounterChunks; i++)
			{
				sweepWrites += m_SweepCounters[i * 2];
				sweepEntries += m_SweepCounters[i * 2 + 1];
				m_SweepCounters[i * 2] = 0;
				m_SweepCounters[i * 2 + 1] = 0;
			}
			// v0.8.6：全城覆盖率扫描的条件要在这里取——下面就把 m_GateFrames 清零了。
			bool hadOverlapActivity = m_GateFrames > 0 || lanes > 0;
			if (m_GateFrames != m_LastGateFrames || lanes != m_LastGateLanes || entries != m_LastGateEntries)
			{
				m_LastGateFrames = m_GateFrames;
				m_LastGateLanes = lanes;
				m_LastGateEntries = entries;
				AccessAnarchyMod.log.Info($"Rebuild gate: {m_GateFrames} frame(s) with vanilla LaneOverlap rebuild this period, last gate {m_GateLastOwners} owner(s); re-stripped {lanes} SetBuffer write(s) / {entries} pedestrian entries (writes, not distinct lanes: the reverse hop can reach one lane from several owners; counter slots: {m_RepairCounterChunks}).");
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
				AccessAnarchyMod.log.Info($"Access-zone rebuild driver: {m_RebuildRounds} round(s) started, {m_RebuildSkips} check(s) skipped (unchanged, jitter below gate, or change not yet seen twice) (check every {kSetRebuildCheckInterval} steps, fallback every {kSetRebuildFallback} steps or {kFallbackMinSeconds:0}s, whichever is later).");
			}

			// v0.8.7 的自证行：全局模式的重剥到底跑没跑、跑的时候有没有活可剥。
			// 这一行是本轮两条反馈的判据：
			// · blind windows > 0 且 re-stripped 条目 > 0 ⇒ "暂停期原版改写、全局模式再也没跟"这个
			//   根因成立，作者看到的"开了全局反而不生效"从此会在 ≤256 步内自愈；
			// · blind windows > 0 而 re-stripped 一直是 0 ⇒ 暂停窗口没造成丢失，得换方向查
			//   （这时看 Overlap coverage 与它新加的 leak sample 行）；
			// · sweeps 在涨而 writes 一直是 0 是正常稳态（重剥只读不写）。
			// 去重：只在四个数真的变了时才写，skipped 不进判据（它每个检查点都 +1，会把这行刷满）。
			if (m_GlobalSweeps != m_LastLoggedSweeps || m_BlindWindows != m_LastLoggedBlindWindows
				|| sweepWrites != m_LastLoggedSweepWrites || sweepEntries != m_LastLoggedSweepEntries)
			{
				m_LastLoggedSweeps = m_GlobalSweeps;
				m_LastLoggedBlindWindows = m_BlindWindows;
				m_LastLoggedSweepWrites = sweepWrites;
				m_LastLoggedSweepEntries = sweepEntries;
				AccessAnarchyMod.log.Info($"Global sweep driver: {m_GlobalSweeps} sweep(s) run ({m_GlobalSweepSkips} check(s) skipped: unchanged or the gate already re-stripped this period), {m_BlindWindows} blind window(s) where the game rebuilt while we were not stepping; this period re-stripped {sweepWrites} SetBuffer write(s) / {sweepEntries} pedestrian entries (overlap entities {m_SweepCountPrev} -> {m_SweepCountNow}, gate {kSweepCountMinDelta} / {kSweepCountMinPercent}%; check every {kSetRebuildCheckInterval} steps, signal gap {kGateRebuildMinGap}, fallback every {kSetRebuildFallback} steps or {kFallbackMinSeconds:0}s, whichever is later).");
			}

			// v0.8.10：出入口与全局两边的"覆盖读数"**合并成一次扫描**。
			// v0.8.9 之前它们是两个独立动作：出入口模式跑 AuditAccessSet（O(集合) 随机访问，
			// 实机 6.5 万条 ⇒ 主线程阻塞 15~40 ms），全局模式跑 ReportOverlapCoverage（O(全城)，
			// 10~30 ms）——同一次心跳里两笔全量扫描只取一行数字。现在由
			// AuditPedestrianOverlapsJob 在同一遍里把两个口径都算出来（多一次集合哈希查找），
			// 阻塞从"两笔"降到"一笔"。
			//
			// 触发条件（沿用 v0.8.7/v0.8.9 那套节奏，只是两个模式现在走同一遍扫描）：
			// · 全局模式：① 刚做过首次全量/全城重剥（m_CoverageDue）**即时**读一次——那是
			//   "重剥到底有没有效"的唯一证据；②③ 这个周期原版真的改写过 / 我们又叫走过条目，
			//   **只在每 4 个心跳一次的采样点上读**（施工期 ②③ 几乎每心跳为真，不设采样点就成常态税）。
			// · 出入口模式：刚起完一轮就读一次，外加同样的 4 心跳慢兜底——探测器和被探测者不能是
			//   同一样东西（v0.8.6 那行假证据的教训，见开发笔记 §⑯-10），完全没动静的图也至少每 ~4×心跳读一次。
			bool slowDue = m_TotalPasses % (kHeartbeatInterval * 4) == 0;
			// 出入口模式：刚起完一轮（轮次号变了）就查一次——那正是"这一轮有没有把范围剥干净"的时刻；
			// 另外保留"每 4 个心跳一次的慢兜底"（探测器和被探测者不能是同一样东西，
			// v0.8.6 那行假证据的教训），否则完全没动静的图就永远不再读数了。
			bool accessDue = !globalMode && m_SetsInitialized
				&& (m_RebuildRounds != m_LastAuditedRounds || slowDue);
			// 全局模式：②③ 两路（这周期原版真的改写过 / 我们又叫走过条目）**只在慢采样点上扫**
			// （v0.8.9 独立复核 #10：施工期 hadOverlapActivity 每心跳为真，不设采样点就成常态税）；
			// ① m_CoverageDue（刚做完首次全量/全城重剥）保持即时，那一次读数是"重剥到底有没有效"的证据。
			bool globalDue = globalMode && (m_CoverageDue || (slowDue && (hadOverlapActivity || sweepWrites > 0)));
			if (accessDue || globalDue)
			{
				m_CoverageDue = false;
				if (!globalMode)
				{
					m_LastAuditedRounds = m_RebuildRounds;
				}
				RunCoverageScan(globalMode);
			}
		}

		/// <summary>
		/// v0.8.10：**一遍**全城扫描给出两个口径的覆盖读数。
		/// 全局模式出 `Overlap coverage (global)` 那行（含残留条目样本）；
		/// 出入口模式出 `Access set audit` 那行（集合内还剩几条）——数字来自同一遍扫描，
		/// 不会再出现"两边读数互相矛盾"。任何一边不成立时另一半照样出，不因为模式互斥而丢证据。
		/// </summary>
		private void RunCoverageScan(bool globalMode)
		{
			NativeHashSet<Entity> set = !globalMode && m_SetsInitialized
				? m_VehicleSets[m_ActiveSetIndex]
				: default(NativeHashSet<Entity>);
			if (!ScanPedestrianOverlaps(out AuditTotals totals, out NativeArray<int> samples, collectSamples: globalMode, vehicleSet: set))
			{
				return;
			}
			if (!globalMode)
			{
				int setCount = set.IsCreated ? set.Count : 0;
				if (setCount != m_LastAuditSet || totals.m_SetLanes != m_LastAuditLanes || totals.m_SetEntries != m_LastAuditEntries)
				{
					m_LastAuditSet = setCount;
					m_LastAuditLanes = totals.m_SetLanes;
					m_LastAuditEntries = totals.m_SetEntries;
					AccessAnarchyMod.log.Info($"Access set audit: {totals.m_SetLanes} of {setCount} vehicle lanes still hold {totals.m_SetEntries} pedestrian overlap entries (0 = clean; crosswalk entries on street lanes are kept by design and are not counted).");
				}
				if (samples.IsCreated)
				{
					samples.Dispose();
				}
				return;
			}
			if (totals.m_VehicleScanned <= 0)
			{
				if (samples.IsCreated)
				{
					samples.Dispose();
				}
				return;
			}
			string line = $"Overlap coverage (global): vehicle lanes {totals.m_VehicleLanes} of {totals.m_VehicleScanned} still hold {totals.m_VehicleEntries} pedestrian entries; street & intersection lanes {totals.m_RoadLanes} of {totals.m_RoadScanned} hold {totals.m_RoadEntries} (crosswalk entries {totals.m_CrosswalkEntries} across all holders).";
			if (line != m_LastCoverageLine)
			{
				m_LastCoverageLine = line;
				AccessAnarchyMod.log.Info(line);
				// 样本跟着覆盖率行一起去重：漏口只要存在就会每个心跳都数到，
				// 不绑在这行的"变了才写"上就会变成每 ~30 秒 3 行 WARN 的新刷屏源
				// （v0.8.4 玩家拿着一屏 WARN 来反馈"模组一直在报警"就是这么来的）。
				if (totals.m_VehicleLanes > 0)
				{
					LogLeakSamples(samples);
				}
			}
			samples.Dispose();
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
		/// v0.8.10：两条之间插入第三条「定向标记」（见 ScheduleMarkUpdatedTargeted），
		/// 并挂上自检与后台有限重试，恢复失败不再要求玩家退出重进存档。
		/// 两条路径之后都排一次审计，把"行人条目到底回来了没有"写进日志。
		/// </summary>
		private void RestoreVanillaOverlaps()
		{
			TriggerVanillaOverlapRebuild("disabled", allowTargetedFallback: true);
			m_AuditChecks = 0;
			m_RestoreBgCountdown = 0;
			m_RestoreBgRetries = 0;
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
		/// v0.8.10 三档阶梯（由轻到重，每一档都让审计来判定是否生效，而不是假设它生效）：
		/// 1) 反射置 m_Loaded=true（便宜，全城一次重算）——代价是依赖一个私有字段名；
		/// 2) 定向标记：只给"我们真的改过"的车道的 owner 打 Updated（不依赖任何私有名字，
		///    开销与我们改过的车道数成正比，通常几百条）；
		/// 3) 全城 owner + 全部车道打 Updated（不依赖私有名字，但是一次全城 archetype 迁移）。
		/// 收窄范围（allowTargetedFallback=false）必须用全城档：收窄要恢复的是"已经不在
		/// 我们清单里的车道"，定向标记覆盖不到，定向漏了就是"切了范围没反应"。
		/// </summary>
		private void TriggerVanillaOverlapRebuild(string reason, bool allowTargetedFallback)
		{
			if (m_LoadedProbe != kLoadedProbeDead && TryForceVanillaOverlapRebuild())
			{
				ArmLoadedProbe();
				AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem {reason}: requested vanilla FULL LaneOverlap rebuild (LaneOverlapSystem.m_Loaded=true, same path as loading a save).");
				return;
			}
			if (allowTargetedFallback && ScheduleMarkUpdatedTargeted())
			{
				AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem {reason}: requested targeted LaneOverlap rebuild (marked {m_LastRestoreTargets} owner(s) of the lanes this mod actually stripped).");
				return;
			}
			AccessAnarchyMod.log.Warn($"AccessZoneOverlapSystem {reason}: targeted rebuild unavailable, falling back to marking ALL network owners + vehicle lanes Updated (a one-time full-city rebuild cost).");
			ScheduleMarkUpdatedRestore();
		}

		/// <summary>
		/// 检查不达标时的加力重试（v0.7.8 起，v0.8.10 重写）。
		/// 阶梯：第 1 次再推一次 m_Loaded；第 2 次起补上定向标记；
		/// 第 3 次才动"全城打脏"这档最重的（且最多一次）；第 4 次回到廉价组合。
		/// 全部轮次都失败后不再放弃：挂一个长周期（<see cref="kRestoreBgDelaySteps"/>）的
		/// 后台有限重试（见 <see cref="TickRestoreAudit"/>），玩家不用退出游戏也能自愈。
		/// </summary>
		private void RetryVanillaOverlapRebuild(int check)
		{
			// 反射路径只有在自检证明它还活着时才值得再试（写入后没人消费=白写）。
			bool forced = m_LoadedProbe != kLoadedProbeDead && TryForceVanillaOverlapRebuild();
			if (forced)
			{
				ArmLoadedProbe();
			}
			bool targeted = false;
			bool fullCity = false;
			// 第 1 轮：反射已推过，这里只补这一下（v0.7.8 的原行为）。
			// 第 2 轮起：反射失效（或本来就拿不到）就补定向标记——它只依赖"Updated 组件"
			// 而不是任何私有字段名，开销与我们改过的车道数成正比。
			if (check >= 2 && !forced)
			{
				targeted = ScheduleMarkUpdatedTargeted();
			}
			// 第 3 轮：两档廉价手段都没顶用，才动"全城打脏"这档最重的，**且封顶一次**——
			// 它与"世界加载"同量级，绝不能反复来。
			if (check == 3 && !forced && !targeted)
			{
				fullCity = true;
				ScheduleMarkUpdatedRestore();
			}
			// 第 4 轮起：回到廉价组合（定向标记），不再重复全城。
			if (check >= 4 && !forced && !targeted)
			{
				targeted = ScheduleMarkUpdatedTargeted();
			}
			AccessAnarchyMod.log.Warn($"Restore retry after check {check}: m_Loaded again={forced}, targeted marking={targeted}, full-city marking={fullCity}; next check in {kRestoreCheckDelays[math.clamp(check, 0, kRestoreCheckDelays.Length - 1)]} frames.");
		}

		/// <summary>
		/// v0.8.10：定向恢复。排空 <see cref="m_WrittenLanes"/>，给"我们真的改写过 LaneOverlap"
		/// 的车道的 **owner**（没有 Owner 的就直接给车道）打 Updated，让原版按自己的增量路径
		/// 重建这些 owner 的 overlap（闸门 m_UpdatedOwnersQuery=[SubLane,Updated,!Deleted]）。
		/// 不依赖任何私有字段名，开销与我们改过的车道数成正比。
		/// 返回 false = 清单为空（没有任何东西需要恢复，原版状态就是我们要的）。
		/// </summary>
		private bool ScheduleMarkUpdatedTargeted()
		{
			// 排空队列必须在没有在飞 job 的时候做：本步前面没有排过任何东西（关闭沿上
			// 调度发生在 OnUpdate 的 disabled 分支，早于任何 job 调度），但上一帧的可能还在飞。
			WaitForJobs();
			if (!m_WrittenLanes.IsCreated || m_WrittenLanes.Count == 0)
			{
				return false;
			}
			// 用 HashSet 去重：同一条车道可能被闸门重删、反向一跳、事件重刷三路各记一次。
			NativeHashSet<Entity> targets = new NativeHashSet<Entity>(m_WrittenLanes.Count, Allocator.TempJob);
			while (m_WrittenLanes.TryDequeue(out Entity lane))
			{
				targets.Add(lane);
			}
			if (targets.Count == 0)
			{
				targets.Dispose();
				return false;
			}
			if (!TryCreateBarrierBuffer(out EntityCommandBuffer.ParallelWriter ecb))
			{
				targets.Dispose();
				return false;
			}
			MarkUpdatedTargetedJob job = new MarkUpdatedTargetedJob
			{
				m_Targets = targets,
				m_UpdatedData = GetComponentLookup<Updated>(isReadOnly: true),
				m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
				m_SortKeyBase = kSortKeyRestoreTargeted,
				m_Ecb = ecb
			};
			m_JobChain = job.Schedule(m_JobChain);
			m_EndFrameBarrier.AddJobHandleForProducer(m_JobChain);
			// 命令已经写进 ECB（回放由 EndFrameBarrier 在帧末负责），这里 Complete 只为
			// 安全释放 targets——与 WaitForJobs 同一套"不能在飞时释放"的约束。
			m_JobChain.Complete();
			m_LastRestoreTargets = targets.Count;
			targets.Dispose();
			return true;
		}

		/// <summary>v0.8.10：置位反射标志并挂上自检（记下置位时的渲染帧，见 m_LoadedProbeFrame）。</summary>
		private void ArmLoadedProbe()
		{
			m_LoadedProbe = kLoadedProbeWait;
			m_LoadedProbeFrame = UnityEngine.Time.frameCount;
		}

		/// <summary>
		/// v0.8.10：反射恢复路径的自检。置位后的下一个**渲染帧**（不是下一个模拟步），原版
		/// GetLoaded() 会把 m_Loaded 复位成 false（反编译核实：`if (m_Loaded) { m_Loaded = false; return true; }`），
		/// 所以读到 false = 这条路真的走通了；读到 true = 没人消费（被游戏改名/换类型），
		/// 判定失效，后续重试跳过这条路，改走定向标记。两种结果都只打一行日志，
		/// 游戏更新后作者能从日志立刻看出来，不用靠玩家反馈。
		/// </summary>
		private void ProbeLoadedFlag()
		{
			if (m_LoadedProbe != kLoadedProbeWait)
			{
				return;
			}
			if (UnityEngine.Time.frameCount <= m_LoadedProbeFrame)
			{
				// 还没跨渲染帧，原版这一帧还没机会消费；下一帧再读。
				return;
			}
			m_LoadedProbe = kLoadedProbeAlive;
			try
			{
				if (s_LoadedField == null || !s_LoadedFieldResolved)
				{
					return;
				}
				LaneOverlapSystem laneOverlapSystem = World.GetExistingSystemManaged<LaneOverlapSystem>();
				if (laneOverlapSystem == null)
				{
					return;
				}
				bool stillLoaded = (bool)s_LoadedField.GetValue(laneOverlapSystem);
				if (stillLoaded)
				{
					m_LoadedProbe = kLoadedProbeDead;
					AccessAnarchyMod.log.Warn("Reflection restore path is not consumed by the game (LaneOverlapSystem.m_Loaded stayed true) - switching to targeted marking for later restore attempts.");
				}
				else
				{
					AccessAnarchyMod.log.Info("Reflection restore path verified: the game consumed the forced full rebuild flag.");
				}
			}
			catch (Exception ex)
			{
				AccessAnarchyMod.log.Warn("Probing LaneOverlapSystem.m_Loaded failed: " + ex);
			}
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
		/// v0.8.10：定向恢复用的标记 job（见 <see cref="ScheduleMarkUpdatedTargeted"/>）。
		/// 清单里的实体可能已经被游戏删掉，所以每条都先做存在性检查：
		/// 优先标它的 owner（LaneOverlapSystem 的重建闸门要的是带 SubLane 的 owner），
		/// 没有 Owner 组件就直接标车道自己（对闸门没用，但对车道 flags 两趟刷新有意义，
		/// 而且绝无害——Updated 会被 CleanUpSystem 在帧末摘掉）。
		/// 已带 Updated 的跳过：ECB 帧末回放时重复 AddComponent 是不能赌的语义。
		/// </summary>
		[BurstCompile]
		private struct MarkUpdatedTargetedJob : IJob
		{
			[ReadOnly]
			public NativeHashSet<Entity> m_Targets;

			[ReadOnly]
			public ComponentLookup<Updated> m_UpdatedData;

			[ReadOnly]
			public ComponentLookup<Owner> m_OwnerData;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kSortKeyRestoreTargeted）。</summary>
			public int m_SortKeyBase;

			public void Execute()
			{
				foreach (Entity lane in m_Targets)
				{
					Entity target = lane;
					if (m_OwnerData.TryGetComponent(lane, out Owner owner) && owner.m_Owner != Entity.Null)
					{
						target = owner.m_Owner;
					}
					if (!m_UpdatedData.HasComponent(target))
					{
						m_Ecb.AddComponent<Updated>(m_SortKeyBase, target);
					}
				}
			}
		}

		/// <summary>全城审计的合计，字段与 <see cref="kAuditCountersPerChunk"/> 的槽位一一对应
		/// （见 <see cref="AuditPedestrianOverlapsJob"/> 的注释）。用结构体而不是十个 out 参数：
		/// v0.8.6 加了按持有者分类的四格之后，out 参数已经比调用方真正读的数多三倍了。
		/// v0.8.10 再加两格：出入口模式邻域集合的残留（与"全城车辆持有者"同一遍扫描得出，
		/// 取代原来单独跑的 <see cref="AuditAccessSetJob"/>）。</summary>
		private struct AuditTotals
		{
			public int m_Lanes;
			public int m_Entries;
			public int m_CrosswalkLanes;
			public int m_CrosswalkEntries;
			public int m_VehicleLanes;
			public int m_VehicleEntries;
			public int m_RoadLanes;
			public int m_RoadEntries;
			public int m_VehicleScanned;
			public int m_RoadScanned;
			public int m_SetLanes;
			public int m_SetEntries;
		}

		/// <summary>
		/// 全城扫一遍车辆车道的 LaneOverlap，统计指向行人道的条目数，并单列其中指向人行横道的部分
		/// 与按持有者分类的那几格。分块并行 + 主线程同步求和（只在基线、恢复检查与"刚有重建发生过"
		/// 的心跳里跑，稳态不跑）。返回 false = 没有匹配到车道（无数据可判）。
		/// v0.8.7 多一个 <paramref name="collectSamples"/>：只有覆盖率那一次需要顺带采残留条目样本，
		/// 采样数组随之交回调用方（**调用方负责 Dispose**；传 false 时返回的是未创建的默认值）。
		/// v0.8.10 多一个 <paramref name="vehicleSet"/>：出入口模式把邻域集合传进来，
		/// 让"集合成员还剩多少行人条目"在这同一遍里算出（原来由 AuditAccessSetJob 单独扫，
		/// 一次心跳付两笔全量扫描）。全局模式传 default（0 成员，集合口径恒为 0）。
		/// 基线与恢复检查两条沿不采——它们要的是总数，多分配一个 Persistent 数组反而带来
		/// "卸载时谁释放它"的新问题（见 <see cref="OnDestroy"/> 的卸载安全约束）。</summary>
		private bool ScanPedestrianOverlaps(out AuditTotals totals, out NativeArray<int> samples, bool collectSamples, NativeHashSet<Entity> vehicleSet)
		{
			totals = default(AuditTotals);
			samples = default(NativeArray<int>);
			WaitForJobs();
			int chunkCount = m_OverlapQuery.CalculateChunkCountWithoutFiltering();
			if (chunkCount <= 0)
			{
				return false;
			}
			NativeArray<int> counters = new NativeArray<int>(chunkCount * kAuditCountersPerChunk, Allocator.TempJob);
			if (collectSamples)
			{
				// 每 chunk 一格、新建即清零（0 = 本块没有样本）。用完由调用方 Dispose。
				samples = new NativeArray<int>(chunkCount * kAuditSamplePerChunk, Allocator.TempJob);
			}
			AuditPedestrianOverlapsJob auditJob = new AuditPedestrianOverlapsJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_OverlapType = GetBufferTypeHandle<LaneOverlap>(isReadOnly: true),
				m_CarLaneType = GetComponentTypeHandle<CarLane>(isReadOnly: true),
				m_ParkingLaneType = GetComponentTypeHandle<ParkingLane>(isReadOnly: true),
				m_GarageLaneType = GetComponentTypeHandle<GarageLane>(isReadOnly: true),
				m_ConnectionLaneType = GetComponentTypeHandle<ConnectionLane>(isReadOnly: true),
				m_AreaLaneType = GetComponentTypeHandle<AreaLane>(isReadOnly: true),
				m_TrackLaneType = GetComponentTypeHandle<TrackLane>(isReadOnly: true),
				m_PedestrianLaneType = GetComponentTypeHandle<PedestrianLane>(isReadOnly: true),
				m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
				m_ConnectionLaneData = GetComponentLookup<ConnectionLane>(isReadOnly: true),
				m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
				m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
				m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
				m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
				m_Set = vehicleSet,
				m_Counters = counters,
				m_Samples = samples
			};
			auditJob.ScheduleParallel(m_OverlapQuery, m_JobChain).Complete();
			for (int i = 0; i < chunkCount; i++)
			{
				int baseIndex = i * kAuditCountersPerChunk;
				totals.m_Lanes += counters[baseIndex + kAuditLanes];
				totals.m_Entries += counters[baseIndex + kAuditEntries];
				totals.m_CrosswalkLanes += counters[baseIndex + kAuditCrosswalkLanes];
				totals.m_CrosswalkEntries += counters[baseIndex + kAuditCrosswalkEntries];
				totals.m_VehicleLanes += counters[baseIndex + kAuditVehicleLanes];
				totals.m_VehicleEntries += counters[baseIndex + kAuditVehicleEntries];
				totals.m_RoadLanes += counters[baseIndex + kAuditRoadLanes];
				totals.m_RoadEntries += counters[baseIndex + kAuditRoadEntries];
				totals.m_VehicleScanned += counters[baseIndex + kAuditVehicleScanned];
				totals.m_RoadScanned += counters[baseIndex + kAuditRoadScanned];
				totals.m_SetLanes += counters[baseIndex + kAuditSetLanes];
				totals.m_SetEntries += counters[baseIndex + kAuditSetEntries];
			}
			counters.Dispose();
			return true;
		}


		/// <summary>v0.8.7：把 <see cref="AuditPedestrianOverlapsJob"/> 采到的残留条目写成人能读的一行。
		/// 样本是每 chunk 一格，所以这里按槽扫过去、最多打 <see cref="kAuditSampleCount"/> 条。
		/// 每条给出：持有者实体号、类型位、归不归路网、曲线中点坐标（作者可以按坐标飞过去看），
		/// 以及对端实体号与它的 PedestrianLane/ConnectionLane flags。
		/// 目的只有一个：把"某一边的人行横道没生效"从肉眼观察变成能定位的数据。</summary>
		private void LogLeakSamples(NativeArray<int> samples)
		{
			if (!samples.IsCreated)
			{
				return;
			}
			int printed = 0;
			for (int chunk = 0; chunk * kAuditSamplePerChunk < samples.Length && printed < kAuditSampleCount; chunk++)
			{
				int b = chunk * kAuditSamplePerChunk;
				int hit = samples[b];
				if (hit <= 0)
				{
					continue;
				}
				printed++;
				int bits = samples[b + 5];
				string holderTypes = (bits & 1) != 0 ? "CarLane" : string.Empty;
				AppendType(ref holderTypes, bits, 2, "ParkingLane");
				AppendType(ref holderTypes, bits, 4, "GarageLane");
				AppendType(ref holderTypes, bits, 8, "ConnectionLane");
				AppendType(ref holderTypes, bits, 16, "AreaLane");
				AppendType(ref holderTypes, bits, 32, "PedestrianLane");
				AppendType(ref holderTypes, bits, 64, "building-owned");
				AppendType(ref holderTypes, bits, 128, "TrackLane");
				string pedFlags = ((PedestrianLaneFlags)samples[b + 6]).ToString();
				string connFlags = ((ConnectionLaneFlags)samples[b + 7]).ToString();
				// 实体号先拼成局部变量再进插值串：`{a}:{b}` 里那个冒号会被编译器当成
				// 格式说明符的起点（CS1003/CS1056），别在插值洞里写裸冒号。
				string holderId = samples[b + 1] + ":" + samples[b + 2];
				string targetId = samples[b + 3] + ":" + samples[b + 4];
				string position = samples[b + 8] + ", " + samples[b + 9];
				AccessAnarchyMod.log.Warn($"Overlap leak sample: lane {holderId} [{holderTypes}] at ({position}) holds {hit} pedestrian entry/entries, first target {targetId} pedFlags={pedFlags} connFlags={connFlags}");
			}
		}

		private static void AppendType(ref string types, int bits, int bit, string name)
		{
			if ((bits & bit) != 0)
			{
				types += types.Length > 0 ? "|" + name : name;
			}
		}

		/// <summary>
		/// 关闭后的恢复检查（v0.7.8 重写）。v0.7.7 的判据是"车辆车道上还有任何行人条目就算恢复"，
		/// 实机被这个判据骗过两次：出入口模式我们从不动横道条目，所以哪怕原版重建完全没落地、
		/// 我们删掉的非横道条目一条都没回来，日志里横道那几万条也照样把判据判成"已恢复"，
		/// 于是不重试、不报警——表现就是"仅出入口模式下只要开启过就永远不避让"。
		/// 新判据看"我们一定删过的那一类"：出入口模式看非横道条目数，全局模式看横道条目数；
		/// 不达标就加力重试（RetryVanillaOverlapRebuild），最多 kRestoreMaxChecks 轮，每轮间隔逐次放大。
		/// v0.8.10：轮次用尽**不再放弃**——挂上后台有限重试（见 RetryRestoreInBackground），
		/// 玩家不用退出游戏；试满 <see cref="kRestoreBgMaxRetries"/> 次才彻底放弃。
		/// </summary>
		private void TickRestoreAudit()
		{
			// v0.8.10：反射路径的自检（置位后的第一个检查点读回 m_Loaded，
			// 判定这条路在本机/本版本上还灵不灵）。
			ProbeLoadedFlag();
			// v0.8.10：后台重试有自己的倒计时，与上面的检查互不干扰
			//（都只在模组处于关闭态才走到这里）。
			TickBackgroundRestore();
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
			if (!ScanPedestrianOverlaps(out AuditTotals scan, out _, collectSamples: false, vehicleSet: m_VehicleSets[m_ActiveSetIndex]))
			{
				AccessAnarchyMod.log.Warn($"Restore check #{m_AuditChecks}: skipped, no vehicle lanes matched.");
				return;
			}
			int lanes = scan.m_Lanes;
			int entries = scan.m_Entries;
			int crosswalkEntries = scan.m_CrosswalkEntries;
			int nonCrosswalk = entries - crosswalkEntries;
			// 主判据：这个模式下"我们一定删过的那一类"条目回来了没有——
			// 出入口模式横道条目从未被动过（v0.7.7 就是被这几万条横道条目糊弄成"已恢复"的），
			// 全局模式横道条目也删过，看横道回来即证明重建落地。
			bool classBack = m_RestoreCheckGlobal ? crosswalkEntries > 0 : nonCrosswalk > 0;
			// 有基线再加一道总量闸：条目数得回到基线的 8 成（留 2 成给期间修路/拆路）。
			bool enough = !m_BaselineTaken || m_BaselineEntries <= 0 || entries >= (m_BaselineEntries * 4) / 5;
			bool restored = classBack && enough;
			AccessAnarchyMod.log.Info($"Restore check #{m_AuditChecks} ({(m_RestoreCheckGlobal ? "global" : "access-zones")}): {lanes} lanes, {entries} pedestrian entries (crosswalk {crosswalkEntries}, non-crosswalk {nonCrosswalk}) vs baseline {m_BaselineEntries} (crosswalk {m_BaselineCrosswalk}); vehicle holders {scan.m_VehicleEntries} entries on {scan.m_VehicleLanes}/{scan.m_VehicleScanned} lane(s), street & intersection {scan.m_RoadEntries} on {scan.m_RoadLanes}/{scan.m_RoadScanned} - {(restored ? "vanilla avoidance restored" : $"NOT restored (classBack={classBack} enough={enough})")}.");
			if (restored)
			{
				return;
			}
			if (m_AuditChecks >= kRestoreMaxChecks)
			{
				AccessAnarchyMod.log.Warn($"Restore failed after {m_AuditChecks} checks: LaneOverlapSystem never rebuilt the {(m_RestoreCheckGlobal ? "crosswalk" : "non-crosswalk")} pedestrian entries.");
				// v0.8.10：不在这里收摊。挂上长周期后台重试（见 RetryRestoreInBackground），
				// 玩家不用退出游戏；试满上限才放弃并明确告知"重进存档即恢复"。
				if (m_RestoreBgRetries < kRestoreBgMaxRetries && m_RestoreBgCountdown == 0)
				{
					m_RestoreBgCountdown = kRestoreBgDelaySteps;
				}
				return;
			}
			RetryVanillaOverlapRebuild(m_AuditChecks);
			QueueRestoreCheck();
		}

		/// <summary>
		/// v0.8.10：恢复检查全部失败后的后台有限重试。
		/// 每 <see cref="kRestoreBgDelaySteps"/> 个模拟步（≈1× 速度 24 秒）跑一次
		/// "反射 + 定向标记"这个廉价组合，然后**立刻**扫一遍判定——重试本身要的就是
		/// 这次判定，不等下一个心跳。代价：一次全城扫描（大城 10~30 ms）每 24 秒最多一次，
		/// 远低于可感知阈值；最多 <see cref="kRestoreBgMaxRetries"/> 次。
		/// </summary>
		private void TickBackgroundRestore()
		{
			if (m_RestoreBgCountdown <= 0)
			{
				return;
			}
			m_RestoreBgCountdown--;
			if (m_RestoreBgCountdown > 0)
			{
				return;
			}
			m_RestoreBgRetries++;
			// 后台重试只用廉价的两档：反射（如果自检还活着）+ 定向标记。
			// 全城打脏那档最重，只留给正式重试链，绝不在后台反复来。
			bool forced = m_LoadedProbe != kLoadedProbeDead && TryForceVanillaOverlapRebuild();
			if (forced)
			{
				ArmLoadedProbe();
			}
			bool targeted = !forced && ScheduleMarkUpdatedTargeted();
			AccessAnarchyMod.log.Warn($"Background restore retry {m_RestoreBgRetries}/{kRestoreBgMaxRetries}: m_Loaded={forced}, targeted marking={targeted}.");
			// 立刻判定这一次有没有效（恢复成功的判据与正式检查相同）。
			if (!ScanPedestrianOverlaps(out AuditTotals scan, out _, collectSamples: false, vehicleSet: m_VehicleSets[m_ActiveSetIndex]))
			{
				return;
			}
			int crosswalkEntries = scan.m_CrosswalkEntries;
			int nonCrosswalk = scan.m_Entries - crosswalkEntries;
			bool classBack = m_RestoreCheckGlobal ? crosswalkEntries > 0 : nonCrosswalk > 0;
			bool enough = !m_BaselineTaken || m_BaselineEntries <= 0 || scan.m_Entries >= (m_BaselineEntries * 4) / 5;
			if (classBack && enough)
			{
				AccessAnarchyMod.log.Info($"Background restore retry {m_RestoreBgRetries} succeeded: vanilla avoidance restored without reloading the save.");
				return;
			}
			if (m_RestoreBgRetries < kRestoreBgMaxRetries)
			{
				m_RestoreBgCountdown = kRestoreBgDelaySteps;
				return;
			}
			AccessAnarchyMod.log.Warn($"Background restore gave up after {m_RestoreBgRetries} retries: pedestrian avoidance is still disabled for this mod's scope. Reloading the save (or re-entering the map) restores it; please send this log.");
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
			// v0.8.9：同 ShouldSweepGlobal，步数之外再要墙钟够 kFallbackMinSeconds。
			if (frameIndex - m_LastRebuildFrame >= (uint)kSetRebuildFallback
				&& UnityEngine.Time.realtimeSinceStartup - m_LastRebuildTime >= kFallbackMinSeconds)
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
				// v0.8.9（独立复核 #3）：这个计数和"车辆集合条数"是同一批实体的读数，会被停车场/临时车道
				// 来回增减——本局 `AccessZoneOverlapSystem heartbeat` 里的 vehicle set 就是
				// 3596 → 3593 → 3587 → 3592（复核报告另外给过锚点数的同类读数 5064 → 5042 → 5057，
				// 那份日志已被覆盖、此处无法复核，只当旁证）。旧写法只要 != 就起轮，于是每个 512 步
				// 检查点都能凑出一轮整城重建。改成**持续变化才算数**：同一个新数值被连续两个检查点看到
				// 才起轮（一次真正的施工会把新数字留在那里，最迟第二个检查点接住；作者这局实测
				// ≈225 步/秒 ⇒ 一个检查点 ≈2.3 秒，代价就是新出入口最多晚 2.3 秒），来回抖的噪声
				// 永远凑不出两个相同的值。本局锚点数是单调增长 5168→5185→5189→5219→5220，
				// 五轮照常起（`Access-zone rebuild driver` 从 1 涨到 8），说明去重没挡住真施工。
				if (anchors == m_AnchorSeenPrev)
				{
					m_AnchorSeenPrev = -1;
					return true;
				}
				m_AnchorSeenPrev = anchors;
			}
			else
			{
				m_AnchorSeenPrev = -1;
			}
			// v0.8.9：候选数只在**变化量够大**时才算"城市变了"。带 LaneOverlap 的实体数在活城里
			// 长期来回抖动（车位/临时车道，实机 6745 ↔ 6739），旧写法每个检查点都判"变了"，
			// 于是整轮重建变成每 512 步一次。锚点数那条判据不动，所以"新建筑带来新出入口"
			// 仍然当检查点触发（新建筑的锚点必变）；这里只挡住纯抖动，小变化靠 8192 步兜底接住。
			int candidates = m_OverlapQuery.CalculateEntityCount();
			int candidatePrev = m_LastCandidateCount;
			m_LastCandidateCount = candidates;
			if (candidatePrev < 0)
			{
				return true;
			}
			int candidateDelta = candidates >= candidatePrev ? candidates - candidatePrev : candidatePrev - candidates;
			if (candidateDelta >= kRebuildCandidateMinDelta || (candidatePrev > 0 && candidateDelta * 100 >= candidatePrev * kRebuildCandidateMinPercent))
			{
				return true;
			}
			m_RebuildSkips++;
			return false;
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
				m_LastRebuildTime = UnityEngine.Time.realtimeSinceStartup;
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

				// v0.8.10：这一轮会把整个出入口区域重新剥一遍，所以"改过哪些车道"从这里重新记账。
				// 位置必须在上面那次 WaitForJobs() **之后**、任何新 job 排出去**之前**——
				// NativeQueue.Clear() 与 ParallelWriter.Enqueue 并发就是写坏队列内存。
				// 全局模式那条全城重剥路径不做清空（它前面没有 WaitForJobs，硬清就有上面这个竞态）：
				// 让清单自然累积即可——恢复判据是审计扫出来的条目数，多几条只是多标几个 owner。
				ClearWrittenLanes();

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
					m_TrackLaneData = GetComponentLookup<TrackLane>(isReadOnly: true),
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
		/// ⚠ 本方法只管"这条车道在不在范围内"，**不决定横道条目删不删**：那一份判据在
		/// <see cref="CrosswalkScopeFor"/>（v0.8.6 起按持有者分类，出入口模式只有资产内部车道的
		/// 横道条目会被删，靠邻近兜底进来的市政道路/路口车道一条都不删——那是 v0.5 横道误伤的修复，
		/// 也是作者 2026-10-01「出入口依然不要影响人行横道等正常道路的避让」与 2026-10-06
		/// 「要删也可以的，不要影响道路上的横道就行」两条要求合起来指向的位置）。
		/// 两处都还要配合"邻近兜底只服务出入口"这一点，别把其中一个改回一刀切。
		/// </summary>
		private static bool IsInNoYieldScope(Entity lane, int mode, NativeHashSet<Entity> vehicleSet,
			ComponentLookup<Curve> curveData, NativeHashMap<int2, FixedList128Bytes<float2>> grid,
			ComponentLookup<PedestrianLane> pedestrianData,
			ComponentLookup<TrackLane> trackData,
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
			// v0.8.10：轨道车道（电车/火车）**只在全局模式**进范围。
			// 依据是反编译：TrainLaneSpeedIterator 走的是与汽车同一条通道
			// （LaneOverlap → 对端 LaneObject → 带 Creature 就 CheckPedestrian，
			//  TrainLaneSpeedIterator.cs:466-468/:512），而 Game.Prefabs.TrackLane 的
			// GetArchetypeComponents 确实贡献 LaneObject + LaneOverlap 缓冲
			// ⇒ 电车/列车在原版里本来就会为行人减速，所以它们必须一并纳入，否则
			//「全部道路（全局）」就名不副实（路面上还有火车在让行人）。
			// 出入口模式**不含**：轨道永远不是建筑出入口，也不该因为"离某个出入口 32 米"
			// 就把一整条铁路干线带进来（那会牵连全城的道口）。
			if (mode != Setting.kModeGlobal)
			{
				if (trackData.HasComponent(lane))
				{
					return false;
				}
			}
			else if (trackData.HasComponent(lane))
			{
				return true;
			}
			if (mode == Setting.kModeGlobal)
			{
				if (carData.HasComponent(lane) || parkingData.HasComponent(lane) || garageData.HasComponent(lane)
					|| areaData.HasComponent(lane))
				{
					return true;
				}
				if (connectionData.TryGetComponent(lane, out ConnectionLane globalConn)
					&& IsVehicleConnection(globalConn))
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
		/// v0.8.6：这条带 <see cref="ConnectionLane"/> 的车道是不是**车走的**。两种模式的范围判定、
		/// chunk 快判、以及审计的"车辆持有者"分类全部走这一处，不要再各自比较 <see cref="kVehicleConnectionFlags"/>。
		/// <list type="bullet">
		/// <item>Road / Track / Parking 任一位 → 车行（原版 VehicleUtils.cs:491-503 同一组位）。</item>
		/// <item>只有 <c>Inside|AllowCargo</c> → 也是车行：`RouteConnectionType.Cargo` 的连道就打这两个位
		/// （RoadConnectionSystem.cs:1354），仓库/商超的货台、装卸口走的就是它。判据与原版同一份：
		/// <c>Pathfind/LaneDataSystem.cs:360-408</c> 给连道定访问限制时就是按
		/// <c>Pedestrian → Road → Parking → AllowCargo → Track</c> 这个顺序判的，其中 AllowCargo 那一支
		/// 打的是 <c>RestrictedCar</c>（:393-401）——原版认它是车行道。⚠ 这一位不能只靠"看车道类型组件"
		/// 兜住：五种 connection type 共用同一个连道 prefab（<c>m_ConnectionPrefabs[0]</c>，:1330 取 archetype，
		/// :1432-1443 创建后只按 <c>Parking</c> 位补 <c>GarageLane</c>），行人连道与车行连道走的是同一条生成路径，
		/// 能区分人车的只有 flags——所以"先看 CarLane/ParkingLane/GarageLane/AreaLane 组件、后看连道位"的
		/// 那套兜底要么接不到它、要么会把行人连道一起接进来。v0.8.5 只补了组件兜底，货台这一类仍然漏。</item>
		/// <item>⚠ 但必须再排除 <c>Pedestrian</c> 位：室外行人连道打的正是
		/// <c>Pedestrian|AllowMiddle|AllowCargo</c>（OutsideConnectionSystem.cs:622），
		/// 只认 AllowCargo 会把一条人行道当车行道处理（删它自己的条目没效果，还会把它放进入口模式的锚点来源池，
		/// 正是 v0.8.4 那次"锚点被行人道灌满"的同一族错误）。</item>
		/// </list>
		/// </summary>
		private static bool IsVehicleConnection(ConnectionLane conn)
		{
			if ((conn.m_Flags & kVehicleConnectionFlags) != 0)
			{
				return true;
			}
			return (conn.m_Flags & ConnectionLaneFlags.AllowCargo) != 0
				&& (conn.m_Flags & ConnectionLaneFlags.Pedestrian) == 0;
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
		/// 判"是不是车走的"用官方同一组位：<see cref="IsVehicleConnection"/>（v0.8.6 起含 AllowCargo），
		/// 两条模式共用同一个判据，不再各写一份。
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
				&& IsVehicleConnection(accessConn))
			{
				return true;
			}
			// v0.8.5：连道但**没有**车行判据，不能就此判定"不是出入口"。
			// v0.8.6 起 `IsVehicleConnection` 已经把 `Inside|AllowCargo` 那一类接回来了，
			// 这里剩下的兜底服务的是"带车库/区域形状位或归建筑自有的车辆车道"：
			// 连道之外还带 ①③④ 任一的仍由后面几条接住。
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

		/// <summary>全局模式的 chunk 级快速类型判定：任何车辆车道（CarLane/ParkingLane/GarageLane/AreaLane/
		/// TrackLane（电车/火车），以及 <see cref="IsVehicleConnection"/> 认可的建筑连道）一律在范围内。
		/// 出入口模式不走这里——那份判定要逐条车道看组件、邻域集合与锚点距离，
		/// 统一在 <see cref="IsInNoYieldScope"/> 里（需要逐车道的 ComponentLookup，没法用 chunk flag 代替）。
		/// ⚠ 这里与 <see cref="IsInNoYieldScope"/> 的全局分支必须给出同一个答案：那条分支先看的也是这几位组件，
		/// 差别只在本函数按 chunk 的组件存在性判（同一 archetype 内所有实体一致），所以连道位必须逐条读
		/// （<paramref name="connectionArray"/>[i]），不能只看"这个 chunk 有没有 ConnectionLane"。</summary>
		private static bool IsGlobalLaneByChunk(bool chunkHasCarLane, bool chunkHasParking, bool chunkHasGarage,
			bool chunkHasConnection, bool chunkHasArea, bool chunkHasTrack, NativeArray<ConnectionLane> connectionArray, int i)
		{
			if (chunkHasCarLane || chunkHasParking || chunkHasGarage || chunkHasArea || chunkHasTrack)
			{
				return true;
			}
			if (chunkHasConnection && IsVehicleConnection(connectionArray[i]))
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
		/// includeCrosswalks == 0 时人行横道（Crosswalk flag）算"不是行人道"，即条目保留；
		/// == 1 时横道条目一起删。<b>这个 0/1 从 v0.8.6 起由 <see cref="CrosswalkScopeFor"/> 按持有者算，
		/// 不要再由调用方按模式写死。</b></summary>
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

		/// <summary>
		/// v0.8.6：指向人行横道的条目删不删，改由**持有者**这条车道决定，不再由模式一刀切。
		/// 全局模式一律删（v0.7.7 口径不变）；出入口模式只有持有者自己是"资产内部的车行道"
		/// （<see cref="IsFacilityCrosswalkHolder"/>）时才删横道条目，靠"邻域集合成员 / 曲线 32 米内有锚点"
		/// 进范围的普通市政道路与路口车道一律保留——作者 2026-10-06：
		/// 「（建筑内部道路或资产门口的横道）要删也可以的，不要影响道路上的横道就行」。
		/// v0.5 那次横道误伤的根因是"邻近兜底跨类"（出入口旁边的马路横道会被 32 米判进范围），
		/// 当时唯一的办法就是一刀切不删横道；现在按持有者分类就能两头都要到：
		/// 挂在车库道/场区道/建筑连道上的那条门口横道删掉，挂在市政 CarLane 上的路口横道不动。
		/// ⚠ 按**每条车道**算一次；同一条车道的"有没有行人条目"与"回写保留项"必须用同一个值，
		/// 否则会出现计数说有、回写时又判成保留的空写（ECB 白占一格命令缓冲）。
		/// </summary>
		private static int CrosswalkScopeFor(Entity holder, int mode,
			ComponentLookup<PedestrianLane> pedestrianData, ComponentLookup<GarageLane> garageData,
			ComponentLookup<ConnectionLane> connectionData, ComponentLookup<AreaLane> areaData,
			ComponentLookup<CarLane> carData, ComponentLookup<ParkingLane> parkingData,
			ComponentLookup<Owner> ownerData, ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData)
		{
			if (mode == Setting.kModeGlobal)
			{
				return 1;
			}
			return IsFacilityCrosswalkHolder(holder, pedestrianData, garageData, connectionData, areaData, carData,
				parkingData, ownerData, edgeData, nodeData) ? 1 : 0;
		}

		/// <summary>
		/// v0.8.6：**比 <see cref="IsAccessOrFacilityLane"/> 更窄**的一档判据，只用来决定"这条车道上的
		/// 横道条目能不能删"。差别在连道那一支：必须带 `Inside`。
		/// 全库 `ConnectionLaneFlags.Inside` 只有一个写入点——`RoadConnectionSystem.cs:1336`
		/// （建筑连道的公共前缀，之后才按 connection type 叠加 Road/Track/Parking/AllowCargo/Pedestrian），
		/// 所以"带 Inside 的连道"＝"某栋建筑/某个设施自己那条大门道"，可以精确等同于玩家理解的"资产门口"。
		/// 不带 Inside 的连道都是**路网的一部分**，最典型是图边连接道
		/// （`OutsideConnectionSystem.cs:603,611,615` 的 `Start|Outside|Road|AllowMiddle`）：
		/// 它们同样命中 `IsVehicleConnection` 的第一支而进入删除范围（v0.8.4 起一直如此，是对的——
		/// 那些道上确实会有货车穿行），但它们是市政道路，**不能让它们把路口横道的避让一起带走**。
		/// 另外 `LaneConnectionSystem.cs:718-724/773-778` 说明 `Pedestrian` 与 `Road` 会被同时打上
		/// （自行车连道），所以这里也不能靠"命中车行位就一定是设施"来推。
		/// 其余三支与 IsAccessOrFacilityLane 相同：GarageLane、AreaLane、建筑自有的 CarLane/ParkingLane。
		/// </summary>
		private static bool IsFacilityCrosswalkHolder(Entity lane,
			ComponentLookup<PedestrianLane> pedestrianData, ComponentLookup<GarageLane> garageData,
			ComponentLookup<ConnectionLane> connectionData, ComponentLookup<AreaLane> areaData,
			ComponentLookup<CarLane> carData, ComponentLookup<ParkingLane> parkingData,
			ComponentLookup<Owner> ownerData, ComponentLookup<Edge> edgeData, ComponentLookup<Node> nodeData)
		{
			if (pedestrianData.HasComponent(lane))
			{
				return false;
			}
			if (garageData.HasComponent(lane) || areaData.HasComponent(lane))
			{
				return true;
			}
			if (connectionData.TryGetComponent(lane, out ConnectionLane conn)
				&& (conn.m_Flags & ConnectionLaneFlags.Inside) != 0 && IsVehicleConnection(conn))
			{
				return true;
			}
			return (carData.HasComponent(lane) || parkingData.HasComponent(lane))
				&& IsBuildingOwned(lane, ownerData, edgeData, nodeData);
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

			/// <summary>v0.8.10：轨道车道（电车/火车）。出入口模式一律跳过，见 Execute。</summary>
			[ReadOnly]
			public ComponentLookup<TrackLane> m_TrackLaneData;

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
					// v0.8.10：轨道车道（电车/火车）同样直接跳过。m_Candidates 来自 m_OverlapQuery，
					// v0.8.10 起它包含 TrackLane；但出入口模式**不含**轨道（见 IsInNoYieldScope），
					// 不在这里跳过就得让每条轨道车道白跑一次 5 采样 × 9 邻格的锚点距离测试。
					if (m_TrackLaneData.HasComponent(lane))
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

			/// <summary>v0.8.10：轨道车道（电车/火车）。只在全局模式进范围，出入口模式一律排除
			/// （理由见 <see cref="IsInNoYieldScope"/> 的注释）。</summary>
			[ReadOnly]
			public ComponentLookup<TrackLane> m_TrackLaneData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			[ReadOnly]
			public NativeHashSet<Entity> m_AccessVehicleSet;

			// v0.8.6：这里原来有 `m_IncludeCrosswalks`（按模式写死的 0/1），已删——
			// 横道条目改由 StripTarget 逐条车道调 CrosswalkScopeFor 决定，m_Mode 就是它的输入。
			public int m_Mode;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			[NativeDisableParallelForRestriction]
			public NativeArray<int> m_RepairCounters;

			/// <summary>v0.8.10：本批真正改写过的车道写进这里（见 m_WrittenLanes）。
			/// NativeQueue.ParallelWriter 是线程安全的，Burst 可用。</summary>
			public NativeQueue<Entity>.ParallelWriter m_Written;

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
					m_PedestrianLaneData, m_TrackLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData, m_ParkingLaneData,
					m_OwnerData, m_EdgeData, m_NodeData))
				{
					return;
				}
				if (!m_Overlaps.TryGetBuffer(lane, out DynamicBuffer<LaneOverlap> buffer) || buffer.Length == 0)
				{
					return;
				}
				// v0.8.6：横道条目按持有者分类（出入口/内部道删、邻域兜底进来的市政道路不删），
				// 一次算出来供"计数"和"回写"两段共用，别在两处各调一次。
				int includeCrosswalks = CrosswalkScopeFor(lane, m_Mode,
					m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData,
					m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData);
				int pedestrian = 0;
				for (int j = 0; j < buffer.Length; j++)
				{
					if (IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, includeCrosswalks))
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
					if (!IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, includeCrosswalks))
					{
						replacement.Add(buffer[j]);
					}
				}
				m_RepairCounters[slot]++;
				m_RepairCounters[slot + 1] += pedestrian;
				// v0.8.10：记下"这条车道是我们改的"，恢复时只标它的 owner（见 m_WrittenLanes）。
				m_Written.Enqueue(lane);
			}
		}

		/// <summary>access 模式干预点 A：只遍历邻域车辆集合，删除指向行人道的 overlap 条目。
		/// 不再 ScheduleParallel 全城 m_OverlapQuery——那是 v0.7.0 性能重构后残留的 O(全城) 热点。
		/// v0.8.6：横道条目不再一律保留——集合里既有设施车道也有靠邻近兜底进来的市政道路，
		/// 按持有者逐条判（<see cref="CrosswalkScopeFor"/>），与另外三处剥离点同一口径。</summary>
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

			/// <summary>本 job 只在出入口模式下排（调用方按 !globalMode 判断），值恒为 Setting.kModeAccessOnly；
			/// 仍然显式传进来给 CrosswalkScopeFor 用，免得将来复用到全局路径时悄悄按"设施"过滤。</summary>
			public int m_Mode;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kSortKeyFromSet）。</summary>
			public int m_SortKeyBase;

			/// <summary>v0.8.10：本批真正改写过的车道（见 m_WrittenLanes）。</summary>
			public NativeQueue<Entity>.ParallelWriter m_Written;

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
					// 同一条车道的"判有没有"与"回写"共用这一个值。
					int includeCrosswalks = CrosswalkScopeFor(lane, m_Mode,
						m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData,
						m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData);
					if (!HasPedestrianOverlap(buffer, includeCrosswalks))
					{
						continue;
					}
					DynamicBuffer<LaneOverlap> replacement = m_Ecb.SetBuffer<LaneOverlap>(sortKey++, lane);
					for (int j = 0; j < buffer.Length; j++)
					{
						if (!IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, includeCrosswalks))
						{
							replacement.Add(buffer[j]);
						}
					}
					// v0.8.10：记下"这条车道是我们改的"（见 m_WrittenLanes）。
					m_Written.Enqueue(lane);
				}
			}

			private bool HasPedestrianOverlap(DynamicBuffer<LaneOverlap> buffer, int includeCrosswalks)
			{
				for (int i = 0; i < buffer.Length; i++)
				{
					if (IsPedestrianLaneEntity(buffer[i].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, includeCrosswalks))
					{
						return true;
					}
				}
				return false;
			}
		}

		/// <summary>
		/// 干预点 A：删除车辆车道 LaneOverlap 缓冲中指向行人道的条目。
		/// 全局模式：所有车辆车道都删（含指向人行横道的条目）。
		/// access 模式：类型判定 / 邻域集合成员 / 锚点邻域三选一进范围；横道条目只在
		/// **持有者自己是出入口/建筑内部道路**时删（v0.8.6，见 <see cref="CrosswalkScopeFor"/>），
		/// 靠邻域兜底进来的普通市政道路与路口车道的横道条目一律保留。
		/// 既用于世界加载/切模式后的首次全量与 v0.8.7 的条件重剥（两者都跑 m_OverlapQuery 全城），
		/// 也用于每帧事件驱动重刷（m_RefreshQuery）。
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

			/// <summary>v0.8.10：轨道车道（电车/火车）的 chunk 级类型位。
			/// 与 <see cref="IsGlobalLaneByChunk"/> 的新增参数配对——全局模式按 chunk 一次判完，
			/// 免得对每条轨道车道再逐条查组件。</summary>
			[ReadOnly]
			public ComponentTypeHandle<TrackLane> m_TrackLaneType;

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

			/// <summary>v0.8.10：轨道车道（电车/火车）。只在全局模式进范围，出入口模式一律排除
			/// （理由见 <see cref="IsInNoYieldScope"/> 的注释）。</summary>
			[ReadOnly]
			public ComponentLookup<TrackLane> m_TrackLaneData;

			public int m_Mode;

			// v0.8.6：原来的 `m_IncludeCrosswalks`（全局 1 / 出入口 0）已删——横道条目改由
			// Execute 逐条车道调 CrosswalkScopeFor，出入口模式下只有设施车道的横道条目会被删。

			/// <summary>v0.8.7：全局重剥（首次全量与 sweep）专用诊断计数，每 chunk 两格
			/// （SetBuffer 写入次数 / 删掉的行人条目数）。事件驱动重刷那一批不传（未创建）——
			/// 那一批的量已经由 <see cref="StripRebuiltLanesJob.m_RepairCounters"/> 报在
			/// `Rebuild gate` 行里，两个成因混在一个数上就没法判分了。</summary>
			[NativeDisableParallelForRestriction]
			public NativeArray<int> m_StripCounters;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			/// <summary>本批 ECB 命令的排序键基址（见 kGateSortKeyBase / kSortKeyRefresh 等常量的注释）：
			/// 同一条车道一帧内可能被多批命令整表覆盖，基址互不重叠才能保证"后跑的、快照更新的"赢。</summary>
			public int m_SortKeyBase;

			/// <summary>v0.8.10：本批真正改写过的车道（见 m_WrittenLanes）。
			/// 首次全量/全城重剥与每帧事件重刷两批都填同一个队列（它们都是"我们动过的车道"）。</summary>
			public NativeQueue<Entity>.ParallelWriter m_Written;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				bool chunkHasCarLane = chunk.Has(ref m_CarLaneType);
				bool chunkHasParking = chunk.Has(ref m_ParkingLaneType);
				bool chunkHasGarage = chunk.Has(ref m_GarageLaneType);
				bool chunkHasConnection = chunk.Has(ref m_ConnectionLaneType);
				bool chunkHasArea = chunk.Has(ref m_AreaLaneType);
				bool chunkHasTrack = chunk.Has(ref m_TrackLaneType);
				if (!chunkHasCarLane && !chunkHasParking && !chunkHasGarage && !chunkHasConnection && !chunkHasArea && !chunkHasTrack)
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
						// 全局模式：chunk 级类型快判（全城车辆车道一律删，不看邻域；v0.8.10 起轨道车道也算）。
						inScope = IsGlobalLaneByChunk(chunkHasCarLane, chunkHasParking, chunkHasGarage,
							chunkHasConnection, chunkHasArea, chunkHasTrack, connectionArray, i);
					}
					else
					{
						// 出入口模式三重判定：出入口/内部道路类型 / 邻域集合成员 / 锚点邻域。
						// 与 BuildAccessSetsJob、StripRebuiltLanesJob 共用 IsInNoYieldScope，防止三份判定再度漂移。
						inScope = IsInNoYieldScope(lane, m_Mode, m_AccessVehicleSet, m_CurveData, m_Grid,
							m_PedestrianLaneData, m_TrackLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData, m_ParkingLaneData,
							m_OwnerData, m_EdgeData, m_NodeData);
					}
					if (!inScope)
					{
						continue;
					}
					DynamicBuffer<LaneOverlap> buffer = overlaps[i];
					// v0.8.6：横道条目按持有者分类，同一条车道的"判有没有"与"回写"共用这一个值。
					int includeCrosswalks = CrosswalkScopeFor(lane, m_Mode,
						m_PedestrianLaneData, m_GarageLaneData, m_ConnectionLaneData, m_AreaLaneData, m_CarLaneData,
						m_ParkingLaneData, m_OwnerData, m_EdgeData, m_NodeData);
					if (!HasPedestrianOverlap(buffer, includeCrosswalks))
					{
						continue;
					}
					DynamicBuffer<LaneOverlap> replacement = m_Ecb.SetBuffer<LaneOverlap>(m_SortKeyBase + unfilteredChunkIndex, lane);
					int kept = 0;
					for (int j = 0; j < buffer.Length; j++)
					{
						if (!IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, includeCrosswalks))
						{
							replacement.Add(buffer[j]);
							kept++;
						}
					}
					// v0.8.7：只有全城重剥那一批传了计数数组（事件重刷传的是未创建默认值）。
					// 槽位与 StripRebuiltLanesJob 同构：unfilteredChunkIndex 独占两格，无跨线程争用。
					if (m_StripCounters.IsCreated)
					{
						int slot = unfilteredChunkIndex * 2;
						m_StripCounters[slot]++;
						m_StripCounters[slot + 1] += buffer.Length - kept;
					}
					// v0.8.10：记下"这条车道是我们改的"（见 m_WrittenLanes）。
					m_Written.Enqueue(lane);
				}
			}

			private bool HasPedestrianOverlap(DynamicBuffer<LaneOverlap> buffer, int includeCrosswalks)
			{
				for (int i = 0; i < buffer.Length; i++)
				{
					if (IsPedestrianLaneEntity(buffer[i].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, includeCrosswalks))
					{
						return true;
					}
				}
				return false;
			}
		}

		/// <summary>
		/// 全城扫一遍车辆车道的 LaneOverlap，统计指向行人道的条目数，并按持有者分类（v0.8.6）。
		/// 前两格沿用 v0.7.7 起的"原版重建有没有发生过"口径——审计要的是事实而不是我们删了什么，
		/// 所以这里横道条目一律计入（includeCrosswalks=1）：全局模式连横道都删，只数非横道会误判。
		/// 新增的 4~9 号把同一批条目按**持有者是谁**再切一刀：
		/// "车辆持有者"= 全局模式会剥的那批（<see cref="IsGlobalLaneByChunk"/> 且不带 PedestrianLane），
		/// "路网持有者"= 其中 Owner 是 edge/node 的，也就是市政道路与**路口**车道。
		/// 玩家评论"it doesn't seem to work for all intersections"要的读数就在这里：
		/// roadEntries == 0 而车仍然停在路口 ⇒ 不是 LaneOverlap 这条通道，见 ReportRebuildDiagnostics。
		/// 每块写 kAuditCountersPerChunk 个 int 避免原子争用，主线程求和。
		/// v0.8.7 再加 <see cref="m_Samples"/>：真数到"该剥却没剥干净"的车辆车道时，就地采最多
		/// <see cref="kAuditSampleCount"/> 条样本（持有者与对端的实体号、双方类型位/flags、持有者曲线中点
		/// 坐标）。作者报"就这一边的人行横道没生效"时，光有计数定位不到是哪条车道，下一轮实机还得靠肉眼再猜；
		/// 有了这三条样本，日志能直接说出漏的是哪一类持有者、指向哪一种对端。
		/// </summary>
		[BurstCompile]
		private struct AuditPedestrianOverlapsJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public BufferTypeHandle<LaneOverlap> m_OverlapType;

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

			/// <summary>v0.8.10：轨道车道（电车/火车）也属于"车辆持有者"——全局模式会剥它们，
			/// 所以覆盖率读数必须把它们算进分母（见 IsGlobalLaneByChunk）。</summary>
			[ReadOnly]
			public ComponentTypeHandle<TrackLane> m_TrackLaneType;

			[ReadOnly]
			public ComponentTypeHandle<PedestrianLane> m_PedestrianLaneType;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<ConnectionLane> m_ConnectionLaneData;

			[ReadOnly]
			public ComponentLookup<Owner> m_OwnerData;

			[ReadOnly]
			public ComponentLookup<Edge> m_EdgeData;

			[ReadOnly]
			public ComponentLookup<Node> m_NodeData;

			/// <summary>v0.8.7：样本里要带持有者的曲线中点，作者才能在游戏里走到那个路口去看。
			/// 只在采样本这一支用，所以没样本可采时它一次都不读。</summary>
			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			/// <summary>v0.8.10：出入口模式的邻域集合（全局模式传 default，这一格恒为 0）。
			/// 作用是把原来 AuditAccessSetJob 那遍**单独**的 O(集合) 随机访问扫描
			/// （作者实机 6.5 万条 ⇒ 主线程阻塞 15~40 ms）并进这一遍全城扫描：
			/// 每条车辆车道多一次哈希查找，换来"一次心跳只扫一遍"。
			/// 判读口径与剥离侧同源（<see cref="CrosswalkScopeFor"/> 按持有者判横道），
			/// 所以"0 = 干净"仍然等价于"该删的都删了"。</summary>
			[ReadOnly]
			public NativeHashSet<Entity> m_Set;

			[NativeDisableParallelForRestriction]
			public NativeArray<int> m_Counters;

			/// <summary>v0.8.7：残留条目样本，**每 chunk 一格**（宽度 <see cref="kAuditSamplePerChunk"/>，
			/// 布局见该常量）。只在"持有者是车辆车道却还留着行人条目"时写第一条；槽位由
			/// <c>unfilteredChunkIndex</c> 独占 ⇒ 无跨线程争用、不会把两条车道的字段混成一条样本。</summary>
			[NativeDisableParallelForRestriction]
			public NativeArray<int> m_Samples;

			/// <summary>样本类型位（<see cref="m_Samples"/> 里第 5 格）。</summary>
			private const int kSampleBitCar = 1;
			private const int kSampleBitParking = 2;
			private const int kSampleBitGarage = 4;
			private const int kSampleBitConnection = 8;
			private const int kSampleBitArea = 16;
			private const int kSampleBitPedestrian = 32;
			private const int kSampleBitBuildingOwned = 64;
			/// <summary>v0.8.10：轨道车道（电车/火车）。</summary>
			private const int kSampleBitTrack = 128;

			private void AddSample(int chunkSlot, Entity holder, Entity target, int holderBits, int pedFlags, int connFlags, float3 mid, int hit)
			{
				if (!m_Samples.IsCreated)
				{
					return;
				}
				// 每 chunk 只留第一条：槽位是本 chunk 独占的，所以既不用原子也不需要重试；
				// 调用点已经保证"本 chunk 还没写过"（见 Execute 里的 chunkSampled）。
				int b = chunkSlot * kAuditSamplePerChunk;
				m_Samples[b] = hit;
				m_Samples[b + 1] = holder.Index;
				m_Samples[b + 2] = holder.Version;
				m_Samples[b + 3] = target.Index;
				m_Samples[b + 4] = target.Version;
				m_Samples[b + 5] = holderBits;
				m_Samples[b + 6] = pedFlags;
				m_Samples[b + 7] = connFlags;
				m_Samples[b + 8] = (int)math.round(mid.x);
				m_Samples[b + 9] = (int)math.round(mid.z);
			}

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				bool chunkHasCarLane = chunk.Has(ref m_CarLaneType);
				bool chunkHasParking = chunk.Has(ref m_ParkingLaneType);
				bool chunkHasGarage = chunk.Has(ref m_GarageLaneType);
				bool chunkHasConnection = chunk.Has(ref m_ConnectionLaneType);
				bool chunkHasArea = chunk.Has(ref m_AreaLaneType);
				bool chunkHasTrack = chunk.Has(ref m_TrackLaneType);
				bool chunkHasPedestrian = chunk.Has(ref m_PedestrianLaneType);
				BufferAccessor<LaneOverlap> overlaps = chunk.GetBufferAccessor(ref m_OverlapType);
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				NativeArray<ConnectionLane> connectionArray = chunkHasConnection ? chunk.GetNativeArray(ref m_ConnectionLaneType) : default;
				// 类型位是 chunk 级的（同一 archetype 内所有实体一致），一次算好给样本用。
				int holderBits = (chunkHasCarLane ? kSampleBitCar : 0)
					| (chunkHasParking ? kSampleBitParking : 0)
					| (chunkHasGarage ? kSampleBitGarage : 0)
					| (chunkHasConnection ? kSampleBitConnection : 0)
					| (chunkHasArea ? kSampleBitArea : 0)
					| (chunkHasTrack ? kSampleBitTrack : 0)
					| (chunkHasPedestrian ? kSampleBitPedestrian : 0);
				int lanes = 0;
				int entries = 0;
				int crosswalkLanes = 0;
				int crosswalkEntries = 0;
				bool chunkSampled = false;
				int vehicleLanes = 0;
				int vehicleEntries = 0;
				int roadLanes = 0;
				int roadEntries = 0;
				int vehicleScanned = 0;
				int roadScanned = 0;
				int setLanes = 0;
				int setEntries = 0;
				// v0.8.10：集合口径（出入口模式的邻域集合残留）。全局模式 m_Set 未创建 ⇒ 恒 false，
				// 每条车道只多一次 IsCreated 读取。判读口径与剥离侧同源，见 m_Set 的注释。
				bool useSet = m_Set.IsCreated && m_Set.Count > 0;
				for (int i = 0; i < chunk.Count; i++)
				{
					Entity lane = entities[i];
					// 与剥离侧同口径：先认"这是不是一条全局模式会剥的车辆车道"，再分路网/设施。
					bool holderIsVehicle = !chunkHasPedestrian
						&& IsGlobalLaneByChunk(chunkHasCarLane, chunkHasParking, chunkHasGarage,
							chunkHasConnection, chunkHasArea, chunkHasTrack, connectionArray, i);
					bool roadHolder = holderIsVehicle
						&& !IsBuildingOwned(lane, m_OwnerData, m_EdgeData, m_NodeData);
					if (holderIsVehicle)
					{
						vehicleScanned++;
						if (roadHolder)
						{
							roadScanned++;
						}
					}
					DynamicBuffer<LaneOverlap> buffer = overlaps[i];
					int hit = 0;
					int crosswalkHit = 0;
					Entity firstPedTarget = Entity.Null;
					for (int j = 0; j < buffer.Length; j++)
					{
						// 审计要的是"原版重建有没有发生过"，所以横道条目也计入（includeCrosswalks=1）：
						// 全局模式下我们连横道都删，只数非横道会误判。
						if (IsPedestrianLaneEntity(buffer[j].m_Other, m_PedestrianLaneData, m_ConnectionLaneData, 1))
						{
							hit++;
							if (firstPedTarget == Entity.Null)
							{
								firstPedTarget = buffer[j].m_Other;
							}
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
						if (useSet && m_Set.Contains(lane))
						{
							setLanes++;
							setEntries += hit;
						}
						if (holderIsVehicle)
						{
							vehicleLanes++;
							vehicleEntries += hit;
							if (roadHolder)
							{
								roadLanes++;
								roadEntries += hit;
							}
							// v0.8.7：车辆持有者还留着行人条目 = 这一条就是"该剥没剥干净"的现场，采样本。
							// 路网与设施两类都要采（作者报的两种形状一边是路口横道、一边是设施出入口，
							// 只采其中一类就又回到"靠肉眼猜"）；每 chunk 只留第一条，槽位本块独占。
							if (m_Samples.IsCreated && !chunkSampled)
							{
								int sampleBits = holderBits | (roadHolder ? 0 : kSampleBitBuildingOwned);
								int pedFlags = m_PedestrianLaneData.TryGetComponent(firstPedTarget, out PedestrianLane samplePed)
									? (int)samplePed.m_Flags : 0;
								int connFlags = m_ConnectionLaneData.TryGetComponent(firstPedTarget, out ConnectionLane sampleConn)
									? (int)sampleConn.m_Flags : 0;
								float3 mid = m_CurveData.TryGetComponent(lane, out Curve sampleCurve)
									? MathUtils.Position(sampleCurve.m_Bezier, 0.5f) : float3.zero;
								AddSample(unfilteredChunkIndex, lane, firstPedTarget, sampleBits, pedFlags, connFlags, mid, hit);
								chunkSampled = true;
							}
						}
					}
					if (crosswalkHit > 0)
					{
						crosswalkLanes++;
						crosswalkEntries += crosswalkHit;
					}
				}
				int baseIndex = unfilteredChunkIndex * kAuditCountersPerChunk;
				m_Counters[baseIndex + kAuditLanes] = lanes;
				m_Counters[baseIndex + kAuditEntries] = entries;
				m_Counters[baseIndex + kAuditCrosswalkLanes] = crosswalkLanes;
				m_Counters[baseIndex + kAuditCrosswalkEntries] = crosswalkEntries;
				m_Counters[baseIndex + kAuditVehicleLanes] = vehicleLanes;
				m_Counters[baseIndex + kAuditVehicleEntries] = vehicleEntries;
				m_Counters[baseIndex + kAuditRoadLanes] = roadLanes;
				m_Counters[baseIndex + kAuditRoadEntries] = roadEntries;
				m_Counters[baseIndex + kAuditVehicleScanned] = vehicleScanned;
				m_Counters[baseIndex + kAuditRoadScanned] = roadScanned;
				// v0.8.10：邻域集合口径（出入口模式；全局模式恒 0）。
				m_Counters[baseIndex + kAuditSetLanes] = setLanes;
				m_Counters[baseIndex + kAuditSetEntries] = setEntries;
			}
		}
	}
}

