using System;
using System.Runtime.CompilerServices;
using Colossal.Mathematics;
using Game;
using Game.Common;
using Game.Creatures;
using Game.Net;
using Game.Simulation;
using Game.Tools;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
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
	/// 1. 邻域集合：把"邻近出入口锚点的行人道/车辆车道"预收进 NativeHashSet（双缓冲），
	///    分帧增量构建（每帧主链上处理 512 个候选，全城几万条约 1-2 秒完成一轮）。
	///    A/C 只处理集合内实体——每帧成本从 O(全城) 降到 O(邻域，几百条)。
	///    构建期间用上一轮集合（新车道最多滞后一轮），双缓冲避免判定空窗。
	/// 2. C 改为单线程 IJob 直接遍历行人道集合（省掉全城 chunk 迭代）。
	/// 3. 删除干预点 B（连接道注册删除）——v0.3 实机日志证明连接道没有 LaneObject
	///    buffer（registration lanes = 0），它是每帧空转的冗余 job。
	/// 4. 全局模式：A 保持全城扫描（本就有效），C 完全跳过（A 已删掉全部车辆车道
	///    的行人 overlap 条目，车辆逻辑上再也读不到任何行人道，C 纯冗余）。
	///
	/// 保留的语义（与 v0.6.0 完全一致）：
	/// - 人行横道（PedestrianLaneFlags.Crosswalk）永不处理，横道避让保持原版；
	/// - 车车 overlap、行人寻路不碰；
	/// - ECB 帧末回放、CarNav 帧中读取 → C 必须每帧执行（缺席一帧车辆就会看到重加的行人）；
	/// - A 的重加来源（LaneOverlapSystem 车道重建）频率低，每 4 帧足够。
	/// </summary>
	[Preserve]
	public partial class AccessZoneOverlapSystem : GameSystemBase
	{
		/// <summary>干预点 A（overlap 删除）的执行间隔（模拟帧）。C 每帧执行。</summary>
		private const int kInterval = 4;

		/// <summary>锚点网格重建间隔（模拟帧）。连接道集合只在修路/建拆时变化。</summary>
		private const int kGridRebuildInterval = 512;

		/// <summary>空间判定的有效半径（米）。锚点落在 32m 网格格子里，判定时校验实际距离。</summary>
		private const float kAccessRadius = 32f;

		private const float kGridCellSize = 32f;

		/// <summary>每帧在主链上处理的候选实体数（集合分帧构建的步长）。</summary>
		private const int kSetBuildStep = 512;

		/// <summary>每隔多少个模拟帧输出一次心跳日志（诊断用）。</summary>
		private const int kHeartbeatInterval = 1800;

		private SimulationSystem m_SimulationSystem;
		private EndFrameBarrier m_EndFrameBarrier;
		private EntityQuery m_OverlapQuery;
		private EntityQuery m_PedestrianQuery;
		private EntityQuery m_AnchorQuery;

		private NativeHashMap<int2, FixedList128Bytes<float2>> m_AnchorGrid;

		/// <summary>邻域集合（双缓冲）：偶数轮 A/B，奇数轮 B/A。Item1 = 活跃（判定用），Item2 = 构建中。</summary>
		private NativeHashSet<Entity>[] m_PedSets;
		private NativeHashSet<Entity>[] m_VehicleSets;
		private int m_ActiveSetIndex;

		/// <summary>分帧构建的候选列表与游标（-1 表示需要重新收集）。</summary>
		private NativeList<Entity> m_Candidates;
		private int m_Cursor;
		private bool m_SetsInitialized;

		private bool m_LoggedFirstRun;
		private ulong m_TotalPasses;

		[Preserve]
		protected override void OnCreate()
		{
			base.OnCreate();
			m_SimulationSystem = World.GetOrCreateSystemManaged<SimulationSystem>();
			m_EndFrameBarrier = World.GetOrCreateSystemManaged<EndFrameBarrier>();
			m_AnchorGrid = new NativeHashMap<int2, FixedList128Bytes<float2>>(4096, Allocator.Persistent);
			m_PedSets = new NativeHashSet<Entity>[2]
			{
				new NativeHashSet<Entity>(4096, Allocator.Persistent),
				new NativeHashSet<Entity>(4096, Allocator.Persistent)
			};
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
			m_PedestrianQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[2]
				{
					ComponentType.ReadOnly<LaneObject>(),
					ComponentType.ReadOnly<PedestrianLane>()
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
			RequireForUpdate(m_OverlapQuery);
		}

		protected override void OnDestroy()
		{
			if (m_AnchorGrid.IsCreated)
			{
				m_AnchorGrid.Dispose();
			}
			for (int i = 0; i < 2; i++)
			{
				if (m_PedSets[i].IsCreated)
				{
					m_PedSets[i].Dispose();
				}
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
			if (setting == null || !setting.Enabled)
			{
				return;
			}
			bool globalMode = setting.Mode == Setting.kModeGlobal;
			uint frameIndex = m_SimulationSystem.frameIndex;
			m_TotalPasses++;

			if (!m_LoggedFirstRun)
			{
				m_LoggedFirstRun = true;
				LogStatus("first pass", globalMode, setting);
			}
			else if (m_TotalPasses % kHeartbeatInterval == 0)
			{
				LogStatus("heartbeat", globalMode, setting);
			}

			EntityCommandBuffer.ParallelWriter ecb = m_EndFrameBarrier.CreateCommandBuffer().AsParallelWriter();

			// 锚点网格：低频重建（连接道集合只在修路/建拆时变化）。单线程 job，几千条曲线。
			if (frameIndex % kGridRebuildInterval == 0)
			{
				BuildAnchorGridJob buildJob = new BuildAnchorGridJob
				{
					m_Entities = m_AnchorQuery.ToEntityArray(Allocator.TempJob),
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_Grid = m_AnchorGrid
				};
				Dependency = buildJob.Schedule(Dependency);
			}

			if (!globalMode)
			{
				// 邻域集合分帧构建（access 模式专属）。每帧处理 kSetBuildStep 个候选；
				// 一轮完成后交换双缓冲并重新收集候选。构建期间判定用上一轮集合。
				AdvanceAccessSets(setting);
			}

			// 干预点 A（overlap 删除）：每 4 帧。
			// access 模式：全城 chunk 迭代但每实体先查集合（一次哈希查询），只有邻域车道才做
			// overlap 扫描与组件查找；全局模式：全量处理（v0.2 实测有效）。
			if (frameIndex % kInterval == 0)
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
					m_AccessVehicleSet = m_VehicleSets[m_ActiveSetIndex],
					m_UseSet = !globalMode,
					m_Mode = setting.Mode,
					m_IncludeGarage = setting.IncludeGarageLanes ? 1 : 0,
					m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
					m_IncludeParkingLots = setting.IncludeParkingLotLanes ? 1 : 0,
					m_Ecb = ecb
				};
				Dependency = overlapJob.ScheduleParallel(m_OverlapQuery, Dependency);
			}

			// 干预点 C（邻近行人道注册删除）：每帧，仅 access 模式。
			// 单线程 job 直接遍历行人道集合（几百条），省掉全城 chunk 迭代。
			// 全局模式跳过：A 已删掉全部车辆车道 overlap 的行人条目，C 纯冗余。
			if (!globalMode && setting.IncludeConnectionLanes)
			{
				StripPedestrianOnNearbyWalkwaysJob walkwayJob = new StripPedestrianOnNearbyWalkwaysJob
				{
					m_Set = m_PedSets[m_ActiveSetIndex],
					m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
					m_CreatureData = GetComponentLookup<Creature>(isReadOnly: true),
					m_LaneObjects = GetBufferLookup<LaneObject>(isReadOnly: false),
					m_Ecb = m_EndFrameBarrier.CreateCommandBuffer()
				};
				Dependency = walkwayJob.Schedule(Dependency);
			}

			m_EndFrameBarrier.AddJobHandleForProducer(Dependency);
		}

		private void LogStatus(string tag, bool globalMode, Setting setting)
		{
			AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem {tag}: pass {m_TotalPasses}, {m_OverlapQuery.CalculateEntityCount()} overlap lanes / {m_PedestrianQuery.CalculateEntityCount()} ped lanes / {m_AnchorQuery.CalculateEntityCount()} anchors / vehicle set {m_VehicleSets[m_ActiveSetIndex].Count} / ped set {m_PedSets[m_ActiveSetIndex].Count}, mode={(globalMode ? "global" : "access-zones")}");
		}

		/// <summary>
		/// 邻域集合的分帧构建。每帧处理 kSetBuildStep 个候选；一轮结束后交换双缓冲
		/// （构建集 ↔ 活跃集）并重新收集候选。首帧构建期间活跃集为空 = 短暂原版行为。
		/// </summary>
		private void AdvanceAccessSets(Setting setting)
		{
			if (m_Cursor < 0 || m_Cursor >= m_Candidates.Length)
			{
				// 一轮结束：交换双缓冲，构建集变活跃集；清空新构建集开始下一轮。
				if (m_SetsInitialized)
				{
					m_ActiveSetIndex = 1 - m_ActiveSetIndex;
				}
				m_SetsInitialized = true;
				int buildIndex = 1 - m_ActiveSetIndex;
				m_PedSets[buildIndex].Clear();
				m_VehicleSets[buildIndex].Clear();

				// 收集候选需完成在途 job（ECB 回放可能移动 chunk）。
				Dependency.Complete();
				m_Candidates.Clear();
				m_Candidates.AddRange(m_PedestrianQuery.ToEntityArray(Allocator.Temp));
				m_Candidates.AddRange(m_OverlapQuery.ToEntityArray(Allocator.Temp));
				m_Cursor = 0;
				AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem set rebuild: {m_Candidates.Length} candidates, active sets now ped={m_PedSets[m_ActiveSetIndex].Count} vehicle={m_VehicleSets[m_ActiveSetIndex].Count}");
			}

			int endIndex = math.min(m_Cursor + kSetBuildStep, m_Candidates.Length);
			if (endIndex <= m_Cursor)
			{
				return;
			}
			BuildAccessSetsJob job = new BuildAccessSetsJob
			{
				m_Candidates = m_Candidates,
				m_Start = m_Cursor,
				m_End = endIndex,
				m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
				m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
				m_Grid = m_AnchorGrid,
				m_BuildPedSet = m_PedSets[1 - m_ActiveSetIndex],
				m_BuildVehicleSet = m_VehicleSets[1 - m_ActiveSetIndex]
			};
			Dependency = job.Schedule(Dependency);
			m_Cursor = endIndex;
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

		/// <summary>分帧构建邻域集合：候选中邻近锚点的行人道/车辆车道分别入集。</summary>
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
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			public NativeHashSet<Entity> m_BuildPedSet;

			public NativeHashSet<Entity> m_BuildVehicleSet;

			public void Execute()
			{
				int end = math.min(m_End, m_Candidates.Length);
				for (int i = m_Start; i < end; i++)
				{
					Entity lane = m_Candidates[i];
					if (!m_CurveData.TryGetComponent(lane, out Curve curve) || !IsNearAnchor(curve.m_Bezier, m_Grid))
					{
						continue;
					}
					if (m_PedestrianLaneData.HasComponent(lane))
					{
						m_BuildPedSet.Add(lane);
					}
					else
					{
						m_BuildVehicleSet.Add(lane);
					}
				}
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

		/// <summary>
		/// 干预点 A：删除车辆车道 LaneOverlap 缓冲中指向行人道的条目。
		/// access 模式下先用邻域集合过滤（一次哈希查询），仅邻域车道进入逐条扫描。
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

			[ReadOnly]
			public NativeHashSet<Entity> m_AccessVehicleSet;

			public bool m_UseSet;

			public int m_Mode;

			public int m_IncludeGarage;

			public int m_IncludeConnections;

			public int m_IncludeParkingLots;

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
						// access 模式：类型判定（车库/连接道/区域车道/建筑自有）或邻域集合成员（引道）。
						isAccess = IsAccessLaneByType(m_Mode, m_IncludeGarage, m_IncludeConnections, m_IncludeParkingLots,
							chunkHasCarLane, chunkHasParking, chunkHasGarage, chunkHasConnection, chunkHasArea,
							connectionArray, i, lane, m_OwnerData, m_EdgeData, m_NodeData)
							|| (m_IncludeConnections != 0 && m_AccessVehicleSet.Contains(lane));
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
						if (!IsPedestrianLaneEntity(buffer[j].m_Other))
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
					if (IsPedestrianLaneEntity(buffer[i].m_Other))
					{
						return true;
					}
				}
				return false;
			}

			private bool IsPedestrianLaneEntity(Entity other)
			{
				if (m_PedestrianLaneData.TryGetComponent(other, out PedestrianLane ped))
				{
					// 人行横道（Crosswalk flag）不删：横道上的正常车让人行为必须保持原版。
					if ((ped.m_Flags & PedestrianLaneFlags.Crosswalk) != 0)
					{
						return false;
					}
					return true;
				}
				if (m_ConnectionLaneData.TryGetComponent(other, out ConnectionLane conn)
					&& (conn.m_Flags & ConnectionLaneFlags.Pedestrian) != 0
					&& (conn.m_Flags & (ConnectionLaneFlags.Road | ConnectionLaneFlags.Track | ConnectionLaneFlags.Parking)) == 0)
				{
					return true;
				}
				return false;
			}
		}

		/// <summary>
		/// 干预点 C：删除邻域行人道的行人注册（每帧，access 模式）。
		/// 人行横道（Crosswalk flag）整条跳过——横道上的行人注册保持原样，车辆继续让行。
		/// </summary>
		[BurstCompile]
		private struct StripPedestrianOnNearbyWalkwaysJob : IJob
		{
			[ReadOnly]
			public NativeHashSet<Entity> m_Set;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public ComponentLookup<Creature> m_CreatureData;

			[ReadOnly]
			public BufferLookup<LaneObject> m_LaneObjects;

			public EntityCommandBuffer m_Ecb;

			public void Execute()
			{
				foreach (Entity lane in m_Set)
				{
					if (m_PedestrianLaneData.TryGetComponent(lane, out PedestrianLane ped)
						&& (ped.m_Flags & PedestrianLaneFlags.Crosswalk) != 0)
					{
						continue;
					}
					if (!m_LaneObjects.TryGetBuffer(lane, out DynamicBuffer<LaneObject> buffer) || buffer.Length == 0)
					{
						continue;
					}
					bool has = false;
					for (int j = 0; j < buffer.Length; j++)
					{
						if (m_CreatureData.HasComponent(buffer[j].m_LaneObject))
						{
							has = true;
							break;
						}
					}
					if (!has)
					{
						continue;
					}
					DynamicBuffer<LaneObject> replacement = m_Ecb.SetBuffer<LaneObject>(lane);
					for (int j = 0; j < buffer.Length; j++)
					{
						if (!m_CreatureData.HasComponent(buffer[j].m_LaneObject))
						{
							replacement.Add(buffer[j]);
						}
					}
				}
			}
		}
	}
}
