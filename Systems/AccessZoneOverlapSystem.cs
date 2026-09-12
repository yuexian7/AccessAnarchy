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
	/// 让车辆在指定范围内不再避让行人（v0.4，双干预点 + 空间判定）。
	///
	/// 机制结论（对 Game.dll 1.6.0f1 反编译核实 + 实机日志验证）：
	/// 车辆"看见"行人的唯一通道是车道的 LaneObject 注册缓冲——CarNavigationSystem 的
	/// CarLaneSpeedIterator 无论走 CheckCurrentLane（本车道）还是 CheckOverlappingLanes
	/// （经 LaneOverlap 指向的对端车道），最终都读某个车道的 LaneObject buffer 找行人
	/// （带 Game.Creatures.Creature 组件）并压低 CarNavigation.m_MaxSpeed。
	///
	/// 关键事实（每一条都有实机/反编译证据，见 开发笔记.md）：
	/// - 连接道（driveway/garage，全部带 SecondaryLane）没有 LaneObject/LaneOverlap buffer，
	///   CarLaneSpeedIterator 对连接道完全不检查（无 CarLane/PedestrianLane 组件），
	///   LaneOverlapSystem 也排除 SecondaryLane——连接道自身没有任何可干预数据。
	/// - 实机日志（v0.3）：registration lanes = 0（连接道/Garage/AreaLane 无 LaneObject
	///   buffer），而全局模式（删全部车辆车道 overlap）实测有效——所以实机看到的
	///   "出入口避让"发生在【出入口紧邻的普通马路车道（引道）与行人道的 LaneOverlap】上，
	///   这类车道是普通 CarLane，按车道类型判定永远覆盖不到。
	///
	/// 因此 v0.4 引入空间判定：以连接道/车库坡道/区域车道的曲线控制点为锚点建网格
	/// （32m 格 + 32m 距离校验），任何【邻近锚点】的车道都视为出入口区域：
	/// A（每 4 帧）：邻近车道的 LaneOverlap 缓冲删除行人条目（引道↔人行道/横道）。
	/// B（每帧）：连接道/车库坡道/区域车道的 LaneObjects 删除行人条目（兜底，通常 0 条）。
	/// C（每帧）：邻近行人道（PedestrianLane）的 LaneObjects 删除行人条目——横穿出入口
	///     区域的行人无论注册在哪条行人道上都会被摘除，釜底抽薪。
	///
	/// 干预点 B/C 的重加来源是行人在自己的 1/16 切片帧经 CheckChanges 的 Update 动作
	/// （UpdateLaneObject 是"有则更新、无则添加"），ECB 在帧末回放、CarNav 帧中读取，
	/// 所以 B/C 必须每帧执行（任何一帧缺席，CarNav 就会读到重加的行人）。
	///
	/// 不碰的东西：车辆与车辆之间的 overlap / 注册条目；远离出入口的行人道上的行人注册
	/// （马路/横道的正常避让保持原版）。行人的寻路与行走不读取这些数据。
	/// </summary>
	[Preserve]
	public partial class AccessZoneOverlapSystem : GameSystemBase
	{
		/// <summary>干预点 A（overlap 删除）的执行间隔（模拟帧）。B/C 每帧执行。</summary>
		private const int kInterval = 4;

		/// <summary>锚点网格重建间隔（模拟帧）。连接道集合只在修路/建拆时变化。</summary>
		private const int kGridRebuildInterval = 512;

		/// <summary>空间判定的有效半径（米）。锚点落在 32m 网格格子里，判定时校验实际距离。</summary>
		private const float kAccessRadius = 32f;

		private const float kGridCellSize = 32f;

		/// <summary>每隔多少个模拟帧输出一次心跳日志（诊断用）。</summary>
		private const int kHeartbeatInterval = 1800;

		private SimulationSystem m_SimulationSystem;
		private EndFrameBarrier m_EndFrameBarrier;
		private EntityQuery m_OverlapQuery;
		private EntityQuery m_RegistrationQuery;
		private EntityQuery m_AnchorQuery;
		private EntityQuery m_PedestrianQuery;
		private NativeHashMap<int2, FixedList128Bytes<float2>> m_AnchorGrid;
		private bool m_LoggedFirstRun;
		private ulong m_TotalPasses;

		[Preserve]
		protected override void OnCreate()
		{
			base.OnCreate();
			m_SimulationSystem = World.GetOrCreateSystemManaged<SimulationSystem>();
			m_EndFrameBarrier = World.GetOrCreateSystemManaged<EndFrameBarrier>();
			m_AnchorGrid = new NativeHashMap<int2, FixedList128Bytes<float2>>(4096, Allocator.Persistent);
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
			m_RegistrationQuery = GetEntityQuery(new EntityQueryDesc
			{
				All = new ComponentType[1]
				{
					ComponentType.ReadOnly<LaneObject>()
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
			RequireForUpdate(m_OverlapQuery);
		}

		protected override void OnDestroy()
		{
			if (m_AnchorGrid.IsCreated)
			{
				m_AnchorGrid.Dispose();
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
			uint frameIndex = m_SimulationSystem.frameIndex;
			m_TotalPasses++;

			if (!m_LoggedFirstRun)
			{
				m_LoggedFirstRun = true;
				AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem first pass: {m_OverlapQuery.CalculateEntityCount()} overlap lanes / {m_RegistrationQuery.CalculateEntityCount()} registration lanes / {m_AnchorQuery.CalculateEntityCount()} anchors / {m_PedestrianQuery.CalculateEntityCount()} pedestrian lanes, mode={(setting.Mode == Setting.kModeGlobal ? "global" : "access-zones")}");
			}
			else if (m_TotalPasses % kHeartbeatInterval == 0)
			{
				AccessAnarchyMod.log.Info($"AccessZoneOverlapSystem heartbeat: pass {m_TotalPasses}, {m_OverlapQuery.CalculateEntityCount()} overlap lanes / {m_RegistrationQuery.CalculateEntityCount()} registration lanes / {m_AnchorQuery.CalculateEntityCount()} anchors / {m_PedestrianQuery.CalculateEntityCount()} pedestrian lanes, mode={(setting.Mode == Setting.kModeGlobal ? "global" : "access-zones")}");
			}

			EntityCommandBuffer.ParallelWriter ecb = m_EndFrameBarrier.CreateCommandBuffer().AsParallelWriter();

			// 锚点网格：低频重建（连接道集合只在修路/建拆时变化）。单线程 IJob，量小。
			if (frameIndex % kGridRebuildInterval == 0)
			{
				NativeArray<Entity> anchorEntities = m_AnchorQuery.ToEntityArray(Allocator.TempJob);
				BuildAnchorGridJob buildJob = new BuildAnchorGridJob
				{
					m_Entities = anchorEntities,
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_Grid = m_AnchorGrid
				};
				Dependency = buildJob.Schedule(Dependency);
				anchorEntities.Dispose(Dependency);
			}

			EntityCommandBuffer.ParallelWriter ecb2 = ecb;

			// 干预点 A（overlap 删除）：每 4 帧。重加来源是 LaneOverlapSystem 的车道重建，
			// 频率低。判定 = 出入口类型车道（原逻辑）或邻近锚点的引道车道（v0.4 空间判定）。
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
					m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
					m_Grid = m_AnchorGrid,
					m_Mode = setting.Mode,
					m_IncludeGarage = setting.IncludeGarageLanes ? 1 : 0,
					m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
					m_IncludeParkingLots = setting.IncludeParkingLotLanes ? 1 : 0,
					m_Ecb = ecb2
				};
				Dependency = overlapJob.ScheduleParallel(m_OverlapQuery, Dependency);
			}

			// 干预点 B（行人注册删除，连接道类兜底）：每帧。
			StripPedestrianRegistrationsJob registrationJob = new StripPedestrianRegistrationsJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_CreatureData = GetComponentLookup<Creature>(isReadOnly: true),
				m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
				m_EdgeData = GetComponentLookup<Edge>(isReadOnly: true),
				m_NodeData = GetComponentLookup<Node>(isReadOnly: true),
				m_CarLaneType = GetComponentTypeHandle<CarLane>(isReadOnly: true),
				m_ParkingLaneType = GetComponentTypeHandle<ParkingLane>(isReadOnly: true),
				m_GarageLaneType = GetComponentTypeHandle<GarageLane>(isReadOnly: true),
				m_ConnectionLaneType = GetComponentTypeHandle<ConnectionLane>(isReadOnly: true),
				m_AreaLaneType = GetComponentTypeHandle<AreaLane>(isReadOnly: true),
				m_LaneObjectType = GetBufferTypeHandle<LaneObject>(isReadOnly: true),
				m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
				m_Grid = m_AnchorGrid,
				m_Mode = setting.Mode,
				m_IncludeGarage = setting.IncludeGarageLanes ? 1 : 0,
				m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
				m_IncludeParkingLots = setting.IncludeParkingLotLanes ? 1 : 0,
				m_Ecb = ecb2
			};
			Dependency = registrationJob.ScheduleParallel(m_RegistrationQuery, Dependency);

			// 干预点 C（行人注册删除，邻近行人道）：每帧。横穿出入口区域的行人无论注册在
			// 哪条行人道上都会被摘除，车辆经任何路径都读不到。人行横道（Crosswalk flag）例外。
			StripPedestrianOnNearbyWalkwaysJob walkwayJob = new StripPedestrianOnNearbyWalkwaysJob
			{
				m_EntityType = GetEntityTypeHandle(),
				m_CreatureData = GetComponentLookup<Creature>(isReadOnly: true),
				m_PedestrianLaneData = GetComponentLookup<PedestrianLane>(isReadOnly: true),
				m_LaneObjectType = GetBufferTypeHandle<LaneObject>(isReadOnly: true),
				m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
				m_Grid = m_AnchorGrid,
				m_IncludeConnections = setting.IncludeConnectionLanes ? 1 : 0,
				m_Mode = setting.Mode,
				m_Ecb = ecb2
			};
			Dependency = walkwayJob.ScheduleParallel(m_PedestrianQuery, Dependency);

			m_EndFrameBarrier.AddJobHandleForProducer(Dependency);
		}

		private static int2 GridKey(float3 position)
		{
			return (int2)math.floor(position.xz / kGridCellSize);
		}

		/// <summary>车道（curve）是否邻近任何出入口锚点（3×3 邻域 + 距离校验）。</summary>
		private static bool IsNearAnchor(Curve curve, NativeHashMap<int2, FixedList128Bytes<float2>> grid)
		{
			Bezier4x3 bezier = curve.m_Bezier;
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

		private static bool IsAccessLane(int mode, int includeGarage, int includeConnections, int includeParkingLots,
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
				// 不按 flags 过滤：车辆连接道与行人连接道都处理。
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
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

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
					bool isAccess = IsAccessLane(m_Mode, m_IncludeGarage, m_IncludeConnections, m_IncludeParkingLots,
						chunkHasCarLane, chunkHasParking, chunkHasGarage, chunkHasConnection, chunkHasArea,
						connectionArray, i, lane, m_OwnerData, m_EdgeData, m_NodeData);
					if (!isAccess && m_IncludeConnections != 0 && chunkHasCarLane && m_CurveData.TryGetComponent(lane, out Curve curve))
					{
						// v0.4 空间判定：出入口连接道邻近的普通车道（引道/紧邻横道）也算出入口区域。
						isAccess = IsNearAnchor(curve, m_Grid);
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
					// 人行横道（Crosswalk flag）不删：横道上的正常车让人行为必须保持原版
					// （出入口与人行横道在城市里经常紧邻，v0.4 的空间判定曾把横道一起吃掉）。
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

		/// <summary>干预点 B：删除出入口类型车道 LaneObject 注册缓冲里的行人与动物条目（兜底）。</summary>
		[BurstCompile]
		private struct StripPedestrianRegistrationsJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public ComponentLookup<Creature> m_CreatureData;

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
			public BufferTypeHandle<LaneObject> m_LaneObjectType;

			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

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
				BufferAccessor<LaneObject> laneObjects = chunk.GetBufferAccessor(ref m_LaneObjectType);
				NativeArray<ConnectionLane> connectionArray = chunkHasConnection ? chunk.GetNativeArray(ref m_ConnectionLaneType) : default;

				for (int i = 0; i < chunk.Count; i++)
				{
					Entity lane = entities[i];
					if (!IsAccessLane(m_Mode, m_IncludeGarage, m_IncludeConnections, m_IncludeParkingLots,
						chunkHasCarLane, chunkHasParking, chunkHasGarage, chunkHasConnection, chunkHasArea,
						connectionArray, i, lane, m_OwnerData, m_EdgeData, m_NodeData))
					{
						continue;
					}
					Strip(lane, laneObjects[i], unfilteredChunkIndex, m_Ecb);
				}
			}

			private void Strip(Entity lane, DynamicBuffer<LaneObject> buffer, int sortKey, EntityCommandBuffer.ParallelWriter ecb)
			{
				bool has = false;
				for (int i = 0; i < buffer.Length; i++)
				{
					if (m_CreatureData.HasComponent(buffer[i].m_LaneObject))
					{
						has = true;
						break;
					}
				}
				if (!has)
				{
					return;
				}
				DynamicBuffer<LaneObject> replacement = ecb.SetBuffer<LaneObject>(sortKey, lane);
				for (int j = 0; j < buffer.Length; j++)
				{
					if (!m_CreatureData.HasComponent(buffer[j].m_LaneObject))
					{
						replacement.Add(buffer[j]);
					}
				}
			}
		}

		/// <summary>
		/// 干预点 C：删除出入口锚点邻近行人道的行人注册。横穿出入口区域的行人无论注册在
		/// 人行道还是横道上都会被摘除，车辆经任何路径（本车道/overlap 对端）都读不到。
		/// 人行横道（Crosswalk flag）例外——横道上的正常避让保持原版。
		/// </summary>
		[BurstCompile]
		private struct StripPedestrianOnNearbyWalkwaysJob : IJobChunk
		{
			[ReadOnly]
			public EntityTypeHandle m_EntityType;

			[ReadOnly]
			public ComponentLookup<Creature> m_CreatureData;

			[ReadOnly]
			public ComponentLookup<PedestrianLane> m_PedestrianLaneData;

			[ReadOnly]
			public BufferTypeHandle<LaneObject> m_LaneObjectType;

			[ReadOnly]
			public ComponentLookup<Curve> m_CurveData;

			[ReadOnly]
			public NativeHashMap<int2, FixedList128Bytes<float2>> m_Grid;

			public int m_IncludeConnections;

			public int m_Mode;

			public EntityCommandBuffer.ParallelWriter m_Ecb;

			public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
			{
				if (m_IncludeConnections == 0 && m_Mode != Setting.kModeGlobal)
				{
					return;
				}
				NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
				BufferAccessor<LaneObject> laneObjects = chunk.GetBufferAccessor(ref m_LaneObjectType);

				for (int i = 0; i < chunk.Count; i++)
				{
					Entity lane = entities[i];
					// 人行横道例外：横道上的行人注册保持原样，车辆继续让行。
					if (m_PedestrianLaneData.TryGetComponent(lane, out PedestrianLane ped)
						&& (ped.m_Flags & PedestrianLaneFlags.Crosswalk) != 0)
					{
						continue;
					}
					if (m_Mode != Setting.kModeGlobal)
					{
						if (!m_CurveData.TryGetComponent(lane, out Curve curve) || !IsNearAnchor(curve, m_Grid))
						{
							continue;
						}
					}
					DynamicBuffer<LaneObject> buffer = laneObjects[i];
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
					DynamicBuffer<LaneObject> replacement = m_Ecb.SetBuffer<LaneObject>(unfilteredChunkIndex, lane);
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
