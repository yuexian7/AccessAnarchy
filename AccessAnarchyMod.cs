using Colossal.Logging;
using Game;
using Game.Input;
using Game.Modding;
using Game.SceneFlow;
using Game.Simulation;
using AccessAnarchy.Systems;
using UnityEngine.InputSystem;

namespace AccessAnarchy
{
	/// <summary>
	/// Access Anarchy 模组入口。
	///
	/// 核心策略：零 Harmony 补丁的 ECS 数据编辑。
	/// 车辆避让行人的机制与干预点见 Systems\AccessZoneOverlapSystem.cs 的类注释。
	/// </summary>
	public class AccessAnarchyMod : IMod
	{
		public static ILog log = LogManager.GetLogger($"{nameof(AccessAnarchy)}.{nameof(AccessAnarchyMod)}").SetShowsErrorsInUI(false);

		private Setting m_Setting;

		public void OnLoad(UpdateSystem updateSystem)
		{
			log.Info($"Access Anarchy v{ModInfo.kVersion} loading (native-crash hardening: own job chain instead of Dependency, world caches reset on preload; scope merged into one switch; performance: anchors collected from vehicle lanes only, access-zone rebuild only when something changed, idle steps leave before touching the barrier; correctness: rebuilt pedestrian lanes re-strip the vehicle lane paired with them, cargo connections back inside the access-zone scope)...");
			if (GameManager.instance.modManager.TryGetExecutableAsset(this, out var asset))
			{
				log.Info($"Current mod asset at {asset.path}");
			}

			m_Setting = new Setting(this);
			Setting.Instance = m_Setting;
			m_Setting.RegisterInOptionsUI();

			// 官方 12 种语言全部注册（语言代码与游戏 GetSupportedLocales() 一致）。
			LocaleTable.SetActiveLocale(GameManager.instance.localizationManager.activeLocaleId);
			string[] locales = LocaleTable.Locales;
			for (int i = 0; i < locales.Length; i++)
			{
				GameManager.instance.localizationManager.AddSource(locales[i], new LocaleSource(m_Setting, locales[i]));
			}

			// 注册快捷键（出厂默认留空，玩家在「快捷键设置」板块里自己按）。官方路线：
			// RegisterKeyBindings 把 [SettingsUIKeyboardAction]/[SettingsUIKeyboardBinding]
			// 声明的动作挂进 InputManager（map = 模组 id），onInteraction 在按下沿触发。
			//
			// 顺序必须是「先注册、后读档」，与官方模板 template/content/Mod.cs 一致
			// （模板里 RegisterKeyBindings 在前、AssetDatabase.global.LoadSettings 在后）。
			// 原因：框架读档后的键位落地钩子 ModSetting.ApplyKeyBindings() 带 [AfterDecode]，
			// 且被 keyBindingRegistered 闸门挡住（ModSetting.cs:269-277），而这个标志要到
			// RegisterKeyBindings 结尾才置为 true（:228）。旧版本把 LoadSettings 放在前面，
			// 于是这个钩子从来没跑过，RegisterKeyBindings 直接拿盘上的 m_Path 去注册——
			// 而空 m_Path 会被当成有效覆盖写进输入系统（CompositeInstance.canBeEmpty 默认 true，
			// CompositeInstance.cs:12；覆盖写入点 InputManager.cs:1029-1034），
			// 结果重启后 F3/F4 全部失效，退出时又把空值存回去自我延续。
			m_Setting.RegisterKeyBindings();

			// 从 AccessAnarchy.coc 读取玩家上次保存的设置与键位（含改过的键位——
			// 上面的注册顺序保证 [AfterDecode] ApplyKeyBindings 能把玩家键盖回默认键之上）。
			Colossal.IO.AssetDatabase.AssetDatabase.global.LoadSettings(nameof(AccessAnarchy), m_Setting, new Setting(this));

			// 键位盘上是什么就是什么：空着就空着（作者 2026-09-25/28 两次明确：首次订阅就该留空，
			// 玩家设过按玩家的，SMC+ 那类外部还原优先级最高，任何一环都不要替玩家回填 F3/F4）。
			// 外部模组（如 SIMPLE MOD CHECKER PLUS）在本函数之后约 3 秒还原它备份的键位/设置时，
			// 后写的赢——我们这边属性都是实时 getter、热键订阅在 ProxyAction 上，所以天然跟随，不需要额外处理。
			AccessAnarchyMod.log.Info($"Settings loaded: Enabled={m_Setting.Enabled}, Mode={m_Setting.Mode}, keyEnabled={FormatPath(m_Setting.ToggleEnabledBinding)}, keyScope={FormatPath(m_Setting.ToggleScopeBinding)}");

			// 盘上值之外，再把"实际注册进 InputManager 的绑定 + 游戏判定的冲突对侧"打一次。
			// OnLoad 这次只反映当下（其它模组可能还没注册完），进游戏后 AccessZoneOverlapSystem
			// 会再打一次 "in-game"，两次一起看才能定位是谁触发了「Key binding conflict detected」。
			Setting.ReportKeyBindings("onLoad");

			ProxyAction toggleEnabled = m_Setting.GetAction(Setting.kToggleEnabledAction);
			toggleEnabled.shouldBeEnabled = true;
			toggleEnabled.onInteraction += OnToggleEnabledInteraction;
			ProxyAction toggleScope = m_Setting.GetAction(Setting.kToggleScopeAction);
			toggleScope.shouldBeEnabled = true;
			toggleScope.onInteraction += OnToggleScopeInteraction;
			log.Info("Hotkey actions registered (we subscribe to the ProxyAction, not to the binding, so player re-binding and empty keys both keep working)");

			// 注册到模拟主循环，并强制排在 CarNavigationSystem 之前：
			// CarNavigationSystem 每帧读取 LaneOverlap / LaneObject 决定避让，我们删完它才读；
			// HumanNavigationSystem（在 CarNavigationSystem 之后，见 Game.Common.SystemOrder
			// 第 310/343 行）的重新注册要等下一帧才会被再次删掉，因此永远追不上。
			updateSystem.UpdateAt<AccessZoneOverlapSystem>(SystemUpdatePhase.GameSimulation);
			updateSystem.UpdateBefore<AccessZoneOverlapSystem, CarNavigationSystem>(SystemUpdatePhase.GameSimulation);

			log.Info("Access Anarchy loaded: AccessZoneOverlapSystem registered in GameSimulation before CarNavigationSystem.");
		}

		/// <summary>把键位打进启动日志用的可读形式：空路径要显式写成 (empty)，
		/// 否则日志里看到的是两个引号中间什么都没有，和缺字段分不清。</summary>
		private static string FormatPath(ProxyBinding binding)
		{
			string path = binding.path;
			return string.IsNullOrEmpty(path) ? "(empty)" : path;
		}

		private static void OnToggleEnabledInteraction(ProxyAction action, InputActionPhase phase)
		{
			if (phase != InputActionPhase.Started)
			{
				return;
			}
			Setting.ToggleEnabledFromHotkey();
		}

		private static void OnToggleScopeInteraction(ProxyAction action, InputActionPhase phase)
		{
			if (phase != InputActionPhase.Started)
			{
				return;
			}
			Setting.ToggleScopeFromHotkey();
		}

		public void OnDispose()
		{
			log.Info("Access Anarchy OnDispose");

			// 稳定性自查（2026-09-30）：退订热键回调。游戏不会替我们销毁 UpdateAt 注册进去的系统
			// （Game.UpdateSystem.cs:158 只是 World.GetOrCreateSystemManaged + Register），
			// 而在模组管理器里"关掉再打开"会让 OnLoad 再跑一遍、对同一个 ProxyAction 再加一次回调 ——
			// 一次按键触发两次 ToggleEnabled = 开关互相抵消。这里把上一轮的订阅读干净。
			if (m_Setting != null)
			{
				UnsubscribeHotkey(Setting.kToggleEnabledAction, OnToggleEnabledInteraction);
				UnsubscribeHotkey(Setting.kToggleScopeAction, OnToggleScopeInteraction);
				// 卸载安全（v0.8.5）：这一句原来裸着。`UnregisterInOptionsUI` 抛异常时
				// `m_Setting = null` 与 `Setting.Instance = null` 都被跳过，ModManager 会记一条
				// "Error disposing mod"——玩家眼里就是"我明明把模组删了，日志里还有 AccessAnarchy 的报错"。
				// 注销选项页只是顺手清理，失败也不能挡住后面的状态复位。
				try
				{
					m_Setting.UnregisterInOptionsUI();
				}
				catch (System.Exception ex)
				{
					log.Warn("Unregistering from options UI failed: " + ex);
				}
				m_Setting = null;
			}
			Setting.Instance = null;
		}

		/// <summary>取消订阅并关掉动作开关。整个动作都要吞异常：退订只是"顺手清理"，
		/// 而 OnDispose 抛出会让模组卸载半途失败（`GetAction` 里 `InputManager.instance` 在关机
		/// 序列里可能已经是 null，见 ModSetting.cs:289-292 它直接解引用）。</summary>
		private void UnsubscribeHotkey(string actionName, System.Action<ProxyAction, InputActionPhase> handler)
		{
			try
			{
				ProxyAction action = m_Setting.GetAction(actionName);
				if (action == null)
				{
					return;
				}
				action.onInteraction -= handler;
				action.shouldBeEnabled = false;
			}
			catch (System.Exception ex)
			{
				log.Warn("Unsubscribing hotkey " + actionName + " failed: " + ex);
			}
		}
	}
}
