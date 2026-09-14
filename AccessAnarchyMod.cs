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
			log.Info("Access Anarchy v0.7.0 loading (zero-Harmony ECS data editing)...");
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

			// 从 ModsSettings\AccessAnarchy.coc 读取玩家上次保存的设置（含改过的键位——
			// LoadSettings 必须先于 RegisterKeyBindings，玩家改的键才能在特性默认键之前恢复）。
			Colossal.IO.AssetDatabase.AssetDatabase.global.LoadSettings(nameof(AccessAnarchy), m_Setting, new Setting(this));

			// 注册快捷键（F3=启用/关闭，F4=切换避让范围）。官方路线：
			// RegisterKeyBindings 把 [SettingsUIKeyboardAction]/[SettingsUIKeyboardBinding]
			// 声明的动作挂进 InputManager（map = 模组 id），onInteraction 在按下沿触发。
			m_Setting.RegisterKeyBindings();
			ProxyAction toggleEnabled = m_Setting.GetAction(Setting.kToggleEnabledAction);
			toggleEnabled.shouldBeEnabled = true;
			toggleEnabled.onInteraction += OnToggleEnabledInteraction;
			ProxyAction toggleScope = m_Setting.GetAction(Setting.kToggleScopeAction);
			toggleScope.shouldBeEnabled = true;
			toggleScope.onInteraction += OnToggleScopeInteraction;
			log.Info("Hotkeys registered: F3=enable/disable, F4=scope mode");

			// 注册到模拟主循环，并强制排在 CarNavigationSystem 之前：
			// CarNavigationSystem 每帧读取 LaneOverlap / LaneObject 决定避让，我们删完它才读；
			// HumanNavigationSystem（在 CarNavigationSystem 之后，见 Game.Common.SystemOrder
			// 第 310/343 行）的重新注册要等下一帧才会被再次删掉，因此永远追不上。
			updateSystem.UpdateAt<AccessZoneOverlapSystem>(SystemUpdatePhase.GameSimulation);
			updateSystem.UpdateBefore<AccessZoneOverlapSystem, CarNavigationSystem>(SystemUpdatePhase.GameSimulation);

			log.Info("Access Anarchy loaded: AccessZoneOverlapSystem registered in GameSimulation before CarNavigationSystem.");
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
			if (m_Setting != null)
			{
				m_Setting.UnregisterInOptionsUI();
				m_Setting = null;
			}
			Setting.Instance = null;
		}
	}
}
