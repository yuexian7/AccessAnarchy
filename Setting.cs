using System;
using System.Collections.Generic;
using Colossal;
using Colossal.IO.AssetDatabase;
using Game.Input;
using Game.Modding;
using Game.SceneFlow;
using Game.Settings;
using Game.UI;
using Game.UI.Widgets;
using UnityEngine;

namespace AccessAnarchy
{
	/// <summary>
	/// 模组选项（官方 ModSetting 框架）。
	/// 落盘位置实测在存档同级目录 <c>...Cities Skylines II\AccessAnarchy.coc</c>
	/// （[FileLocation] 是文件名而不是子目录；ModsSettings\ 下是另一批按目录存放的模组数据）。
	/// 框架会在主线程直接调属性 setter，所以 getter 的返回值总是当前生效值；
	/// AccessZoneOverlapSystem 每次调度 Job 前直接读 Instance 的 getter，无需静态镜像。
	/// </summary>
	[FileLocation(nameof(AccessAnarchy))]
	[SettingsUIGroupOrder(kGroupMain, kGroupScope, kGroupAbout)]
	[SettingsUIShowGroupName(kGroupMain, kGroupScope, kGroupAbout)]
	// 两个快捷键动作（官方 InputManager 注册，usages 缺省 = DefaultSet 全集：菜单+游戏内都生效）。
	// 反编译核实：ModSetting.RegisterKeyBindings 按 [SettingsUIKeyboardBinding] 属性生成
	// ProxyBinding 并 InputManager.instance.AddActions；map = ModSetting.id（模组自己的 map）。
	[SettingsUIKeyboardAction(kToggleEnabledAction)]
	[SettingsUIKeyboardAction(kToggleScopeAction)]
	public class Setting : ModSetting
	{
		public const string kSection = "Main";

		public const string kGroupMain = "Main";
		public const string kGroupScope = "Scope";
		public const string kGroupAbout = "About";

		/// <summary>快捷键动作名：启用/关闭（默认 F3）。</summary>
		public const string kToggleEnabledAction = "ToggleEnabled";

		/// <summary>快捷键动作名：切换避让范围（默认 F4）。</summary>

		public const string kToggleScopeAction = "ToggleScope";

		/// <summary>作用模式：只处理建筑出入口区域的车道。</summary>
		public const int kModeAccessOnly = 0;

		/// <summary>作用模式：处理所有道路（全局禁用车辆-行人避让）。</summary>
		public const int kModeGlobal = 1;

		/// <summary>不避让范围三类之一：停车场（地面停车场与其出入口、停车楼/车库坡道）。
		/// 掩码位与 AccessZoneOverlapSystem.LaneCategory 的返回值一一对应。</summary>
		public const int kCatParking = 1;

		/// <summary>不避让范围三类之一：非停车场建筑与外部道路之间的车辆出入连接道。</summary>
		public const int kCatBuildingAccess = 2;

		/// <summary>不避让范围三类之一：非停车场建筑的内部道路（场区道路、自有车道）。</summary>
		public const int kCatBuildingInternal = 4;

		/// <summary>系统访问当前设置实例的入口；OnLoad 赋值，OnDispose 清空。</summary>
		public static Setting Instance;

		private bool m_Enabled = true;
		private int m_Mode = kModeAccessOnly;
		private bool m_IncludeParkingLots = true;
		private bool m_IncludeBuildingAccess = true;
		private bool m_IncludeBuildingInternal = true;

		public Setting(IMod mod) : base(mod)
		{
		}

		/// <summary>总开关。关闭后系统完全不运行，游戏自带的避让行为自然恢复。</summary>
		[SettingsUISection(kSection, kGroupMain)]
		public bool Enabled
		{
			get => m_Enabled;
			set => m_Enabled = value;
		}

		/// <summary>作用模式：0 = 仅建筑出入口区域（默认），1 = 全局所有道路。
		/// ValueVersion 给下拉框提供刷新信号：版本值（=当前语言）变化时框架才会重新调
		/// GetModeItems——DropdownField.Update() 只在 itemsVersion 变化时重取 items
		/// （Game.UI.Menu.AutomaticSettings L1268 / Game.UI.Widgets.DropdownField L62-74），
		/// 没有 ValueVersion，items 建页时取一次就永远不刷新，切语言后下拉框仍是旧语言。</summary>
		[SettingsUIDropdown(typeof(Setting), nameof(GetModeItems))]
		[SettingsUIValueVersion(typeof(Setting), nameof(GetModeItemsVersion))]
		[SettingsUISection(kSection, kGroupMain)]
		public int Mode
		{
			get => m_Mode;
			set => m_Mode = value;
		}

		/// <summary>停车场：地面停车场/停车楼本身的车道与其出入连接道，以及车库坡道（GarageLane）。
		/// 判据是车道所属建筑带 Game.Buildings.CarParkingFacility / ParkingFacility
		/// （ParkingLaneDataSystem.cs:405-418 证明商店/办公/工业这类建筑没有该组件，
		/// 停车场建筑才有），所以本项与下面两项互不重叠。</summary>
		[SettingsUISection(kSection, kGroupScope)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(IsGlobalMode))]
		// ForceSave 必需：带 DisableByCondition 的属性默认不进设置文件
		// （原版同一处证据 Game.Settings.GeneralSettings.cs:93-96）。
		[SettingsUIForceSave]
		public bool IncludeParkingLots
		{
			get => m_IncludeParkingLots;
			set => m_IncludeParkingLots = value;
		}

		/// <summary>建筑与外部道路之间的车辆出入连接道（ConnectionLane），不含停车场出入口。</summary>
		[SettingsUISection(kSection, kGroupScope)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(IsGlobalMode))]
		[SettingsUIForceSave]
		public bool IncludeBuildingAccess
		{
			get => m_IncludeBuildingAccess;
			set => m_IncludeBuildingAccess = value;
		}

		/// <summary>建筑内部道路（AreaLane 与建筑自有的 CarLane/ParkingLane），不含停车场内部。</summary>
		[SettingsUISection(kSection, kGroupScope)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(IsGlobalMode))]
		[SettingsUIForceSave]
		public bool IncludeBuildingInternal
		{
			get => m_IncludeBuildingInternal;
			set => m_IncludeBuildingInternal = value;
		}

		/// <summary>三类勾选合成的掩码。v0.8.0 之前这个值只喂给"车道类型"判定，
		/// 而锚点邻域判定（IsNearAnchor）用的是不分类型的锚点网格，勾掉任何一类都还是有锚点命中，
		/// 所以三个开关实测完全无效——现在锚点收集与邻域集合都按本掩码过滤。</summary>
		public int IncludeMask
		{
			get
			{
				int mask = 0;
				if (m_IncludeParkingLots)
				{
					mask |= kCatParking;
				}
				if (m_IncludeBuildingAccess)
				{
					mask |= kCatBuildingAccess;
				}
				if (m_IncludeBuildingInternal)
				{
					mask |= kCatBuildingInternal;
				}
				return mask;
			}
		}

		public bool IsGlobalMode()
		{
			return m_Mode == kModeGlobal;
		}

		/// <summary>启用/关闭快捷键（出厂默认 F3，玩家可改可清空）。
		/// 注意：模组键位**不在**游戏的 KeybindingSettings 里——那个设置类的 bindings 取值时
		/// 过滤了 BindingOptions.OnlyBuiltIn（KeybindingSettings.cs:18 / InputManager.cs:1199 一侧），
		/// 所以本模组的键位只存在自己的 AccessAnarchy.coc 里，加载与落盘全走 ModSetting 这条路。
		/// 顺序必须是 RegisterKeyBindings() 在前、LoadSettings() 在后（见 AccessAnarchyMod.OnLoad），
		/// 否则框架的 [AfterDecode] ApplyKeyBindings() 因 keyBindingRegistered 还是 false 而不执行，
		/// 玩家存的键（包括主动清空的空键）就拿不到落地机会。
		/// 选项页标签必须用 GetOptionLabelLocaleID(属性名)——GetBindingKeyLocaleID(actionName)
		/// 生成的是输入系统动作名（Options.OPTION[id/action/Press]），不是设置页那一行的标题。</summary>
		[SettingsUIKeyboardBinding(BindingKeyboard.F3, kToggleEnabledAction)]
		[SettingsUISection(kSection, kGroupMain)]
		public ProxyBinding ToggleEnabledBinding { get; set; }

		/// <summary>切换避让范围快捷键（默认 F4）。</summary>
		[SettingsUIKeyboardBinding(BindingKeyboard.F4, kToggleScopeAction)]
		[SettingsUISection(kSection, kGroupMain)]
		public ProxyBinding ToggleScopeBinding { get; set; }

		/// <summary>关于板块第一行：模组版本。只读 string 属性由框架渲染成只读行
		/// （AutomaticSettings.GetWidgetType：get-only string → WidgetType.StringField，
		/// 落到 Game.UI.Widgets.LocalizedValueField，setter 是空委托 → 玩家改不了）。
		/// 原版同款先例：Game.Settings.About 的 gameVersion/unityVersion 就是只读 string。</summary>
		[SettingsUISection(kSection, kGroupAbout)]
		public string Version => ModInfo.kVersion;

		/// <summary>关于板块第二行：作者。</summary>
		[SettingsUISection(kSection, kGroupAbout)]
		public string Author => ModInfo.kAuthor;

		/// <summary>关于板块第三行：三个跳转按钮之一。写法照抄官方模板与原版
		/// Game.Settings.ModdingSettings：只写不读的 bool 属性即渲染成按钮
		/// （AutomaticSettings.AddBoolButtonProperty 的守卫是 if (canRead || !canWrite) return null，
		/// 所以属性必须真的没有 getter，否则整行消失）；同 [SettingsUIButtonGroup] 的按钮合并成一行。
		/// 点击只执行 setter，框架不会顺带 ApplyAndSave，这里也不需要。</summary>
		[SettingsUISection(kSection, kGroupAbout)]
		[SettingsUIButtonGroup(ModInfo.kLinkGroup)]
		public bool OpenKoFi
		{
			set => OpenLink(ModInfo.kKoFiUrl, "ko-fi");
		}

		[SettingsUISection(kSection, kGroupAbout)]
		[SettingsUIButtonGroup(ModInfo.kLinkGroup)]
		public bool OpenForum
		{
			set => OpenLink(ModInfo.kForumUrl, "forum");
		}

		[SettingsUISection(kSection, kGroupAbout)]
		[SettingsUIButtonGroup(ModInfo.kLinkGroup)]
		public bool OpenRainbow
		{
			set => OpenLink(ModInfo.kRainbowUrl, "rainbow");
		}

		/// <summary>用系统默认浏览器打开链接。走的是游戏自己的做法：
		/// Game.UI.Menu.ParadoxBindings.ShowLink 里就是 Application.OpenURL(link)（ParadoxBindings.cs:708），
		/// UnityEngine.CoreModule 已在 csproj 引用。</summary>
		private static void OpenLink(string url, string tag)
		{
			try
			{
				Application.OpenURL(url);
				AccessAnarchyMod.log.Info($"Opened {tag} link: {url}");
			}
			catch (Exception ex)
			{
				AccessAnarchyMod.log.Warn($"OpenLink({tag}) failed: " + ex);
			}
		}

		/// <summary>诊断用（2026-09-25）：把两个热键"真正注册进 InputManager 的那条绑定"连游戏
		/// 判定的冲突对侧一起打出来。需要它是因为键位有三层：盘上的 m_Path、属性里的
		/// ProxyBinding、实际生效的 InputBinding。ModSetting.RegisterKeyBindings 先按
		/// [SettingsUIKeyboardBinding] 的默认键（F3/F4）建动作，LoadSettings 的
		/// [AfterDecode] ApplyKeyBindings 再调 InputManager.SetBindings 把存档值推下去
		/// （ModSetting.cs:270-277）；SetBindingImpl 允许空（compositeInstance.canBeEmpty 默认 true），
		/// TrySetMainBinding 写的是 binding.overridePath，而 Unity 的 effectivePath 是
		/// overridePath ?? path（不是 IsNullOrEmpty）→ 空串确实能清掉绑定（Unity.InputSystem.dll
		/// 反编译 InputBinding.cs:246）。哪一层没对齐，这行日志直接看出来。
		/// 判据：ProxyBinding.hasConflicts（:414）第一句就是 if (!isSet) return None，
		/// isSet = path 非空（:600）→ **空键位不可能让游戏弹「Key binding conflict detected」**。
		/// 若这里显示 (empty) 且 conflicts=None 而玩家仍看到提示，那张卡是别的模组的
		/// （卡片标题=该模组显示名，InputManager.cs:1512 SetModConflictNotification）。</summary>
		public static void ReportKeyBindings(string when)
		{
			Setting s = Instance;
			if (s == null)
			{
				return;
			}
			ReportAction(s, when, kToggleEnabledAction);
			ReportAction(s, when, kToggleScopeAction);
		}

		private static void ReportAction(Setting s, string when, string actionName)
		{
			ProxyAction action = s.GetAction(actionName);
			if (action == null)
			{
				AccessAnarchyMod.log.Warn($"Key binding check {when}: action {actionName} not registered");
				return;
			}
			foreach (ProxyBinding b in action.bindings)
			{
				string path = string.IsNullOrEmpty(b.path) ? "(empty)" : b.path;
				if (b.modifiers != null && b.modifiers.Count > 0)
				{
					List<string> mods = new List<string>();
					for (int i = 0; i < b.modifiers.Count; i++)
					{
						mods.Add(b.modifiers[i].m_Path);
					}
					path += " +" + string.Join(",", mods);
				}
				IList<ProxyBinding> vs = b.conflicts;
				string detail = "none";
				if (vs != null && vs.Count > 0)
				{
					List<string> names = new List<string>();
					for (int i = 0; i < vs.Count; i++)
					{
						ProxyBinding other = vs[i];
						names.Add($"{other.mapName}/{other.actionName}[{other.name}]={other.path}({(other.isBuiltIn ? "built-in" : "mod")})");
					}
					detail = string.Join(" ; ", names);
				}
				AccessAnarchyMod.log.Info($"Key binding check {when}: {actionName} device={b.device} path={path} actionEnabled={action.enabled} conflicts={b.hasConflicts} vs {detail}");
			}
		}

		/// <summary>热键入口：翻转总开关并落盘。只在主线程（输入事件）调用。</summary>
		public static void ToggleEnabledFromHotkey()
		{
			Setting s = Instance;
			if (s == null)
			{
				return;
			}
			s.Enabled = !s.Enabled;
			try
			{
				s.ApplyAndSave();
			}
			catch (Exception ex)
			{
				AccessAnarchyMod.log.Warn("ToggleEnabledFromHotkey save failed: " + ex);
			}
			AccessAnarchyMod.log.Info($"Hotkey: Enabled -> {s.Enabled}");
		}

		/// <summary>热键入口：在"仅出入口区域"与"全局"之间切换并落盘。</summary>
		public static void ToggleScopeFromHotkey()
		{
			Setting s = Instance;
			if (s == null)
			{
				return;
			}
			s.Mode = (s.Mode == kModeAccessOnly) ? kModeGlobal : kModeAccessOnly;
			try
			{
				s.ApplyAndSave();
			}
			catch (Exception ex)
			{
				AccessAnarchyMod.log.Warn("ToggleScopeFromHotkey save failed: " + ex);
			}
			AccessAnarchyMod.log.Info($"Hotkey: Mode -> {(s.Mode == kModeGlobal ? "global" : "access-zones")}");
		}

		public DropdownItem<int>[] GetModeItems()
		{
			// 下拉框选项名不走框架字典（int dropdown 无按值 API），每次重取都跟随当前游戏语言。
			// 框架通过 GetModeItemsVersion 决定何时重取（见 Mode 属性注释）。
			LocaleTable.SetActiveLocale(GameManager.instance.localizationManager.activeLocaleId);
			return new DropdownItem<int>[2]
			{
				new DropdownItem<int>
				{
					value = kModeAccessOnly,
					displayName = LocaleTable.T(LocaleTable.kModeAccess)
				},
				new DropdownItem<int>
				{
					value = kModeGlobal,
					displayName = LocaleTable.T(LocaleTable.kModeGlobal)
				}
			};
		}

		/// <summary>下拉框 items 的版本号 = 当前语言标识。语言切换 → 版本变化 → 框架重取 items。</summary>
		public int GetModeItemsVersion()
		{
			return GameManager.instance.localizationManager.activeLocaleId.GetHashCode();
		}

		public override void SetDefaults()
		{
			m_Enabled = true;
			m_Mode = kModeAccessOnly;
			m_IncludeParkingLots = true;
			m_IncludeBuildingAccess = true;
			m_IncludeBuildingInternal = true;
		}
	}

	/// <summary>
	/// 官方 12 种语言的全部文案。语言代码与游戏的 GetSupportedLocales() 一致：
	/// de-DE en-US es-ES fr-FR it-IT ja-JP ko-KR pl-PL pt-BR ru-RU zh-HANS zh-HANT。
	/// 下拉框选项名无法走框架本地化（int dropdown 无按值 API），由 T() 按当前语言返回。
	/// </summary>
	internal static class LocaleTable
	{
		// 下拉框词条 key
		public const string kModeAccess = "mode.access";
		public const string kModeGlobal = "mode.global";

		private static readonly string[] kLocales =
		{
			"de-DE", "en-US", "es-ES", "fr-FR", "it-IT", "ja-JP",
			"ko-KR", "pl-PL", "pt-BR", "ru-RU", "zh-HANS", "zh-HANT"
		};

		public static string[] Locales => kLocales;

		private static string _activeLocale = "en-US";

		public static void SetActiveLocale(string locale)
		{
			for (int i = 0; i < kLocales.Length; i++)
			{
				if (string.Equals(kLocales[i], locale, StringComparison.OrdinalIgnoreCase))
				{
					_activeLocale = kLocales[i];
					return;
				}
			}
			_activeLocale = "en-US";
		}

		/// <summary>给下拉框 displayName 用的按当前语言取词（框架选项描述走 IDictionarySource）。</summary>
		public static string T(string key)
		{
			Dictionary<string, string> table = Build(_activeLocale);
			if (table.TryGetValue(key, out string value))
			{
				return value;
			}
			return Build("en-US")[key];
		}

		public static Dictionary<string, string> Build(Setting setting, string locale)
		{
			// 必须用传入的 locale 取词——每份字典各填各的语言；
			// v0.2 曾误用全局 activeLocale，导致 12 份字典全被填成同一语言（切英文仍是中文）。
			Dictionary<string, string> d = Build(locale);
			return new Dictionary<string, string>
			{
				{ setting.GetSettingsLocaleID(), d["mod.name"] },
				{ setting.GetOptionTabLocaleID(Setting.kSection), d["tab"] },
				{ setting.GetOptionGroupLocaleID(Setting.kGroupMain), d["group.main"] },
				{ setting.GetOptionGroupLocaleID(Setting.kGroupScope), d["group.scope"] },
				{ setting.GetOptionGroupLocaleID(Setting.kGroupAbout), d["group.about"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.Enabled)), d["enabled.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.Enabled)), d["enabled.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.Mode)), d["mode.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.Mode)), d["mode.desc"] },
				{ setting.GetOptionWarningLocaleID(nameof(Setting.Mode)), d["mode.warning"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.IncludeParkingLots)), d["parking.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.IncludeParkingLots)), d["parking.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.IncludeBuildingAccess)), d["access.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.IncludeBuildingAccess)), d["access.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.IncludeBuildingInternal)), d["internal.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.IncludeBuildingInternal)), d["internal.desc"] },

				// 键位行标题/说明走属性名（与 BLG v1.0.3 同款）：框架设置页渲染
				// ProxyBinding 属性时用 GetOptionLabelLocaleID(nameof(属性))，不是
				// GetBindingKeyLocaleID(actionName)。后者只给输入系统动作名用。
				{ setting.GetOptionLabelLocaleID(nameof(Setting.ToggleEnabledBinding)), d["hotkey.enabled"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.ToggleEnabledBinding)), d["hotkey.enabled.desc"] },
				{ setting.GetOptionLabelLocaleID(nameof(Setting.ToggleScopeBinding)), d["hotkey.scope"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.ToggleScopeBinding)), d["hotkey.scope.desc"] },
				{ setting.GetBindingMapLocaleID(), d["bindingMap"] },

				// 「关于」板块：只读两行 + 一行三个跳转按钮。
				// 按钮的按钮面文字就是 GetOptionLabelLocaleID(属性名)（Game.UI.Widgets.Button
				// 只有一个 label 字段），描述文字对按钮行没有意义，所以只注册 label。
				{ setting.GetOptionLabelLocaleID(nameof(Setting.Version)), d["version.label"] },
				{ setting.GetOptionLabelLocaleID(nameof(Setting.Author)), d["author.label"] },
				{ setting.GetOptionLabelLocaleID(nameof(Setting.OpenKoFi)), d["link.kofi"] },
				{ setting.GetOptionLabelLocaleID(nameof(Setting.OpenForum)), d["link.forum"] },
				{ setting.GetOptionLabelLocaleID(nameof(Setting.OpenRainbow)), d["link.rainbow"] }
			};
		}

		private static Dictionary<string, string> Build(string locale)
		{
			switch (locale)
			{
				case "de-DE": return De();
				case "es-ES": return Es();
				case "fr-FR": return Fr();
				case "it-IT": return It();
				case "ja-JP": return Ja();
				case "ko-KR": return Ko();
				case "pl-PL": return Pl();
				case "pt-BR": return Pt();
				case "ru-RU": return Ru();
				case "zh-HANS": return ZhHans();
				case "zh-HANT": return ZhHant();
				default: return En();
			}
		}

		private static Dictionary<string, string> En()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "General" },
				{ "group.main", "Behavior" },
				{ "group.scope", "Non-yielding scope" },
				{ "enabled.label", "Vehicles pass through pedestrians" },
				{ "enabled.desc", "Vehicles no longer slow down or stop for pedestrians at building access points (parking lots, warehouses, stores...). Pedestrians keep walking normally. Turn off to fully restore vanilla behavior." },
				{ "mode.label", "Scope mode" },
				{ "mode.desc", "Access zones only: only building entrances, garage ramps and parking lots. All roads (global): vehicles never yield to pedestrians anywhere, crosswalks included." },
				{ "mode.warning", "Global mode disables all car-pedestrian yielding citywide." },
				{ kModeAccess, "Access zones only" },
				{ kModeGlobal, "All roads (global)" },
				{ "parking.label", "Parking lots and parking buildings" },
				{ "parking.desc", "Surface parking lots, parking buildings, multi-storey car parks and garage ramps, including their entrance and exit lanes. Parking lots are NOT covered by the option below." },
				{ "access.label", "Building-to-road access lanes" },
				{ "access.desc", "The connection lanes between shops, warehouses, offices and similar buildings and the road network. Parking lot entrances are handled by the option above, not by this one." },
				{ "internal.label", "Building-internal roads" },
				{ "internal.desc", "Yards and internal roads owned by buildings, other than parking lots." },
				{ "hotkey.enabled", "Enable/disable the mod" },
				{ "hotkey.enabled.desc", "Toggle the entire mod on or off without opening the options menu." },
				{ "hotkey.scope", "Switch scope mode" },
				{ "hotkey.scope.desc", "Switch between access zones only and all roads (global)." },
				{ "group.about", "About" },
				{ "version.label", "Mod version" },
				{ "author.label", "Author" },
				{ "link.kofi", "Buy me a coffee" },
				{ "link.forum", "Forum thread" },
				{ "link.rainbow", "RAINBOW website" },
				{ "bindingMap", "Access Anarchy key bindings" }
			};
		}

		private static Dictionary<string, string> ZhHans()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "出入口无碰撞" },
				{ "tab", "综合" },
				{ "group.main", "行为" },
				{ "group.scope", "不避让范围" },
				{ "enabled.label", "车辆穿过行人" },
				{ "enabled.desc", "车辆在停车场、仓库、商场等建筑出入口处不再因行人减速或停车，直接穿过行人；行人照常行走。关闭后完全恢复原版行为。" },
				{ "mode.label", "避让范围" },
				{ "mode.desc", "仅出入口区域：只处理建筑出入口、车库坡道与停车场。全部道路（全局）：车辆在任何地方都不再让行行人，包括人行横道。" },
				{ "mode.warning", "全局模式会禁用全市所有车辆对行人的避让。" },
				{ kModeAccess, "仅出入口区域" },
				{ kModeGlobal, "全部道路（全局）" },
				{ "parking.label", "停车场与停车楼" },
				{ "parking.desc", "地面停车场、停车楼、车库坡道，连同它们的出入口车道。停车场不算在下一条「建筑与外部道路的车辆出入口」里。" },
				{ "access.label", "建筑与外部道路的车辆出入口" },
				{ "access.desc", "商店、仓库、写字楼等建筑与城市道路之间的连接车道。停车场的出入口由上一条控制，不属于本条。" },
				{ "internal.label", "建筑内部道路" },
				{ "internal.desc", "建筑自有场区内的内部道路与车道，不含停车场内部。" },
				{ "hotkey.enabled", "启用/关闭模组" },
				{ "hotkey.enabled.desc", "无需打开设置选项，直接开关整个模组。" },
				{ "hotkey.scope", "切换避让范围" },
				{ "hotkey.scope.desc", "在「仅出入口区域」与「全部道路（全局）」之间切换。" },
				{ "group.about", "关于" },
				{ "version.label", "模组版本" },
				{ "author.label", "作者" },
				{ "link.kofi", "请我喝杯咖啡" },
				{ "link.forum", "论坛页面" },
				{ "link.rainbow", "RAINBOW官网" },
				{ "bindingMap", "Access Anarchy 快捷键" }
			};
		}

		private static Dictionary<string, string> ZhHant()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "出入口無碰撞" },
				{ "tab", "綜合" },
				{ "group.main", "行為" },
				{ "group.scope", "不讓避範圍" },
				{ "enabled.label", "車輛穿越行人" },
				{ "enabled.desc", "車輛在停車場、倉庫、商場等建築出入口處不再因行人減速或停車，直接穿越行人；行人照常行走。關閉後完全恢復原版行為。" },
				{ "mode.label", "避讓範圍" },
				{ "mode.desc", "僅出入口區域：只處理建築出入口、車庫坡道與停車場。全部道路（全域）：車輛在任何地方都不再讓行人，包括斑馬線。" },
				{ "mode.warning", "全域模式會停用全市所有車輛對行人的避讓。" },
				{ kModeAccess, "僅出入口區域" },
				{ kModeGlobal, "全部道路（全域）" },
				{ "parking.label", "停車場與立體停車場" },
				{ "parking.desc", "地面停車場、立體停車場、車庫坡道，連同它們的出入口車道。停車場不算在下條「建築與外部道路的車輛出入口」裡。" },
				{ "access.label", "建築與外部道路的車輛出入口" },
				{ "access.desc", "商店、倉庫、辦公大樓等建築與道路之間的連接車道。停車場出入口由上條控制，不屬於本條。" },
				{ "internal.label", "建築內部道路" },
				{ "internal.desc", "建築自有場區內的內部道路與車道，不含停車場內部。" },
				{ "hotkey.enabled", "啟用/關閉模組" },
				{ "hotkey.enabled.desc", "無需打開選項，直接開關整個模組。" },
				{ "hotkey.scope", "切換避讓範圍" },
				{ "hotkey.scope.desc", "在「僅出入口區域」與「全部道路（全域）」之間切換。" },
				{ "group.about", "關於" },
				{ "version.label", "模組版本" },
				{ "author.label", "作者" },
				{ "link.kofi", "請我喝杯咖啡" },
				{ "link.forum", "論壇頁面" },
				{ "link.rainbow", "RAINBOW官網" },
				{ "bindingMap", "Access Anarchy 捷徑" }
			};
		}

		private static Dictionary<string, string> De()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Keine Kollision an Ein- und Ausfahrten" },
				{ "tab", "Allgemein" },
				{ "group.main", "Verhalten" },
				{ "group.scope", "Bereich ohne Vorrang für Fußgänger" },
				{ "enabled.label", "Fahrzeuge durchqueren Fußgänger" },
				{ "enabled.desc", "Fahrzeuge bremsen an Gebäudezugängen (Parkplätze, Lagerhäuser, Geschäfte ...) nicht mehr für Fußgänger. Fußgänger laufen normal weiter. Ausschalten stellt das Originalverhalten wieder her." },
				{ "mode.label", "Geltungsbereich" },
				{ "mode.desc", "Nur Zugangsbereiche: nur Gebäudeeingänge, Garagenrampen und Parkplätze. Alle Straßen (global): Fahrzeuge halten nirgendwo mehr für Fußgänger, auch nicht an Zebrastreifen." },
				{ "mode.warning", "Der globale Modus deaktiviert das Fußgänger-Vorrangverhalten stadtweit." },
				{ kModeAccess, "Nur Zugangsbereiche" },
				{ kModeGlobal, "Alle Straßen (global)" },
				{ "parking.label", "Parkplätze und Parkhäuser" },
				{ "parking.desc", "Oberirdische Parkplätze, Parkhäuser, Tiefgaragen und Rampen – inklusive ihrer Zu- und Abfahrten. Parkplätze fallen nicht unter die nächste Option." },
				{ "access.label", "Gebäudezufahrten zur Straße" },
				{ "access.desc", "Verbindungsspuren zwischen Geschäften, Lagerhäusern, Büros und dem Straßennetz. Parkplätze werden von der oberen Option geregelt." },
				{ "internal.label", "Interne Straßen der Gebäude" },
				{ "internal.desc", "Hof- und interne Straßen auf eigenen Grundstücken von Gebäuden, ausgenommen Parkplätze." },
				{ "hotkey.enabled", "Mod aktivieren/deaktivieren" },
				{ "hotkey.enabled.desc", "Das gesamte Mod ohne das Menü Optionen ein- oder ausschalten." },
				{ "hotkey.scope", "Bereichsmodus wechseln" },
				{ "hotkey.scope.desc", "Zwischen nur Zugangsbereichen und allen Straßen (global) wechseln." },
				{ "group.about", "Über das Mod" },
				{ "version.label", "Mod-Version" },
				{ "author.label", "Autor" },
				{ "link.kofi", "Spendier mir einen Kaffee" },
				{ "link.forum", "Forum-Thread" },
				{ "link.rainbow", "RAINBOW-Webseite" },
				{ "bindingMap", "Kurzbefehle von Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Es()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Sin colisiones en entradas y salidas" },
				{ "tab", "General" },
				{ "group.main", "Comportamiento" },
				{ "group.scope", "Ámbito sin cesión a peatones" },
				{ "enabled.label", "Los vehículos atraviesan peatones" },
				{ "enabled.desc", "Los vehículos ya no frenan ni se detienen ante peatones en los accesos a edificios (aparcamientos, almacenes, tiendas...). Los peatones siguen caminando con normalidad. Desactiva la opción para restaurar el comportamiento original." },
				{ "mode.label", "Modo de alcance" },
				{ "mode.desc", "Solo accesos: solo entradas de edificios, rampas de garaje y aparcamientos. Todas las carreteras (global): los vehículos nunca ceden el paso a los peatones, incluidos los pasos de cebra." },
				{ "mode.warning", "El modo global desactiva la cesión a peatones en toda la ciudad." },
				{ kModeAccess, "Solo accesos" },
				{ kModeGlobal, "Todas las carreteras (global)" },
				{ "parking.label", "Aparcamientos y edificios de aparcamiento" },
				{ "parking.desc", "Aparcamientos en superficie, edificios de aparcamiento, garajes y sus rampas, incluidos los carriles de entrada y salida. Los aparcamientos NO los cubre la opción siguiente." },
				{ "access.label", "Carriles de acceso de edificios a la red viaria" },
				{ "access.desc", "Los carriles conectores entre tiendas, almacenes, oficinas y similares y la red de carreteras. Las entradas de aparcamiento las controla la opción anterior." },
				{ "internal.label", "Vías internas de los edificios" },
				{ "internal.desc", "Patios y vías internas propiedad de los edificios, excepto aparcamientos." },
				{ "hotkey.enabled", "Activar/desactivar el mod" },
				{ "hotkey.enabled.desc", "Activa o desactiva todo el mod sin abrir el menú de opciones." },
				{ "hotkey.scope", "Cambiar modo de alcance" },
				{ "hotkey.scope.desc", "Alterna entre solo accesos y todas las vías (global)." },
				{ "group.about", "Acerca del mod" },
				{ "version.label", "Versión del mod" },
				{ "author.label", "Autor" },
				{ "link.kofi", "Invítame a un café" },
				{ "link.forum", "Hilo del foro" },
				{ "link.rainbow", "Sitio web de RAINBOW" },
				{ "bindingMap", "Atajos de Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Fr()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Aucune collision aux entrées et sorties" },
				{ "tab", "Général" },
				{ "group.main", "Comportement" },
				{ "group.scope", "Périmètre sans priorité aux piétons" },
				{ "enabled.label", "Les véhicules traversent les piétons" },
				{ "enabled.desc", "Les véhicules ne ralentissent plus ni ne s'arrêtent devant les piétons aux accès de bâtiments (parkings, entrepôts, magasins...). Les piétons continuent de marcher normalement. Désactivez pour restaurer le comportement d'origine." },
				{ "mode.label", "Mode de portée" },
				{ "mode.desc", "Accès uniquement : uniquement les entrées de bâtiments, rampes de parking et parkings. Toutes les routes (global) : les véhicules ne cèdent plus le passage aux piétons nulle part, passages piétons compris." },
				{ "mode.warning", "Le mode global désactive la priorité aux piétons dans toute la ville." },
				{ kModeAccess, "Accès uniquement" },
				{ kModeGlobal, "Toutes les routes (global)" },
				{ "parking.label", "Parkings et parkings couverts" },
				{ "parking.desc", "Parcs de stationnement en surface, parkings couverts, garages et leurs rampes, y compris les voies d'entrée et de sortie. Les parkings ne relèvent pas de l'option suivante." },
				{ "access.label", "Voies d'accès bâtiment-route" },
				{ "access.desc", "Les voies de raccordement entre commerces, entrepôts, bureaux et le réseau routier. Les accès des parkings relèvent de l'option précédente." },
				{ "internal.label", "Voies internes des bâtiments" },
				{ "internal.desc", "Cours et voies internes appartenant aux bâtiments, hors parkings." },
				{ "hotkey.enabled", "Activer/désactiver le mod" },
				{ "hotkey.enabled.desc", "Active ou désactive tout le mod sans ouvrir le menu des options." },
				{ "hotkey.scope", "Changer le mode de portée" },
				{ "hotkey.scope.desc", "Bascule entre accès uniquement et toutes les voies (global)." },
				{ "group.about", "À propos" },
				{ "version.label", "Version du mod" },
				{ "author.label", "Auteur" },
				{ "link.kofi", "Offrez-moi un café" },
				{ "link.forum", "Sujet du forum" },
				{ "link.rainbow", "Site web RAINBOW" },
				{ "bindingMap", "Raccourcis d'Access Anarchy" }
			};
		}

		private static Dictionary<string, string> It()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Nessuna collisione a ingressi e uscite" },
				{ "tab", "Generale" },
				{ "group.main", "Comportamento" },
				{ "group.scope", "Ambito senza precedenza ai pedoni" },
				{ "enabled.label", "I veicoli attraversano i pedoni" },
				{ "enabled.desc", "I veicoli non rallentano più né si fermano per i pedoni agli accessi degli edifici (parcheggi, magazzini, negozi...). I pedoni continuano a camminare normalmente. Disattiva per ripristinare il comportamento originale." },
				{ "mode.label", "Modalità di ambito" },
				{ "mode.desc", "Solo accessi: solo ingressi degli edifici, rampe dei garage e parcheggi. Tutte le strade (globale): i veicoli non danno più precedenza ai pedoni da nessuna parte, strisce pedonali incluse." },
				{ "mode.warning", "La modalità globale disattiva la precedenza ai pedoni in tutta la città." },
				{ kModeAccess, "Solo accessi" },
				{ kModeGlobal, "Tutte le strade (globale)" },
				{ "parking.label", "Parcheggi e strutture di parcheggio" },
				{ "parking.desc", "Parcheggi in superficie, strutture di parcheggio, garage e relative rampe, corsie di ingresso e uscita incluse. I parcheggi NON rientrano nell'opzione successiva." },
				{ "access.label", "Corsie di accesso edificio-strada" },
				{ "access.desc", "Le corsie di collegamento tra negozi, magazzini, uffici e la rete stradale. Gli ingressi dei parcheggi dipendono dall'opzione precedente." },
				{ "internal.label", "Strade interne degli edifici" },
				{ "internal.desc", "Aie e strade interne di proprietà degli edifici, esclusi i parcheggi." },
				{ "hotkey.enabled", "Attiva/disattiva la mod" },
				{ "hotkey.enabled.desc", "Attiva o disattiva l'intera mod senza aprire il menu delle opzioni." },
				{ "hotkey.scope", "Cambia modalità di ambito" },
				{ "hotkey.scope.desc", "Alterna tra solo accessi e tutte le strade (globale)." },
				{ "group.about", "Informazioni" },
				{ "version.label", "Versione della mod" },
				{ "author.label", "Autore" },
				{ "link.kofi", "Offrimi un caffè" },
				{ "link.forum", "Thread sul forum" },
				{ "link.rainbow", "Sito web RAINBOW" },
				{ "bindingMap", "Scorciatoie di Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Ja()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "出入口は衝突しない" },
				{ "tab", "一般" },
				{ "group.main", "動作" },
				{ "group.scope", "歩行者を優先しない範囲" },
				{ "enabled.label", "車両が歩行者を避けない" },
				{ "enabled.desc", "建物の出入口（駐車場・倉庫・店舗など）で、車両が歩行者に合わせて減速・停止しなくなり、そのまま通り抜けます。歩行者は普段どおり歩き続けます。オフにすると標準の動作に戻ります。" },
				{ "mode.label", "適用範囲" },
				{ "mode.desc", "アクセスのみ：建物の出入口・ガレージランプ・駐車場のみ。すべての道路（グローバル）：横断歩道を含め、車両はどこでも歩行者に道を譲らなくなります。" },
				{ "mode.warning", "グローバルモードでは、市内全体で車両が歩行者に道を譲らなくなります。" },
				{ kModeAccess, "アクセスのみ" },
				{ kModeGlobal, "すべての道路（グローバル）" },
				{ "parking.label", "駐車場・立体駐車場" },
				{ "parking.desc", "地上駐車場、立体駐車場、ガレージのスロープを、出入口のレーンも含めて対象にします。駐車場は次の項目には含まれません。" },
				{ "access.label", "建物と道路の車両出入口" },
				{ "access.desc", "店舗・倉庫・オフィスなどを道路網に接続する接続レーン（車両用）。駐車場の出入口は上の項目が担当します。" },
				{ "internal.label", "建物内の内部道路" },
				{ "internal.desc", "建物が所有する構内道路・敷地内レーン（駐車場は除く）。" },
				{ "hotkey.enabled", "Modの有効/無効" },
				{ "hotkey.enabled.desc", "オプションメニューを開かずにMod全体をオン/オフします。" },
				{ "hotkey.scope", "適用範囲の切り替え" },
				{ "hotkey.scope.desc", "「アクセスのみ」と「すべての道路（グローバル）」を切り替えます。" },
				{ "group.about", "このModについて" },
				{ "version.label", "Modバージョン" },
				{ "author.label", "作者" },
				{ "link.kofi", "コーヒーをおごる" },
				{ "link.forum", "フォーラムスレッド" },
				{ "link.rainbow", "RAINBOW公式サイト" },
				{ "bindingMap", "Access Anarchy のショートカット" }
			};
		}

		private static Dictionary<string, string> Ko()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "출입구 충돌 없음" },
				{ "tab", "일반" },
				{ "group.main", "동작" },
				{ "group.scope", "양보 제외 범위" },
				{ "enabled.label", "차량이 보행자를 통과" },
				{ "enabled.desc", "건물 출입구(주차장, 창고, 상점 등)에서 차량이 보행자 때문에 감속하거나 정지하지 않고 그대로 통과합니다. 보행자는 평소대로 걷습니다. 끄면 원래 동작으로 돌아갑니다." },
				{ "mode.label", "적용 범위" },
				{ "mode.desc", "출입 구역만: 건물 출입구, 주차장 경사로, 주차장 내부만 적용. 모든 도로(전체): 횡단보도를 포함해 차량이 어디서도 보행자에게 양보하지 않습니다." },
				{ "mode.warning", "전체 모드에서는 도시 전체에서 차량의 보행자 양보가 비활성화됩니다." },
				{ kModeAccess, "출입 구역만" },
				{ kModeGlobal, "모든 도로(전체)" },
				{ "parking.label", "주차장과 주차 건물" },
				{ "parking.desc", "지상 주차장, 주차 건물, 지하 주차장 경사로를 출입 차로까지 포함해 대상에 넣습니다. 주차장은 다음 항목에 포함되지 않습니다." },
				{ "access.label", "건물과 도로의 차량 출입구" },
				{ "access.desc", "상점·창고·사무실 등을 도로망에 연결하는 연결 차로(차량용). 주차장 출입구는 위 항목이 담당합니다." },
				{ "internal.label", "건물 내부 도로" },
				{ "internal.desc", "건물이 소유한 부지 내부 도로와 차로 (주차장 제외)." },
				{ "hotkey.enabled", "모드 켜기/끄기" },
				{ "hotkey.enabled.desc", "옵션 메뉴를 열지 않고 모드 전체를 켜거나 끕니다." },
				{ "hotkey.scope", "적용 범위 전환" },
				{ "hotkey.scope.desc", "출입 구역만과 모든 도로(전역) 사이를 전환합니다." },
				{ "group.about", "모드 정보" },
				{ "version.label", "모드 버전" },
				{ "author.label", "작성자" },
				{ "link.kofi", "커피 한 잔 쏘기" },
				{ "link.forum", "포럼 게시물" },
				{ "link.rainbow", "RAINBOW 공식 사이트" },
				{ "bindingMap", "Access Anarchy 단축키" }
			};
		}

		private static Dictionary<string, string> Pl()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Bez kolizji na wjeździe i wyjeździe" },
				{ "tab", "Ogólne" },
				{ "group.main", "Zachowanie" },
				{ "group.scope", "Zakres bez ustępowania pieszym" },
				{ "enabled.label", "Pojazdy przenikają przez pieszych" },
				{ "enabled.desc", "Pojazdy nie zwalniają już i nie zatrzymują się przed pieszymi przy wjazdach do budynków (parkingi, magazyny, sklepy...). Piesi chodzą normalnie. Wyłącz, aby przywrócić zachowanie z gry." },
				{ "mode.label", "Zakres działania" },
				{ "mode.desc", "Tylko strefy dostępu: tylko wjazdy do budynków, rampy garaży i parkingi. Wszystkie drogi (globalnie): pojazdy nigdzie nie ustępują pierwszeństwa pieszym, także na przejściach." },
				{ "mode.warning", "Tryb globalny wyłącza ustępowanie pierwszeństwa pieszym w całym mieście." },
				{ kModeAccess, "Tylko strefy dostępu" },
				{ kModeGlobal, "Wszystkie drogi (globalnie)" },
				{ "parking.label", "Parkingi i budynki parkingowe" },
				{ "parking.desc", "Parkingi naziemne, budynki parkingowe, garaże i ich rampy, wraz z pasami wjazdu i wyjazdu. Parkingi NIE należą do następnej opcji." },
				{ "access.label", "Pasy dojazdowe budynków do dróg" },
				{ "access.desc", "Drogi łączące sklepy, magazyny, biura i podobne obiekty z siecią dróg. Wjazdy na parkingi reguluje opcja powyżej." },
				{ "internal.label", "Drogi wewnętrzne budynków" },
				{ "internal.desc", "Dziedzińce i drogi wewnętrzne należące do budynków, z wyjątkiem parkingów." },
				{ "hotkey.enabled", "Włącz/wyłącz mod" },
				{ "hotkey.enabled.desc", "Włącz lub wyłącz cały mod bez otwierania menu opcji." },
				{ "hotkey.scope", "Przełącz tryb zakresu" },
				{ "hotkey.scope.desc", "Przełącza między tylko strefami dostępu a wszystkimi drogami (globalnie)." },
				{ "group.about", "O modzie" },
				{ "version.label", "Wersja moda" },
				{ "author.label", "Autor" },
				{ "link.kofi", "Postaw mi kawę" },
				{ "link.forum", "Wątek na forum" },
				{ "link.rainbow", "Strona RAINBOW" },
				{ "bindingMap", "Skróty Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Pt()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Sem colisões nas entradas e saídas" },
				{ "tab", "Geral" },
				{ "group.main", "Comportamento" },
				{ "group.scope", "Escopo sem preferência a pedestres" },
				{ "enabled.label", "Veículos atravessam pedestres" },
				{ "enabled.desc", "Veículos não reduzem mais a velocidade nem param para pedestres nos acessos de edifícios (estacionamentos, armazéns, lojas...). Os pedestres continuam caminhando normalmente. Desligue para restaurar o comportamento original." },
				{ "mode.label", "Modo de escopo" },
				{ "mode.desc", "Somente acessos: apenas entradas de edifícios, rampas de garagem e estacionamentos. Todas as vias (global): veículos nunca dão preferência a pedestres, incluindo faixas de pedestres." },
				{ "mode.warning", "O modo global desativa a preferência a pedestres em toda a cidade." },
				{ kModeAccess, "Somente acessos" },
				{ kModeGlobal, "Todas as vias (global)" },
				{ "parking.label", "Estacionamentos e edifícios de estacionamento" },
				{ "parking.desc", "Estacionamentos ao ar livre, edifícios de estacionamento, garagens e suas rampas, incluindo as faixas de entrada e saída. Estacionamentos NÃO são cobertos pela opção seguinte." },
				{ "access.label", "Faixas de acesso dos edifícios às vias" },
				{ "access.desc", "As vias de conexão entre lojas, armazéns, escritórios e a malha viária. Entradas de estacionamento são controladas pela opção acima." },
				{ "internal.label", "Vias internas dos edifícios" },
				{ "internal.desc", "Pátios e vias internas pertencentes aos edifícios, exceto estacionamentos." },
				{ "hotkey.enabled", "Ativar/desativar o mod" },
				{ "hotkey.enabled.desc", "Ativa ou desativa todo o mod sem abrir o menu de opções." },
				{ "hotkey.scope", "Alternar modo de escopo" },
				{ "hotkey.scope.desc", "Alterna entre somente acessos e todas as vias (global)." },
				{ "group.about", "Sobre o mod" },
				{ "version.label", "Versão do mod" },
				{ "author.label", "Autor" },
				{ "link.kofi", "Me pague um café" },
				{ "link.forum", "Tópico no fórum" },
				{ "link.rainbow", "Site do RAINBOW" },
				{ "bindingMap", "Atalhos do Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Ru()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Без столкновений на въезде и выезде" },
				{ "tab", "Общее" },
				{ "group.main", "Поведение" },
				{ "group.scope", "Область без уступки пешеходам" },
				{ "enabled.label", "Транспорт проходит сквозь пешеходов" },
				{ "enabled.desc", "Транспорт больше не замедляется и не останавливается перед пешеходами у въездов на объекты (парковки, склады, магазины...). Пешеходы ходят как обычно. Выключите, чтобы вернуть исходное поведение." },
				{ "mode.label", "Режим действия" },
				{ "mode.desc", "Только зоны въездов: только въезды на объекты, рампы паркингов и парковки. Все дороги (глобально): транспорт нигде не уступает дорогу пешеходам, включая пешеходные переходы." },
				{ "mode.warning", "Глобальный режим отключает пропуск пешеходов по всему городу." },
				{ kModeAccess, "Только зоны въездов" },
				{ kModeGlobal, "Все дороги (глобально)" },
				{ "parking.label", "Парковки и многоэтажные паркинги" },
				{ "parking.desc", "Наземные парковки, многоэтажные паркинги, гаражи и их рампы, включая полосы въезда и выезда. Парковки не относятся к следующей опции." },
				{ "access.label", "Полосы въезда от зданий к дорогам" },
				{ "access.desc", "Соединительные полосы между магазинами, складами, офисами и дорожной сетью. Въезды на парковки регулируются опцией выше." },
				{ "internal.label", "Внутренние дороги территории зданий" },
				{ "internal.desc", "Внутренние и дворовые полосы, принадлежащие зданиям, кроме парковок." },
				{ "hotkey.enabled", "Включить/выключить мод" },
				{ "hotkey.enabled.desc", "Включает или выключает весь мод без открытия меню параметров." },
				{ "hotkey.scope", "Переключить режим зоны" },
				{ "hotkey.scope.desc", "Переключает между зонами въездов и всеми дорогами (глобально)." },
				{ "group.about", "О моде" },
				{ "version.label", "Версия мода" },
				{ "author.label", "Автор" },
				{ "link.kofi", "Угостите меня кофе" },
				{ "link.forum", "Тема на форуме" },
				{ "link.rainbow", "Сайт RAINBOW" },
				{ "bindingMap", "Ярлыки Access Anarchy" }
			};
		}
	}

	/// <summary>
	/// 本地化字典源：把 LocaleTable 的词条按 ModSetting 的 locale id 映射后交给框架。
	/// 12 种官方语言各注册一份（Mod.OnLoad）。
	/// </summary>
	internal class LocaleSource : IDictionarySource
	{
		private readonly Setting m_Setting;
		private readonly Dictionary<string, string> m_Entries;

		public LocaleSource(Setting setting, string locale)
		{
			m_Setting = setting;
			m_Entries = LocaleTable.Build(setting, locale);
		}

		public IEnumerable<KeyValuePair<string, string>> ReadEntries(IList<IDictionaryEntryError> errors, Dictionary<string, int> indexCounts)
		{
			return m_Entries;
		}

		public void Unload()
		{
		}
	}
}
