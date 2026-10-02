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
	[SettingsUIGroupOrder(kGroupMain, kGroupCustomMark, kGroupShortcut, kGroupAbout)]
	[SettingsUIShowGroupName(kGroupMain, kGroupCustomMark, kGroupShortcut, kGroupAbout)]
	// 两个快捷键动作。usages 缺省 = BuiltInUsages.DefaultSet(0x41E) = DefaultTool|Overlay|Tool|
	// CancelableTool|DiscardableTool —— 注意它**不含 Menu / Options**，所以是"游戏内各工具上下文生效"，
	// 不是"菜单+游戏内都生效"（SettingsUIInputActionAttribute.cs:45-50 与 Usages.cs:72）。
	// 也正因为与原版 in-game 动作的 usages 有交集，键位撞原版时游戏会判冲突并把我们的动作禁掉（§⑤）。
	// 反编译核实：ModSetting.RegisterKeyBindings 按 [SettingsUIKeyboardBinding] 属性生成
	// ProxyBinding 并 InputManager.instance.AddActions；map = ModSetting.id（模组自己的 map）。
	[SettingsUIKeyboardAction(kToggleEnabledAction)]
	[SettingsUIKeyboardAction(kToggleScopeAction)]
	public class Setting : ModSetting
	{
		public const string kSection = "Main";

		public const string kGroupMain = "Main";

		/// <summary>「自定义标记」板块（本版只有三条灰置占位，功能在 v0.9.0）。位置写在
		/// 类头 [SettingsUIGroupOrder] 的下标里：行为 → 自定义标记 → 快捷键设置 → 关于。
		/// v0.8.3 之前这里下面是另一个 kGroupScope「出入口区域选择」板块放三个类别勾选，已被整体删除：
		/// 两轮实机（§⑨2 与 §⑪）证明"停车场 / 建筑出入口 / 建筑内部道路"在 CS2 的数据里
		/// 无法按车道稳定分开（出入口大门道多为无 LaneOverlap 的 ConnectionLane，邻近兜底又会跨类互相覆盖），
		/// 作者 2026-10-01 决定三合一，只剩总开关与取消避让范围。</summary>
		public const string kGroupCustomMark = "CustomMark";

		/// <summary>v0.8.1：两个键位从「行为」里拆出来单独成组，排在范围板块下面
		/// （v0.8.2 起那个板块叫「出入口区域选择」）。
		/// v0.8.2：组名不再复用 d["bindingMap"]（官方 Options.INPUT_MAP[Shortcuts] 只有"快捷键"
		/// 没有"设置"），改用自己的 d["group.shortcut"] 条目，简中＝快捷键设置。</summary>
		public const string kGroupShortcut = "Shortcut";

		public const string kGroupAbout = "About";

		/// <summary>快捷键动作名：启用/关闭（出厂默认不绑定）。</summary>
		public const string kToggleEnabledAction = "ToggleEnabled";

		/// <summary>快捷键动作名：切换取消避让范围（出厂默认不绑定）。</summary>

		public const string kToggleScopeAction = "ToggleScope";

		/// <summary>作用模式：只处理建筑出入口区域的车道。</summary>
		public const int kModeAccessOnly = 0;

		/// <summary>作用模式：处理所有道路（全局禁用车辆-行人避让）。</summary>
		public const int kModeGlobal = 1;

		/// <summary>系统访问当前设置实例的入口；OnLoad 赋值，OnDispose 清空。</summary>
		public static Setting Instance;

		private bool m_Enabled = true;
		private int m_Mode = kModeAccessOnly;

		// v0.9.0 才实现的「自定义标记」三项：本版只挂出板块与灰置条目占位。
		private bool m_MarkBuildings;
		private bool m_MarkCrosswalks;
		private bool m_MarkIntersections;

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

		/// <summary>「自定义标记」三项之一（v0.9.0 计划）：点击建筑或设施把它设为无碰撞模式。
		/// 本版只挂出条目：恒灰不可点（DisableByCondition 条件方法永远返回"不可用"），
		/// 不加 [SettingsUIForceSave] ⇒ 不写进 AccessAnarchy.coc，将来实现时不会被旧值绊住。
		/// 注意带 DisableByCondition 的属性框架默认不落盘（原版同证据 Game.Settings.GeneralSettings.cs:93-96），
		/// 这里正是我们要的。</summary>
		[SettingsUISection(kSection, kGroupCustomMark)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(CustomMarkUnavailable))]
		public bool MarkBuildings
		{
			get => m_MarkBuildings;
			set => m_MarkBuildings = value;
		}

		/// <summary>「自定义标记」之二（v0.9.0 计划）：点击某条人行横道把它设为无碰撞模式。</summary>
		[SettingsUISection(kSection, kGroupCustomMark)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(CustomMarkUnavailable))]
		public bool MarkCrosswalks
		{
			get => m_MarkCrosswalks;
			set => m_MarkCrosswalks = value;
		}

		/// <summary>「自定义标记」之三（v0.9.0 计划）：点击某个路口，把该路口所有人行横道设为无碰撞模式。</summary>
		[SettingsUISection(kSection, kGroupCustomMark)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(CustomMarkUnavailable))]
		public bool MarkIntersections
		{
			get => m_MarkIntersections;
			set => m_MarkIntersections = value;
		}

		/// <summary>「自定义标记」整块暂未上线：条件方法恒为真 ⇒ 三条目全部标灰。
		/// 0.9.0 实现点击拾取（建筑/人行横道/路口）与按标记持久剥离 LaneOverlap 后，
		/// 把这个方法改成返回 false 即可解锁，属性与文案都不用再动。</summary>
		public bool CustomMarkUnavailable()
		{
			return true;
		}

		/// <summary>启用/关闭快捷键。出厂默认**不绑定**（BindingKeyboard.None → control 空串，见
		/// SettingsUIKeyboardBindingAttribute.cs:19），首次订阅的玩家两个热键都是空的，要自己在
		/// 「快捷键设置」板块里按一个键；玩家设过之后就一直按玩家存的走，外部模组
		/// （SIMPLE MOD CHECKER PLUS 等）在 OnLoad 之后还原备份时后写的赢。
		/// 注意：模组键位**不在**游戏的 KeybindingSettings 里——那个设置类的 bindings 取值时
		/// 过滤了 BindingOptions.OnlyBuiltIn（KeybindingSettings.cs:18 / InputManager.cs:1199 一侧），
		/// 所以本模组的键位只存在自己的 AccessAnarchy.coc 里，加载与落盘全走 ModSetting 这条路。
		/// 顺序必须是 RegisterKeyBindings() 在前、LoadSettings() 在后（见 AccessAnarchyMod.OnLoad），
		/// 否则框架的 [AfterDecode] ApplyKeyBindings() 因 keyBindingRegistered 还是 false 而不执行，
		/// 玩家存的键（包括主动清空的空键）就拿不到落地机会。
		/// 选项页标签必须用 GetOptionLabelLocaleID(属性名)——GetBindingKeyLocaleID(actionName)
		/// 生成的是输入系统动作名（Options.OPTION[id/action/Press]），不是设置页那一行的标题。</summary>
		[SettingsUIKeyboardBinding(BindingKeyboard.None, kToggleEnabledAction)]
		[SettingsUISection(kSection, kGroupShortcut)]
		public ProxyBinding ToggleEnabledBinding { get; set; }

		/// <summary>切换取消避让范围快捷键（出厂默认同样不绑定）。</summary>
		[SettingsUIKeyboardBinding(BindingKeyboard.None, kToggleScopeAction)]
		[SettingsUISection(kSection, kGroupShortcut)]
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
		/// [SettingsUIKeyboardBinding] 声明的默认键（本模组＝None，即空）建动作，LoadSettings 的
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
			// 卸载安全（v0.8.5）：GetAction 里面是 `InputManager.instance.actions`（ModSetting.cs 直接解引用），
			// 关机序列里 instance 可能已经是 null。这条诊断只是读键位，没键位可读就整段跳过，
			// 不能让一个 NRE 顺着 OnUpdate 把"模组已卸载但系统还在被 tick"变成一条玩家可见的报错。
			if (InputManager.instance == null)
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
			m_MarkBuildings = false;
			m_MarkCrosswalks = false;
			m_MarkIntersections = false;
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
				// v0.8.3 占位板块：作者要求板块名带上"（暂未上线）"，条目本身标灰不可选。
				{ setting.GetOptionGroupLocaleID(Setting.kGroupCustomMark), d["group.custommark"] },
				// v0.8.2：快捷键板块的组名不再复用 bindingMap，改用自己的条目（作者要求叫「快捷键设置」，
				// 官方那词只是"快捷键/Shortcuts"，没有"设置"二字）。
				{ setting.GetOptionGroupLocaleID(Setting.kGroupShortcut), d["group.shortcut"] },
				{ setting.GetOptionGroupLocaleID(Setting.kGroupAbout), d["group.about"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.Enabled)), d["enabled.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.Enabled)), d["enabled.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.Mode)), d["mode.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.Mode)), d["mode.desc"] },
				{ setting.GetOptionWarningLocaleID(nameof(Setting.Mode)), d["mode.warning"] },

				// v0.8.3：「出入口区域选择」板块连同三个勾选项整体删除（见 kGroupCustomMark 的注释），
				// 这里的 parking/access/internal 六条映射与 12 份字典里的同名条目一起清掉。
				{ setting.GetOptionLabelLocaleID(nameof(Setting.MarkBuildings)), d["mark.building.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.MarkBuildings)), d["mark.building.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.MarkCrosswalks)), d["mark.crosswalk.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.MarkCrosswalks)), d["mark.crosswalk.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.MarkIntersections)), d["mark.intersection.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.MarkIntersections)), d["mark.intersection.desc"] },

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
				{ "group.custommark", "Custom Marks (Not available yet)" },
				{ "group.shortcut", "Shortcut Settings" },
				{ "enabled.label", "Vehicles pass through pedestrians" },
				{ "enabled.desc", "Vehicles no longer slow down or stop for pedestrians at building access points (parking lots, warehouses, stores...). Pedestrians keep walking normally. Turn off to fully restore vanilla behavior. Changes can take a few seconds to apply fully, longer in larger cities. Avoid changing the settings repeatedly in a short time while playing; results can be confusing. Warning: if road speed limits are set too high, the vehicle AI breaks down and stops working." },
				{ "mode.label", "Non-yielding scope" },
				{ "mode.desc", "Access zones only: building entrances and exits, garage ramps, parking lots and roads inside buildings. All roads (global): vehicles never yield to pedestrians anywhere, crosswalks included." },
				{ "mode.warning", "Global mode disables all car-pedestrian yielding citywide." },
				{ kModeAccess, "Access zones only" },
				{ kModeGlobal, "All roads (global)" },
				{ "mark.building.label", "Buildings and facilities" },
				{ "mark.building.desc", "When enabled, click a building or facility to set that building or facility to anarchy mode, regardless of the master switch and the non-yielding scope setting." },
				{ "mark.crosswalk.label", "Crosswalks" },
				{ "mark.crosswalk.desc", "When enabled, click a crosswalk to set that crosswalk to anarchy mode, regardless of the master switch and the non-yielding scope setting." },
				{ "mark.intersection.label", "Intersections" },
				{ "mark.intersection.desc", "When enabled, click an intersection to set all crosswalks at that intersection to anarchy mode, regardless of the master switch and the non-yielding scope setting." },
				{ "hotkey.enabled", "Enable/disable the mod" },
				{ "hotkey.enabled.desc", "Toggle the entire mod on or off without opening the options menu. Note: mod hotkeys take priority. To avoid conflicts, do not use the same keys as other mods." },
				{ "hotkey.scope", "Switch non-yielding scope" },
				{ "hotkey.scope.desc", "Switch between access zones only and all roads (global). Note: mod hotkeys take priority. To avoid conflicts, do not use the same keys as other mods." },
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
				{ "group.custommark", "自定义标记（暂未上线）" },
				{ "group.shortcut", "快捷键设置" },
				{ "enabled.label", "车辆穿过行人" },
				{ "enabled.desc", "车辆在停车场、仓库、商场等建筑出入口处不再因行人减速或停车，直接穿过行人；行人照常行走。关闭后完全恢复原版行为。 调整后需要几秒才完全生效，城市越大越慢。请勿游戏内短时间频繁修改设置，容易产生混乱。注意：如果道路限速设置过高，车辆AI系统会崩溃失效。" },
				{ "mode.label", "取消避让范围" },
				{ "mode.desc", "仅出入口区域：只处理建筑出入口、车库坡道、停车场与建筑内部道路。全部道路（全局）：车辆在任何地方都不再让行行人，包括人行横道。" },
				{ "mode.warning", "全局模式会禁用全市所有车辆对行人的避让。" },
				{ kModeAccess, "仅出入口区域" },
				{ kModeGlobal, "全部道路（全局）" },
				{ "mark.building.label", "建筑和设施" },
				{ "mark.building.desc", "开启后玩家可以点击某个建筑或设施，即可设置该建筑或设施为无碰撞模式，并且不受总开关和取消避让范围设置项的限制。" },
				{ "mark.crosswalk.label", "人行横道" },
				{ "mark.crosswalk.desc", "开启后玩家可以点击某条人行横道，即可设置该人行横道为无碰撞模式，并且不受总开关和取消避让范围设置项的限制。" },
				{ "mark.intersection.label", "路口" },
				{ "mark.intersection.desc", "开启后玩家可以点击某个路口，即可设置该路口所有人行横道为无碰撞模式，并且不受总开关和取消避让范围设置项的限制。" },
				{ "hotkey.enabled", "启用/关闭模组" },
				{ "hotkey.enabled.desc", "无需打开设置选项，直接开关整个模组。注意：模组快捷键优先，建议不要与其它模组重复。" },
				{ "hotkey.scope", "切换取消避让范围" },
				{ "hotkey.scope.desc", "在「仅出入口区域」与「全部道路（全局）」之间切换。注意：模组快捷键优先，建议不要与其它模组重复。" },
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
				{ "group.custommark", "自訂標記（暫未上線）" },
				{ "group.shortcut", "捷徑設定" },
				{ "enabled.label", "車輛穿越行人" },
				{ "enabled.desc", "車輛在停車場、倉庫、商場等建築出入口處不再因行人減速或停車，直接穿越行人；行人照常行走。關閉後完全恢復原版行為。 調整後需要幾秒才完全生效，城市越大越慢。請勿遊戲內短時間頻繁修改設定，容易產生混亂。注意：如果道路限速設定過高，車輛AI系統會崩潰失效。" },
				{ "mode.label", "取消避讓範圍" },
				{ "mode.desc", "僅出入口區域：只處理建築出入口、車庫坡道、停車場與建築內部道路。全部道路（全域）：車輛在任何地方都不再讓行人，包括斑馬線。" },
				{ "mode.warning", "全域模式會停用全市所有車輛對行人的避讓。" },
				{ kModeAccess, "僅出入口區域" },
				{ kModeGlobal, "全部道路（全域）" },
				{ "mark.building.label", "建築和設施" },
				{ "mark.building.desc", "開啟後玩家可以點擊某個建築或設施，即可設置該建築或設施為無碰撞模式，並且不受總開關和取消避讓範圍設置項的限制。" },
				{ "mark.crosswalk.label", "人行橫道" },
				{ "mark.crosswalk.desc", "開啟後玩家可以點擊某條人行橫道，即可設置該人行橫道為無碰撞模式，並且不受總開關和取消避讓範圍設置項的限制。" },
				{ "mark.intersection.label", "路口" },
				{ "mark.intersection.desc", "開啟後玩家可以點擊某個路口，即可設置該路口所有人行橫道為無碰撞模式，並且不受總開關和取消避讓範圍設置項的限制。" },
				{ "hotkey.enabled", "啟用/關閉模組" },
				{ "hotkey.enabled.desc", "無需打開選項，直接開關整個模組。注意：模組快捷鍵優先，建議不要與其他模組重複。" },
				{ "hotkey.scope", "切換取消避讓範圍" },
				{ "hotkey.scope.desc", "在「僅出入口區域」與「全部道路（全域）」之間切換。注意：模組快捷鍵優先，建議不要與其他模組重複。" },
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
				{ "group.custommark", "Eigene Markierungen (noch nicht verfügbar)" },
				{ "group.shortcut", "Kurzbefehl-Einstellungen" },
				{ "enabled.label", "Fahrzeuge durchqueren Fußgänger" },
				{ "enabled.desc", "Fahrzeuge bremsen an Gebäudezugängen (Parkplätze, Lagerhäuser, Geschäfte ...) nicht mehr für Fußgänger. Fußgänger laufen normal weiter. Ausschalten stellt das Originalverhalten wieder her. Änderungen brauchen einige Sekunden bis sie vollständig wirken, in großen Städten länger. Bitte die Einstellungen im Spiel nicht kurz hintereinander mehrfach ändern – das kann verwirrende Ergebnisse erzeugen. Hinweis: Wenn die erlaubten Höchstgeschwindigkeiten zu hoch eingestellt sind, bricht die Fahrzeug-KI zusammen und funktioniert nicht mehr." },
				{ "mode.label", "Bereich ohne Vorrang für Fußgänger" },
				{ "mode.desc", "Nur Zufahrtsbereiche: Gebäudezufahrten, Garagenrampen, Parkplätze und Straßen innerhalb von Gebäuden. Alle Straßen (global): Fahrzeuge halten nirgendwo mehr für Fußgänger, auch nicht an Zebrastreifen." },
				{ "mode.warning", "Der globale Modus deaktiviert das Fußgänger-Vorrangverhalten stadtweit." },
				{ kModeAccess, "Nur Zufahrtsbereiche" },
				{ kModeGlobal, "Alle Straßen (global)" },
				{ "mark.building.label", "Gebäude und Anlagen" },
				{ "mark.building.desc", "Wenn aktiviert, kann ein Gebäude oder eine Anlage angeklickt und auf keine Kollision gesetzt werden – unabhängig vom Hauptschalter und vom Bereich ohne Vorrang für Fußgänger." },
				{ "mark.crosswalk.label", "Fußgängerüberwege" },
				{ "mark.crosswalk.desc", "Wenn aktiviert, kann ein Fußgängerüberweg angeklickt und auf keine Kollision gesetzt werden – unabhängig vom Hauptschalter und vom Bereich ohne Vorrang für Fußgänger." },
				{ "mark.intersection.label", "Kreuzungen" },
				{ "mark.intersection.desc", "Wenn aktiviert, kann eine Kreuzung angeklickt und alle Überwege dort auf keine Kollision gesetzt werden – unabhängig vom Hauptschalter und vom Bereich ohne Vorrang für Fußgänger." },
				{ "hotkey.enabled", "Mod aktivieren/deaktivieren" },
				{ "hotkey.enabled.desc", "Das gesamte Mod ohne das Menü Optionen ein- oder ausschalten. Hinweis: Mod-Hotkeys haben Vorrang. Bitte nicht dieselben Tasten wie andere Mods belegen." },
				{ "hotkey.scope", "Bereich ohne Vorrang wechseln" },
				{ "hotkey.scope.desc", "Zwischen nur Zufahrtsbereichen und allen Straßen (global) wechseln. Hinweis: Mod-Hotkeys haben Vorrang. Bitte nicht dieselben Tasten wie andere Mods belegen." },
				{ "group.about", "Über die Mod" },
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
				{ "group.custommark", "Marcas personalizadas (aún no disponible)" },
				{ "group.shortcut", "Ajustes de atajos" },
				{ "enabled.label", "Los vehículos atraviesan peatones" },
				{ "enabled.desc", "Los vehículos ya no frenan ni se detienen ante peatones en los accesos a edificios (aparcamientos, almacenes, tiendas...). Los peatones siguen caminando con normalidad. Desactiva la opción para restaurar el comportamiento original. Los cambios tardan unos segundos en aplicarse del todo, más en ciudades grandes. Evita cambiar los ajustes repetidamente en poco tiempo durante la partida; puede dar resultados confusos. Nota: si los límites de velocidad de las carreteras son demasiado altos, la IA de los vehículos se bloquea y deja de funcionar." },
				{ "mode.label", "Ámbito sin ceder el paso a peatones" },
				{ "mode.desc", "Solo zonas de acceso: entradas y salidas de edificios, rampas de garaje, aparcamientos y vías interiores de los edificios. Todas las carreteras (global): los vehículos nunca ceden el paso a los peatones, incluidos los pasos de cebra." },
				{ "mode.warning", "El modo global hace que los vehículos nunca cedan el paso a los peatones en toda la ciudad." },
				{ kModeAccess, "Solo zonas de acceso" },
				{ kModeGlobal, "Todas las carreteras (global)" },
				{ "mark.building.label", "Edificios e instalaciones" },
				{ "mark.building.desc", "Si está activado, puedes hacer clic en un edificio o instalación para ponerlo en modo sin colisiones, sin que influyan el interruptor principal ni el alcance sin prioridad." },
				{ "mark.crosswalk.label", "Pasos de peatones" },
				{ "mark.crosswalk.desc", "Si está activado, puedes hacer clic en un paso de peatones para ponerlo en modo sin colisiones, sin que influyan el interruptor principal ni el alcance sin prioridad." },
				{ "mark.intersection.label", "Intersecciones" },
				{ "mark.intersection.desc", "Si está activado, puedes hacer clic en una intersección para poner todos los pasos de peatones de esa intersección en modo sin colisiones, sin que influyan el interruptor principal ni el alcance sin prioridad." },
				{ "hotkey.enabled", "Activar/desactivar el mod" },
				{ "hotkey.enabled.desc", "Activa o desactiva todo el mod sin abrir el menú de opciones. Nota: los atajos del mod tienen prioridad; evita usar las mismas teclas que otros mods." },
				{ "hotkey.scope", "Cambiar ámbito sin ceder el paso" },
				{ "hotkey.scope.desc", "Alterna entre solo zonas de acceso y todas las carreteras (global). Nota: los atajos del mod tienen prioridad; evita usar las mismas teclas que otros mods." },
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
				{ "group.custommark", "Repères personnalisés (pas encore disponible)" },
				{ "group.shortcut", "Paramètres des raccourcis" },
				{ "enabled.label", "Les véhicules traversent les piétons" },
				{ "enabled.desc", "Les véhicules ne ralentissent plus ni ne s'arrêtent devant les piétons aux accès de bâtiments (parkings, entrepôts, magasins...). Les piétons continuent de marcher normalement. Désactivez pour restaurer le comportement d'origine. Les modifications mettent quelques secondes à s'appliquer pleinement, davantage dans les grandes villes. Évitez de modifier les réglages à répétition en peu de temps pendant la partie ; les résultats peuvent prêter à confusion. Remarque : si les limites de vitesse sont réglées trop haut, l’IA des véhicules se bloque et ne fonctionne plus." },
				{ "mode.label", "Périmètre sans priorité aux piétons" },
				{ "mode.desc", "Zones d'accès uniquement : entrées et sorties de bâtiments, rampes, parcs de stationnement et voirie interne des bâtiments. Toutes les routes (globale) : les véhicules ne cèdent plus le passage aux piétons nulle part, passages piétons compris." },
				{ "mode.warning", "Le mode global désactive la priorité aux piétons dans toute la ville." },
				{ kModeAccess, "Zones d'accès uniquement" },
				{ kModeGlobal, "Toutes les routes (globale)" },
				{ "mark.building.label", "Bâtiments et équipements" },
				{ "mark.building.desc", "Une fois activé, cliquez sur un bâtiment ou une installation pour le passer en mode sans collision, indépendamment du commutateur principal et de la portée sans céder le passage." },
				{ "mark.crosswalk.label", "Passages piétons" },
				{ "mark.crosswalk.desc", "Une fois activé, cliquez sur un passage piéton pour le passer en mode sans collision, indépendamment du commutateur principal et de la portée sans céder le passage." },
				{ "mark.intersection.label", "Carrefours" },
				{ "mark.intersection.desc", "Une fois activé, cliquez sur un carrefour pour passer tous les passages piétons de ce carrefour en mode sans collision, indépendamment du commutateur principal et de la portée sans céder le passage." },
				{ "hotkey.enabled", "Activer/désactiver le mod" },
				{ "hotkey.enabled.desc", "Active ou désactive tout le mod sans ouvrir le menu des options. Remarque : les raccourcis du mod sont prioritaires ; évitez les mêmes touches que d’autres mods." },
				{ "hotkey.scope", "Changer le périmètre sans priorité" },
				{ "hotkey.scope.desc", "Bascule entre zones d'accès uniquement et toutes les routes (globale). Remarque : les raccourcis du mod sont prioritaires ; évitez les mêmes touches que d’autres mods." },
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
				{ "group.custommark", "Marcature personalizzate (non ancora disponibile)" },
				{ "group.shortcut", "Impostazioni scorciatoie" },
				{ "enabled.label", "I veicoli attraversano i pedoni" },
				{ "enabled.desc", "I veicoli non rallentano più né si fermano per i pedoni agli accessi degli edifici (parcheggi, magazzini, negozi...). I pedoni continuano a camminare normalmente. Disattiva per ripristinare il comportamento originale. Le modifiche richiedono alcuni secondi per applicarsi del tutto, di più nelle città grandi. Evita di modificare le impostazioni ripetutamente in poco tempo durante la partita; i risultati possono risultare confusi. Nota: se i limiti di velocità stradali sono impostati troppo alti, l’IA dei veicoli si blocca e smette di funzionare." },
				{ "mode.label", "Ambito senza precedenza ai pedoni" },
				{ "mode.desc", "Solo zone di accesso: ingressi e uscite degli edifici, rampe dei garage, parcheggi e strade interne degli edifici. Tutte le strade (globale): i veicoli non danno più precedenza ai pedoni da nessuna parte, strisce pedonali incluse." },
				{ "mode.warning", "La modalità globale disattiva la precedenza ai pedoni in tutta la città." },
				{ kModeAccess, "Solo zone di accesso" },
				{ kModeGlobal, "Tutte le strade (globale)" },
				{ "mark.building.label", "Edifici e impianti" },
				{ "mark.building.desc", "Se attivo, clicca un edificio o un impianto per impostarlo in modalità senza collisioni, indipendentemente da interruttore principale e ambito senza precedenza." },
				{ "mark.crosswalk.label", "Attraversamenti pedonali" },
				{ "mark.crosswalk.desc", "Se attivo, clicca un attraversamento pedonale per impostarlo in modalità senza collisioni, indipendentemente da interruttore principale e ambito senza precedenza." },
				{ "mark.intersection.label", "Incroci" },
				{ "mark.intersection.desc", "Se attivo, clicca un incrocio per impostare in modalità senza collisioni tutti gli attraversamenti pedonali di quell'incrocio, indipendentemente da interruttore principale e ambito senza precedenza." },
				{ "hotkey.enabled", "Attiva/disattiva la mod" },
				{ "hotkey.enabled.desc", "Attiva o disattiva l'intera mod senza aprire il menu delle opzioni. Nota: le scorciatoie della mod hanno la precedenza; evita gli stessi tasti di altre mod." },
				{ "hotkey.scope", "Cambia ambito senza precedenza" },
				{ "hotkey.scope.desc", "Alterna tra solo zone di accesso e tutte le strade (globale). Nota: le scorciatoie della mod hanno la precedenza; evita gli stessi tasti di altre mod." },
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
				{ "group.custommark", "カスタムマーカー（未実装）" },
				{ "group.shortcut", "ショートカット設定" },
				{ "enabled.label", "車両が歩行者を避けない" },
				{ "enabled.desc", "建物の出入口（駐車場・倉庫・店舗など）で、車両が歩行者に合わせて減速・停止しなくなり、そのまま通り抜けます。歩行者は普段どおり歩き続けます。オフにすると標準の動作に戻ります。 反映には数秒かかります。都市が大きいほど時間がかかります。プレイ中に設定を短時間で何度も変更しないでください。挙動が分かりにくくなることがあります。注意：道路の速度制限が高すぎると、車両AIが破綻して機能しなくなります。" },
				{ "mode.label", "歩行者を優先しない範囲" },
				{ "mode.desc", "出入口のみ：建物の出入口・ガレージランプ・駐車場・建物内の道路のみ。すべての道路（グローバル）：横断歩道を含め、車両はどこでも歩行者に道を譲らなくなります。" },
				{ "mode.warning", "グローバルモードでは、市内全体で車両が歩行者に道を譲らなくなります。" },
				{ kModeAccess, "出入口のみ" },
				{ kModeGlobal, "すべての道路（グローバル）" },
				{ "mark.building.label", "建築物・施設" },
				{ "mark.building.desc", "オンにすると、建物または施設をクリックして、その建物・施設を衝突なしモードに設定できます。オン/オフスイッチと対象範囲の設定は影響しません。" },
				{ "mark.crosswalk.label", "横断歩道" },
				{ "mark.crosswalk.desc", "オンにすると、横断歩道をクリックして、その横断歩道を衝突なしモードに設定できます。オン/オフスイッチと対象範囲の設定は影響しません。" },
				{ "mark.intersection.label", "交差点" },
				{ "mark.intersection.desc", "オンにすると、交差点をクリックして、その交差点のすべての横断歩道を衝突なしモードに設定できます。オン/オフスイッチと対象範囲の設定は影響しません。" },
				{ "hotkey.enabled", "Modの有効/無効" },
				{ "hotkey.enabled.desc", "オプションメニューを開かずにMod全体をオン/オフします。注意：MODのショートカットキーが優先されます。他のMODと同じキーに割り当てないでください。" },
				{ "hotkey.scope", "優先しない範囲を切り替え" },
				{ "hotkey.scope.desc", "「出入口のみ」と「すべての道路（グローバル）」を切り替えます。注意：MODのショートカットキーが優先されます。他のMODと同じキーに割り当てないでください。" },
				{ "group.about", "このModについて" },
				{ "version.label", "Modバージョン" },
				{ "author.label", "作成者" },
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
				{ "group.custommark", "사용자 지정 표시 (미출시)" },
				{ "group.shortcut", "단축키 설정" },
				{ "enabled.label", "차량이 보행자를 통과" },
				{ "enabled.desc", "건물 출입구(주차장, 창고, 상점 등)에서 차량이 보행자 때문에 감속하거나 정지하지 않고 그대로 통과합니다. 보행자는 평소대로 걷습니다. 끄면 원래 동작으로 돌아갑니다. 적용에 몇 초 걸릴 수 있으며, 도시가 클수록 더 오래 걸립니다. 게임 중에 설정을 짧은 시간에 반복해서 바꾸지 마세요. 결과가 혼란스러울 수 있습니다.참고: 도로 속도 제한을 너무 높게 설정하면 차량 AI가 정상적으로 작동하지 않습니다." },
				{ "mode.label", "양보 제외 범위" },
				{ "mode.desc", "출입구 영역만: 건물 출입구, 주차장 경사로, 주차장 내부와 건물 내부 도로만 적용. 모든 도로(전체): 횡단보도를 포함해 차량이 어디서도 보행자에게 양보하지 않습니다." },
				{ "mode.warning", "전체 모드에서는 도시 전체에서 차량의 보행자 양보가 비활성화됩니다." },
				{ kModeAccess, "출입구 영역만" },
				{ kModeGlobal, "모든 도로(전체)" },
				{ "mark.building.label", "건물 및 시설" },
				{ "mark.building.desc", "켠 상태에서 건물이나 시설을 클릭하면 해당 건물·시설을 충돌 없음 모드로 설정할 수 있으며, 전체 스위치와 양보 해제 범위 설정의 영향을 받지 않습니다." },
				{ "mark.crosswalk.label", "횡단보도" },
				{ "mark.crosswalk.desc", "켠 상태에서 횡단보도를 클릭하면 해당 횡단보도를 충돌 없음 모드로 설정할 수 있으며, 전체 스위치와 양보 해제 범위 설정의 영향을 받지 않습니다." },
				{ "mark.intersection.label", "교차로" },
				{ "mark.intersection.desc", "켠 상태에서 교차로를 클릭하면 그 교차로의 모든 횡단보도를 충돌 없음 모드로 설정할 수 있으며, 전체 스위치와 양보 해제 범위 설정의 영향을 받지 않습니다." },
				{ "hotkey.enabled", "모드 켜기/끄기" },
				{ "hotkey.enabled.desc", "옵션 메뉴를 열지 않고 모드 전체를 켜거나 끕니다.참고: 모드 단축키가 우선 적용됩니다. 다른 모드와 같은 키를 사용하지 마세요." },
				{ "hotkey.scope", "양보 제외 범위 전환" },
				{ "hotkey.scope.desc", "출입구 영역만과 모든 도로(전역) 사이를 전환합니다.참고: 모드 단축키가 우선 적용됩니다. 다른 모드와 같은 키를 사용하지 마세요." },
				{ "group.about", "모드 정보" },
				{ "version.label", "모드 버전" },
				{ "author.label", "제작자" },
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
				{ "group.custommark", "Własne zaznaczenia (jeszcze niedostępne)" },
				{ "group.shortcut", "Ustawienia skrótów" },
				{ "enabled.label", "Pojazdy przenikają przez pieszych" },
				{ "enabled.desc", "Pojazdy nie zwalniają już i nie zatrzymują się przed pieszymi przy wjazdach do budynków (parkingi, magazyny, sklepy...). Piesi chodzą normalnie. Wyłącz, aby przywrócić zachowanie z gry. Zmiany potrzebują kilku sekund, w dużych miastach dłużej. Nie zmieniaj ustawień wielokrotnie w krótkim czasie podczas gry – może to dawać mylące rezultaty. Uwaga: jeśli limity prędkości na drogach są ustawione zbyt wysoko, AI pojazdów przestaje działać poprawnie." },
				{ "mode.label", "Zakres bez ustępowania pieszym" },
				{ "mode.desc", "Tylko strefy dostępu: wjazdy i wyjazdy z budynków, rampy garaży, parkingi oraz drogi wewnętrzne budynków. Wszystkie drogi (globalnie): pojazdy nigdzie nie ustępują pierwszeństwa pieszym, także na przejściach." },
				{ "mode.warning", "Tryb globalny wyłącza ustępowanie pierwszeństwa pieszym w całym mieście." },
				{ kModeAccess, "Tylko strefy dostępu" },
				{ kModeGlobal, "Wszystkie drogi (globalnie)" },
				{ "mark.building.label", "Budynki i obiekty" },
				{ "mark.building.desc", "Po włączeniu kliknij budynek lub obiekt, aby ustawić go w tryb bez kolizji – ustawienie główne i zakres bez pierwszeństwa nie mają na to wpływu." },
				{ "mark.crosswalk.label", "Przejścia dla pieszych" },
				{ "mark.crosswalk.desc", "Po włączeniu kliknij przejście dla pieszych, aby ustawić je w tryb bez kolizji – ustawienie główne i zakres bez pierwszeństwa nie mają na to wpływu." },
				{ "mark.intersection.label", "Skrzyżowania" },
				{ "mark.intersection.desc", "Po włączeniu kliknij skrzyżowanie, aby ustawić w tryb bez kolizji wszystkie przejścia na tym skrzyżowaniu – ustawienie główne i zakres bez pierwszeństwa nie mają na to wpływu." },
				{ "hotkey.enabled", "Włącz/wyłącz mod" },
				{ "hotkey.enabled.desc", "Włącz lub wyłącz cały mod bez otwierania menu opcji. Uwaga: skróty moda mają priorytet; nie przypisuj im klawiszy używanych przez inne mody." },
				{ "hotkey.scope", "Przełącz zakres bez ustępowania" },
				{ "hotkey.scope.desc", "Przełącza między tylko strefami dostępu a wszystkimi drogami (globalnie). Uwaga: skróty moda mają priorytet; nie przypisuj im klawiszy używanych przez inne mody." },
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
				{ "group.custommark", "Marcas personalizadas (ainda não disponível)" },
				{ "group.shortcut", "Configurações de atalhos" },
				{ "enabled.label", "Veículos atravessam pedestres" },
				{ "enabled.desc", "Veículos não reduzem mais a velocidade nem param para pedestres nos acessos de edifícios (estacionamentos, armazéns, lojas...). Os pedestres continuam caminhando normalmente. Desligue para restaurar o comportamento original. As alterações levam alguns segundos para valer por completo, mais nas cidades grandes. Evite alterar as configurações repetidamente em pouco tempo durante a partida; isso pode gerar resultados confusos. Observação: se os limites de velocidade das estradas forem definidos muito altos, a IA dos veículos entra em colapso e para de funcionar." },
				{ "mode.label", "Escopo sem dar preferência a pedestres" },
				{ "mode.desc", "Somente zonas de acesso: entradas e saídas de edifícios, rampas de garagem, estacionamentos e vias internas dos edifícios. Todas as vias (global): veículos nunca dão preferência a pedestres, incluindo faixas de pedestres." },
				{ "mode.warning", "O modo global desativa a preferência a pedestres em toda a cidade." },
				{ kModeAccess, "Somente zonas de acesso" },
				{ kModeGlobal, "Todas as vias (global)" },
				{ "mark.building.label", "Edifícios e instalações" },
				{ "mark.building.desc", "Quando ativado, clique em um edifício ou instalação para defini-lo no modo sem colisões, sem influência do interruptor principal nem do alcance sem preferência." },
				{ "mark.crosswalk.label", "Faixas de pedestres" },
				{ "mark.crosswalk.desc", "Quando ativado, clique em uma faixa de pedestres para defini-la no modo sem colisões, sem influência do interruptor principal nem do alcance sem preferência." },
				{ "mark.intersection.label", "Cruzamentos" },
				{ "mark.intersection.desc", "Quando ativado, clique em um cruzamento para definir todas as faixas daquele cruzamento no modo sem colisões, sem influência do interruptor principal nem do alcance sem preferência." },
				{ "hotkey.enabled", "Ativar/desativar o mod" },
				{ "hotkey.enabled.desc", "Ativa ou desativa todo o mod sem abrir o menu de opções. Observação: os atalhos do mod têm prioridade; evite usar as mesmas teclas de outros mods." },
				{ "hotkey.scope", "Alternar escopo sem dar preferência" },
				{ "hotkey.scope.desc", "Alterna entre somente zonas de acesso e todas as vias (global). Observação: os atalhos do mod têm prioridade; evite usar as mesmas teclas de outros mods." },
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
				{ "group.custommark", "Пользовательские метки (пока недоступно)" },
				{ "group.shortcut", "Настройки ярлыков" },
				{ "enabled.label", "Транспорт проходит сквозь пешеходов" },
				{ "enabled.desc", "Транспорт больше не замедляется и не останавливается перед пешеходами у въездов на объекты (парковки, склады, магазины...). Пешеходы ходят как обычно. Выключите, чтобы вернуть исходное поведение. Изменения применяются несколько секунд, в крупных городах — дольше. Не меняйте настройки многократно за короткое время во время игры — это может дать запутанный результат. Примечание: если лимиты скорости на дорогах слишком высоки, ИИ транспорта перестаёт корректно работать." },
				{ "mode.label", "Область, где не уступают дорогу пешеходам" },
				{ "mode.desc", "Только зоны въездов: въезды и выезды на объекты, рампы паркингов, парковки и внутренние дороги зданий. Все дороги (глобально): транспорт нигде не уступает дорогу пешеходам, включая пешеходные переходы." },
				{ "mode.warning", "Глобальный режим отменяет уступку дороги пешеходам по всему городу." },
				{ kModeAccess, "Только зоны въездов" },
				{ kModeGlobal, "Все дороги (глобально)" },
				{ "mark.building.label", "Здания и объекты" },
				{ "mark.building.desc", "Если включено, кликните здание или объект, чтобы перевести его в режим без столкновений; главный переключатель и диапазон без уступки проезда на это не влияют." },
				{ "mark.crosswalk.label", "Пешеходные переходы" },
				{ "mark.crosswalk.desc", "Если включено, кликните пешеходный переход, чтобы перевести его в режим без столкновений; главный переключатель и диапазон без уступки проезда на это не влияют." },
				{ "mark.intersection.label", "Перекрёстки" },
				{ "mark.intersection.desc", "Если включено, кликните перекрёсток, чтобы перевести все переходы на нём в режим без столкновений; главный переключатель и диапазон без уступки проезда на это не влияют." },
				{ "hotkey.enabled", "Включить/выключить мод" },
				{ "hotkey.enabled.desc", "Включает или выключает весь мод без открытия меню параметров. Примечание: горячие клавиши мода имеют приоритет; не назначайте те же клавиши, что и у других модов." },
				{ "hotkey.scope", "Переключить область, где не уступают дорогу" },
				{ "hotkey.scope.desc", "Переключает между зонами въездов и всеми дорогами (глобально). Примечание: горячие клавиши мода имеют приоритет; не назначайте те же клавиши, что и у других модов." },
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
