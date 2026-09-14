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

namespace AccessAnarchy
{
	/// <summary>
	/// 模组选项（官方 ModSetting 框架，自动持久化到 ModsSettings\AccessAnarchy.coc）。
	/// 框架会在主线程直接调属性 setter，所以 getter 的返回值总是当前生效值；
	/// AccessZoneOverlapSystem 每次调度 Job 前直接读 Instance 的 getter，无需静态镜像。
	/// </summary>
	[FileLocation(nameof(AccessAnarchy))]
	[SettingsUIGroupOrder(kGroupMain, kGroupScope)]
	[SettingsUIShowGroupName(kGroupMain, kGroupScope)]
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

		/// <summary>快捷键动作名：启用/关闭（默认 F3）。</summary>
		public const string kToggleEnabledAction = "ToggleEnabled";

		/// <summary>快捷键动作名：切换避让范围（默认 F4）。</summary>

		public const string kToggleScopeAction = "ToggleScope";

		/// <summary>作用模式：只处理建筑出入口区域的车道。</summary>
		public const int kModeAccessOnly = 0;

		/// <summary>作用模式：处理所有道路（全局禁用车辆-行人避让）。</summary>
		public const int kModeGlobal = 1;

		/// <summary>系统访问当前设置实例的入口；OnLoad 赋值，OnDispose 清空。</summary>
		public static Setting Instance;

		private bool m_Enabled = true;
		private int m_Mode = kModeAccessOnly;
		private bool m_IncludeGarageLanes = true;
		private bool m_IncludeConnectionLanes = true;
		private bool m_IncludeParkingLotLanes = true;

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

		/// <summary>车库/停车楼出入口坡道（GarageLane）。</summary>
		[SettingsUISection(kSection, kGroupScope)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(IsGlobalMode))]
		public bool IncludeGarageLanes
		{
			get => m_IncludeGarageLanes;
			set => m_IncludeGarageLanes = value;
		}

		/// <summary>建筑/场站与道路之间的出入连接道（ConnectionLane，含行人连接道）。</summary>
		[SettingsUISection(kSection, kGroupScope)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(IsGlobalMode))]
		public bool IncludeConnectionLanes
		{
			get => m_IncludeConnectionLanes;
			set => m_IncludeConnectionLanes = value;
		}

		/// <summary>停车场内部车道与建筑自有车道（AreaLane / 非道路所有）。</summary>
		[SettingsUISection(kSection, kGroupScope)]
		[SettingsUIDisableByCondition(typeof(Setting), nameof(IsGlobalMode))]
		public bool IncludeParkingLotLanes
		{
			get => m_IncludeParkingLotLanes;
			set => m_IncludeParkingLotLanes = value;
		}

		public bool IsGlobalMode()
		{
			return m_Mode == kModeGlobal;
		}

		/// <summary>启用/关闭快捷键（默认 F3）。玩家改键由框架 KeybindingSettings 持久化。</summary>
		[SettingsUIKeyboardBinding(BindingKeyboard.F3, kToggleEnabledAction)]
		public ProxyBinding ToggleEnabledBinding { get; set; }

		/// <summary>切换避让范围快捷键（默认 F4）。</summary>
		[SettingsUIKeyboardBinding(BindingKeyboard.F4, kToggleScopeAction)]
		public ProxyBinding ToggleScopeBinding { get; set; }

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
			m_IncludeGarageLanes = true;
			m_IncludeConnectionLanes = true;
			m_IncludeParkingLotLanes = true;
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

				{ setting.GetOptionLabelLocaleID(nameof(Setting.Enabled)), d["enabled.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.Enabled)), d["enabled.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.Mode)), d["mode.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.Mode)), d["mode.desc"] },
				{ setting.GetOptionWarningLocaleID(nameof(Setting.Mode)), d["mode.warning"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.IncludeGarageLanes)), d["garage.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.IncludeGarageLanes)), d["garage.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.IncludeConnectionLanes)), d["conn.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.IncludeConnectionLanes)), d["conn.desc"] },

				{ setting.GetOptionLabelLocaleID(nameof(Setting.IncludeParkingLotLanes)), d["lot.label"] },
				{ setting.GetOptionDescLocaleID(nameof(Setting.IncludeParkingLotLanes)), d["lot.desc"] },

				{ setting.GetBindingKeyLocaleID(Setting.kToggleEnabledAction), d["hotkey.enabled"] },
				{ setting.GetBindingKeyLocaleID(Setting.kToggleScopeAction), d["hotkey.scope"] },
				{ setting.GetBindingMapLocaleID(), d["bindingMap"] }
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
				{ "group.main", "Behaviour" },
				{ "group.scope", "Access zone scope" },
				{ "enabled.label", "Vehicles pass through pedestrians" },
				{ "enabled.desc", "Vehicles no longer slow down or stop for pedestrians at building access points (parking lots, warehouses, stores...). Pedestrians keep walking normally. Turn off to fully restore vanilla behaviour." },
				{ "mode.label", "Scope mode" },
				{ "mode.desc", "Access zones only: only building entrances, garage ramps and parking lots. All roads (global): vehicles never yield to pedestrians anywhere, crosswalks included." },
				{ "mode.warning", "Global mode disables all car-pedestrian yielding citywide." },
				{ kModeAccess, "Access zones only" },
				{ kModeGlobal, "All roads (global)" },
				{ "garage.label", "Garage ramps" },
				{ "garage.desc", "Include garage / parking building entrance ramps." },
				{ "conn.label", "Building driveways" },
				{ "conn.desc", "Include the lanes between buildings/terminals and the road network (vehicle and pedestrian connection lanes)." },
				{ "lot.label", "Parking lots and building-internal lanes" },
				{ "lot.desc", "Include lanes inside parking lots and other building-owned roads." },
				{ "hotkey.enabled", "Enable/disable the mod" },
				{ "hotkey.scope", "Switch scope mode" },
				{ "bindingMap", "Access Anarchy key bindings" }
			};
		}

		private static Dictionary<string, string> ZhHans()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "常规" },
				{ "group.main", "行为" },
				{ "group.scope", "出入口范围" },
				{ "enabled.label", "车辆穿过行人" },
				{ "enabled.desc", "车辆在停车场、仓库、商场等建筑出入口处不再因行人减速或停车，直接穿过行人；行人照常行走。关闭后完全恢复原版行为。" },
				{ "mode.label", "作用范围" },
				{ "mode.desc", "仅出入口区域：只处理建筑出入口、车库坡道与停车场内部。全部道路（全局）：车辆在任何地方都不再让行行人，包括人行横道。" },
				{ "mode.warning", "全局模式会禁用全市所有车辆对行人的避让。" },
				{ kModeAccess, "仅出入口区域" },
				{ kModeGlobal, "全部道路（全局）" },
				{ "garage.label", "车库坡道" },
				{ "garage.desc", "包含车库/停车楼的出入口坡道。" },
				{ "conn.label", "建筑出入连接道" },
				{ "conn.desc", "包含建筑/场站与路网之间的连接道（车辆与行人连接道）。" },
				{ "lot.label", "停车场与建筑内部车道" },
				{ "lot.desc", "包含停车场内部车道与建筑自有道路。" },
				{ "hotkey.enabled", "启用/关闭模组" },
				{ "hotkey.scope", "切换避让范围" },
				{ "bindingMap", "Access Anarchy 键位" }
			};
		}

		private static Dictionary<string, string> ZhHant()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "一般" },
				{ "group.main", "行為" },
				{ "group.scope", "出入口範圍" },
				{ "enabled.label", "車輛穿越行人" },
				{ "enabled.desc", "車輛在停車場、倉庫、商場等建築出入口處不再因行人減速或停車，直接穿越行人；行人照常行走。關閉後完全恢復原版行為。" },
				{ "mode.label", "作用範圍" },
				{ "mode.desc", "僅出入口區域：只處理建築出入口、車庫坡道與停車場內部。全部道路（全域）：車輛在任何地方都不再禮讓行人，包括行人穿越道。" },
				{ "mode.warning", "全域模式會停用全市所有車輛對行人的禮讓。" },
				{ kModeAccess, "僅出入口區域" },
				{ kModeGlobal, "全部道路（全域）" },
				{ "garage.label", "車庫坡道" },
				{ "garage.desc", "包含車庫/停車樓的出入口坡道。" },
				{ "conn.label", "建築出入連接道" },
				{ "conn.desc", "包含建築/場站與路網之間的連接道（車輛與行人連接道）。" },
				{ "lot.label", "停車場與建築內部車道" },
				{ "lot.desc", "包含停車場內部車道與建築自有道路。" },
				{ "hotkey.enabled", "啟用/關閉模組" },
				{ "hotkey.scope", "切換避讓範圍" },
				{ "bindingMap", "Access Anarchy 鍵位" }
			};
		}

		private static Dictionary<string, string> De()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "Allgemein" },
				{ "group.main", "Verhalten" },
				{ "group.scope", "Zugangsbereiche" },
				{ "enabled.label", "Fahrzeuge durchqueren Fußgänger" },
				{ "enabled.desc", "Fahrzeuge bremsen an Gebäudezugängen (Parkplätze, Lagerhäuser, Geschäfte ...) nicht mehr für Fußgänger. Fußgänger laufen normal weiter. Ausschalten stellt das Originalverhalten wieder her." },
				{ "mode.label", "Geltungsbereich" },
				{ "mode.desc", "Nur Zugangsbereiche: nur Gebäudeeingänge, Garagenrampen und Parkplätze. Alle Straßen (global): Fahrzeuge halten nirgendwo mehr für Fußgänger, auch nicht an Zebrastreifen." },
				{ "mode.warning", "Der globale Modus deaktiviert das Fußgänger-Vorrangverhalten stadtwelt." },
				{ kModeAccess, "Nur Zugangsbereiche" },
				{ kModeGlobal, "Alle Straßen (global)" },
				{ "garage.label", "Garagenrampen" },
				{ "garage.desc", "Einfahrtsrampen von Tiefgaragen und Parkhäusern einschließen." },
				{ "conn.label", "Gebäudezufahrten" },
				{ "conn.desc", "Verbindungsspur zwischen Gebäuden und Straßennetz einschließen (Fahrzeug- und Fußgängerverbindungen)." },
				{ "lot.label", "Parkplätze und Gebäudeinnenflächen" },
				{ "lot.desc", "Fahrbahnen innerhalb von Parkplätzen und andere grundstückseigene Straßen einschließen." },
				{ "hotkey.enabled", "Mod aktivieren/deaktivieren" },
				{ "hotkey.scope", "Bereichsmodus wechseln" },
				{ "bindingMap", "Access Anarchy-Tastenkürzel" }
			};
		}

		private static Dictionary<string, string> Es()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "General" },
				{ "group.main", "Comportamiento" },
				{ "group.scope", "Ámbito de accesos" },
				{ "enabled.label", "Los vehículos atraviesan peatones" },
				{ "enabled.desc", "Los vehículos ya no frenan ni se detienen ante peatones en los accesos a edificios (aparcamientos, almacenes, tiendas...). Los peatones siguen caminando con normalidad. Desactiva la opción para restaurar el comportamiento original." },
				{ "mode.label", "Modo de alcance" },
				{ "mode.desc", "Solo accesos: solo entradas de edificios, rampas de garaje y aparcamientos. Todas las vías (global): los vehículos nunca ceden el paso a los peatones, incluidos los pasos de cebra." },
				{ "mode.warning", "El modo global desactiva la cesión a peatones en toda la ciudad." },
				{ kModeAccess, "Solo accesos" },
				{ kModeGlobal, "Todas las vías (global)" },
				{ "garage.label", "Rampas de garaje" },
				{ "garage.desc", "Incluir las rampas de entrada de garajes y aparcamientos." },
				{ "conn.label", "Accesos a edificios" },
				{ "conn.desc", "Incluir los carriles entre edificios y la red viaria (conexiones de vehículo y de peatón)." },
				{ "lot.label", "Aparcamientos y vías internas" },
				{ "lot.desc", "Incluir los carriles dentro de aparcamientos y otras vías propias de los edificios." },
				{ "hotkey.enabled", "Activar/desactivar el mod" },
				{ "hotkey.scope", "Cambiar modo de alcance" },
				{ "bindingMap", "Atajos de Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Fr()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "Général" },
				{ "group.main", "Comportement" },
				{ "group.scope", "Périmètre des accès" },
				{ "enabled.label", "Les véhicules traversent les piétons" },
				{ "enabled.desc", "Les véhicules ne ralentissent plus ni ne s'arrêtent devant les piétons aux accès de bâtiments (parkings, entrepôts, magasins...). Les piétons continuent de marcher normalement. Désactivez pour restaurer le comportement d'origine." },
				{ "mode.label", "Mode de portée" },
				{ "mode.desc", "Accès uniquement : uniquement les entrées de bâtiments, rampes de parking et parkings. Toutes les voies (global) : les véhicules ne cèdent plus le passage aux piétons nulle part, passages piétons compris." },
				{ "mode.warning", "Le mode global désactive la priorité aux piétons dans toute la ville." },
				{ kModeAccess, "Accès uniquement" },
				{ kModeGlobal, "Toutes les voies (global)" },
				{ "garage.label", "Rampes de garage" },
				{ "garage.desc", "Inclure les rampes d'entrée des garages et parkings silos." },
				{ "conn.label", "Accès aux bâtiments" },
				{ "conn.desc", "Inclure les voies entre les bâtiments et le réseau routier (connexions véhicules et piétons)." },
				{ "lot.label", "Parkings et voies internes" },
				{ "lot.desc", "Inclure les voies à l'intérieur des parkings et autres routes privées des bâtiments." },
				{ "hotkey.enabled", "Activer/désactiver le mod" },
				{ "hotkey.scope", "Changer le mode de portée" },
				{ "bindingMap", "Raccourcis d'Access Anarchy" }
			};
		}

		private static Dictionary<string, string> It()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "Generale" },
				{ "group.main", "Comportamento" },
				{ "group.scope", "Ambito degli accessi" },
				{ "enabled.label", "I veicoli attraversano i pedoni" },
				{ "enabled.desc", "I veicoli non rallentano più né si fermano per i pedoni agli accessi degli edifici (parcheggi, magazzini, negozi...). I pedoni continuano a camminare normalmente. Disattiva per ripristinare il comportamento originale." },
				{ "mode.label", "Modalità di ambito" },
				{ "mode.desc", "Solo accessi: solo ingressi degli edifici, rampe dei garage e parcheggi. Tutte le strade (globale): i veicoli non danno più precedenza ai pedoni da nessuna parte, strisce pedonali incluse." },
				{ "mode.warning", "La modalità globale disattiva la precedenza ai pedoni in tutta la città." },
				{ kModeAccess, "Solo accessi" },
				{ kModeGlobal, "Tutte le strade (globale)" },
				{ "garage.label", "Rampe dei garage" },
				{ "garage.desc", "Includi le rampe d'ingresso di garage e parcheggi multipiano." },
				{ "conn.label", "Accessi agli edifici" },
				{ "conn.desc", "Includi le corsie tra edifici e rete stradale (connessioni veicoli e pedoni)." },
				{ "lot.label", "Parcheggi e strade interne" },
				{ "lot.desc", "Includi le corsie all'interno dei parcheggi e le strade private degli edifici." },
				{ "hotkey.enabled", "Attiva/disattiva la mod" },
				{ "hotkey.scope", "Cambia modalità di ambito" },
				{ "bindingMap", "Scorciatoie di Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Ja()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "一般" },
				{ "group.main", "動作" },
				{ "group.scope", "アクセス範囲" },
				{ "enabled.label", "車両が歩行者を避けない" },
				{ "enabled.desc", "建物の出入口（駐車場・倉庫・店舗など）で、車両が歩行者に合わせて減速・停止しなくなり、そのまま通り抜けます。歩行者は普段どおり歩き続けます。オフにすると標準の動作に戻ります。" },
				{ "mode.label", "適用範囲" },
				{ "mode.desc", "アクセスのみ：建物の出入口・ガレージランプ・駐車場のみ。すべての道路（グローバル）：横断歩道を含め、車両はどこでも歩行者に道を譲らなくなります。" },
				{ "mode.warning", "グローバルモードでは、市内全体で車両が歩行者に道を譲らなくなります。" },
				{ kModeAccess, "アクセスのみ" },
				{ kModeGlobal, "すべての道路（グローバル）" },
				{ "garage.label", "ガレージランプ" },
				{ "garage.desc", "ガレージ・駐車場の出入口ランプを含めます。" },
				{ "conn.label", "建物のアクセス路" },
				{ "conn.desc", "建物と道路網をつなぐ車線を含めます（車両用・歩行者用の両方）。" },
				{ "lot.label", "駐車場と敷地内道路" },
				{ "lot.desc", "駐車場内の車線や建物所有の道路を含めます。" },
				{ "hotkey.enabled", "Modの有効/無効" },
				{ "hotkey.scope", "適用範囲の切り替え" },
				{ "bindingMap", "Access Anarchy のキー割り当て" }
			};
		}

		private static Dictionary<string, string> Ko()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "일반" },
				{ "group.main", "동작" },
				{ "group.scope", "출입 구역 범위" },
				{ "enabled.label", "차량이 보행자를 통과" },
				{ "enabled.desc", "건물 출입구(주차장, 창고, 상점 등)에서 차량이 보행자 때문에 감속하거나 정지하지 않고 그대로 통과합니다. 보행자는 평소대로 걷습니다. 끄면 원래 동작으로 돌아갑니다." },
				{ "mode.label", "적용 범위" },
				{ "mode.desc", "출입 구역만: 건물 출입구, 주차장 경사로, 주차장 내부만 적용. 모든 도로(전역): 횡단보도를 포함해 차량이 어디서도 보행자에게 양보하지 않습니다." },
				{ "mode.warning", "전역 모드에서는 도시 전체에서 차량의 보행자 양보가 비활성화됩니다." },
				{ kModeAccess, "출입 구역만" },
				{ kModeGlobal, "모든 도로(전역)" },
				{ "garage.label", "주차장 경사로" },
				{ "garage.desc", "지하주차장·주차타워의 출입 경사로를 포함합니다." },
				{ "conn.label", "건물 출입 연결로" },
				{ "conn.desc", "건물과 도로망 사이의 차선을 포함합니다(차량용 및 보행자용 연결로)." },
				{ "lot.label", "주차장 및 부지 내 도로" },
				{ "lot.desc", "주차장 내부 차선과 건물 소유 도로를 포함합니다." },
				{ "hotkey.enabled", "모드 켜기/끄기" },
				{ "hotkey.scope", "적용 범위 전환" },
				{ "bindingMap", "Access Anarchy 키 설정" }
			};
		}

		private static Dictionary<string, string> Pl()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "Ogólne" },
				{ "group.main", "Zachowanie" },
				{ "group.scope", "Zakres stref dostępu" },
				{ "enabled.label", "Pojazdy przenikają przez pieszych" },
				{ "enabled.desc", "Pojazdy nie zwalniają już i nie zatrzymują się przed pieszymi przy wjazdach do budynków (parkingi, magazyny, sklepy...). Piesi chodzą normalnie. Wyłącz, aby przywrócić zachowanie z gry." },
				{ "mode.label", "Zakres działania" },
				{ "mode.desc", "Tylko strefy dostępu: tylko wjazdy do budynków, rampy garaży i parkingi. Wszystkie drogi (globalnie): pojazdy nigdzie nie ustępują pierwszeństwa pieszym, także na przejściach." },
				{ "mode.warning", "Tryb globalny wyłącza ustępowanie pierwszeństwa pieszym w całym mieście." },
				{ kModeAccess, "Tylko strefy dostępu" },
				{ kModeGlobal, "Wszystkie drogi (globalnie)" },
				{ "garage.label", "Rampy garażowe" },
				{ "garage.desc", "Uwzględnij rampy wjazdowe garaży i parkingów wielopoziomowych." },
				{ "conn.label", "Wjazdy do budynków" },
				{ "conn.desc", "Uwzględnij pasy między budynkami a siecią drogową (połączenia pojazdowe i piesze)." },
				{ "lot.label", "Parkingi i drogi wewnętrzne" },
				{ "lot.desc", "Uwzględnij pasy na parkingach oraz inne drogi należące do budynków." },
				{ "hotkey.enabled", "Włącz/wyłącz mod" },
				{ "hotkey.scope", "Przełącz tryb zakresu" },
				{ "bindingMap", "Skróty Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Pt()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "Geral" },
				{ "group.main", "Comportamento" },
				{ "group.scope", "Escopo dos acessos" },
				{ "enabled.label", "Veículos atravessam pedestres" },
				{ "enabled.desc", "Veículos não reduzem mais a velocidade nem param para pedestres nos acessos de edifícios (estacionamentos, armazéns, lojas...). Os pedestres continuam caminhando normalmente. Desligue para restaurar o comportamento original." },
				{ "mode.label", "Modo de escopo" },
				{ "mode.desc", "Somente acessos: apenas entradas de edifícios, rampas de garagem e estacionamentos. Todas as vias (global): veículos nunca dão preferência a pedestres, incluindo faixas de pedestres." },
				{ "mode.warning", "O modo global desativa a preferência a pedestres em toda a cidade." },
				{ kModeAccess, "Somente acessos" },
				{ kModeGlobal, "Todas as vias (global)" },
				{ "garage.label", "Rampas de garagem" },
				{ "garage.desc", "Incluir rampas de entrada de garagens e edifícios de estacionamento." },
				{ "conn.label", "Acessos de edifícios" },
				{ "conn.desc", "Incluir as faixas entre edifícios e a malha viária (conexões de veículos e de pedestres)." },
				{ "lot.label", "Estacionamentos e vias internas" },
				{ "lot.desc", "Incluir faixas dentro de estacionamentos e vias particulares de edifícios." },
				{ "hotkey.enabled", "Ativar/desativar o mod" },
				{ "hotkey.scope", "Alternar modo de escopo" },
				{ "bindingMap", "Atalhos do Access Anarchy" }
			};
		}

		private static Dictionary<string, string> Ru()
		{
			return new Dictionary<string, string>
			{
				{ "mod.name", "Access Anarchy" },
				{ "tab", "Общие" },
				{ "group.main", "Поведение" },
				{ "group.scope", "Зона действия" },
				{ "enabled.label", "Транспорт проходит сквозь пешеходов" },
				{ "enabled.desc", "Транспорт больше не замедляется и не останавливается перед пешеходами у въездов на объекты (парковки, склады, магазины...). Пешеходы ходят как обычно. Выключите, чтобы вернуть исходное поведение." },
				{ "mode.label", "Режим действия" },
				{ "mode.desc", "Только зоны въездов: только въезды на объекты, пандусы паркингов и парковки. Все дороги (глобально): транспорт нигде не уступает дорогу пешеходам, включая пешеходные переходы." },
				{ "mode.warning", "Глобальный режим отключает пропуск пешеходов по всему городу." },
				{ kModeAccess, "Только зоны въездов" },
				{ kModeGlobal, "Все дороги (глобально)" },
				{ "garage.label", "Пандусы паркингов" },
				{ "garage.desc", "Включить въездные пандусы гаражей и многоэтажных паркингов." },
				{ "conn.label", "Въезды к зданиям" },
				{ "conn.desc", "Включить полосы между зданиями и дорожной сетью (транспортные и пешеходные соединения)." },
				{ "lot.label", "Парковки и внутренние дороги" },
				{ "lot.desc", "Включить полосы внутри парковок и другие дороги на территории зданий." },
				{ "hotkey.enabled", "Включить/выключить мод" },
				{ "hotkey.scope", "Переключить режим зоны" },
				{ "bindingMap", "Горячие клавиши Access Anarchy" }
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
