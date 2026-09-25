namespace AccessAnarchy
{
	/// <summary>
	/// 模组身份信息唯一来源：选项界面「关于」板块与启动日志都读这里，
	/// 避免版本号散落在 AccessAnarchyMod.OnLoad 的日志字符串里
	/// （v0.7.9 就出现过产物已发 0.7.9、启动日志仍写 v0.7.8 的情况）。
	/// 发布时 Properties\PublishConfiguration.xml 的 ModVersion 必须与 kVersion 一致。
	/// </summary>
	public static class ModInfo
	{
		/// <summary>模组版本号（不带 v 前缀，与平台 ModVersion 同格式）。</summary>
		public const string kVersion = "0.8.0";

		/// <summary>作者署名，按作者本人要求逐字使用。</summary>
		public const string kAuthor = "yuexian";

		/// <summary>三个链接按钮共用一个按钮组：同组按钮被合并成一行显示
		/// （AutomaticSettings 按 SettingsUIButtonGroupAttribute.name 归并，
		/// 原版先例 Game.Settings.ModdingSettings 的 toolchainAction 四个按钮）。</summary>
		public const string kLinkGroup = "AccessAnarchyLinks";

		public const string kKoFiUrl = "https://ko-fi.com/yuexian7";

		public const string kForumUrl = "https://forum.paradoxplaza.com/forum/threads/access-anarchy.1941285/latest";

		public const string kRainbowUrl = "https://rainbow-series-hvpma89wi25.qoder.zone/#top";
	}
}
