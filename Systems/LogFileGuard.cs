using System;
using System.IO;

namespace AccessAnarchy.Systems
{
	/// <summary>
	/// v0.8.11：日志文件不可写时的**自保闸**。
	///
	/// 背景是 Colossal.Logging 框架里的一个潜在缺陷（反编译 Colossal.Logging.dll 逐行核实）：
	/// `UnityLogger.Open()` 用 `try { new FileStream(...) } catch { Close(); }` 包住建流，
	/// 而 `Close()` 的守卫是 `if (isOpen)`，`isOpen` 又定义为 `m_Stream != null &amp;&amp; m_StreamWriter != null`。
	/// 于是**建流失败时 `Close()` 什么也不做**，`m_StreamWriter` 留在 null；
	/// 紧接着 `Internal_WriteStream` 第一句就是 `m_StreamWriter.SetSourceAndStd(...)` → NullReferenceException。
	///
	/// 触发条件全在玩家环境侧（日志文件被别的程序占着）：日志查看器/加速器把文件开着、OneDrive
	/// 同步中、杀软锁定、只读属性、权限被改。此时 `isOpen` 恒为 false，**每次写日志都重开→再失败→再抛**。
	/// 异常最终被 `CustomLogHandler.LogFormat` 的 `catch (Exception)` 吞掉（所以功能照常，只是日志没落地），
	/// 而它顺手 `LogException` 把这条 NRE 送进 Unity 默认 handler → 玩家 Player.log 里出现成串堆栈。
	/// 模组本身没错，但玩家看到的是"这个模组在刷异常"。
	///
	/// 修法用**框架自带**的降级开关，不抢补丁、不吞日志：
	/// `ILog.redirectToDefault = true` 之后 `CustomLogHandler.PostProcessFormat` 会在
	/// `!LogManager.stdOutActive` 时于 `Internal_WriteStream` **之前** return，坏掉的流从此不被触碰；
	/// 同时它仍把 `[{logger}] {正文}` 写进 Unity 默认 handler → 诊断搬去 Player.log，一条不少。
	///
	/// 为什么单独一个类：这里一个游戏类型都不碰，`tests\t3` 离线壳能直接编进去钉回归
	/// （开发技能 §8「能在下层测出来的绝不带到上层」）。判定与副作用刻意分开——
	/// 本类只回答「文件现在能不能写」，改 `redirectToDefault` 和发警告留给入口处，
	/// 因为那两件事要碰 `ILog`，离线壳没有它。
	/// </summary>
	internal static class LogFileGuard
	{
		/// <summary>日志文件当前可写，框架那条坏路径不会被踩到。</summary>
		internal const int Writable = 0;

		/// <summary>日志文件写不进去，必须改用默认 handler（Player.log）。</summary>
		internal const int UseDefaultHandler = 1;

		/// <summary>
		/// 只探查、不写内容地判断 `path` 能不能**追加**写入。
		///
		/// 刻意用 `FileMode.Append + FileShare.ReadWrite`，理由是它必须**如实复现**框架那次开流：
		///  · `Append` 不截断文件——绝不可以用 `Create`，那会把玩家已有的日志清空；
		///  · `FileShare.ReadWrite` 与 `Open()` 里 `new FileStream(..., FileShare.ReadWrite)` 同参数，
		///    所以"别人占着文件"在两边表现一致，不会出现探查过了、框架照样失败；
		///  · 这条路径**不写入任何字节**（拿到句柄就立刻关），对玩家日志无副作用。
		///
		/// 返回 <see cref="Writable"/> 或 <see cref="UseDefaultHandler"/>。本方法**永不抛异常**：
		/// 判据本身不该成为新的崩溃点。空路径/目录创建失败都归到"不可写"。
		/// </summary>
		internal static int Check(string logPath)
		{
			if (string.IsNullOrEmpty(logPath))
			{
				return UseDefaultHandler;
			}
			try
			{
				// 框架 `Open()` 第一件事就是 Directory.CreateDirectory(Path.GetDirectoryName(logPath))，
				// 这里同样做掉：目录本身不存在也算"当前写不了"，正好一起覆盖。
				string directory = Path.GetDirectoryName(logPath);
				if (!string.IsNullOrEmpty(directory))
				{
					Directory.CreateDirectory(directory);
				}
				using (new FileStream(logPath, FileMode.Append, FileAccess.Write, FileShare.ReadWrite))
				{
				}
				return Writable;
			}
			catch (Exception)
			{
				// 包括 IOException / UnauthorizedAccessException / ArgumentException 等一切形状：
				// 只要不是"能打开"，就走默认 handler。宁可少写一个文件，也不让玩家看到成串 NRE。
				return UseDefaultHandler;
			}
		}
	}
}
