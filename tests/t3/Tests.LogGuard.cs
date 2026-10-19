using System;
using System.IO;
using AccessAnarchy.Systems;

namespace AccessAnarchy.T3
{
    /// <summary>
    /// Systems\LogFileGuard.cs 的判定表（钉住 v0.8.11 修的「坏日志文件让框架刷 NullReferenceException」）。
    ///
    /// 与 TestsJudgment 的区别：这一组的判定对象是**文件系统状态**，穷举不了；但三类输入必须钉死——
    ///   ① 空/坏路径要归到降级；
    ///   ② 正常可写文件要判成可写（写成「永远返回降级」的桩会通过全部 ①，却让所有玩家白丢文件日志）；
    ///   ③ **文件被别的程序占着**要判成降级 —— 这就是玩家那台机器的原始条件，用来反证修法对症。
    ///
    /// 仍然守 T3 的硬约束：零游戏 DLL、零联网、**零写盘**、不碰部署目录。夹具全用**已存在**的文件：
    ///   正例 = 本壳自己的 `T3.csproj`（`FileMode.Append` 不截断，且探查一个字节都不写，前后逐字节比过）；
    ///   负例甲 = 同一文件被另一个**只读**句柄用 `FileShare.Read` 占着 —— 日志查看器/同步盘的典型开法
    ///            （允许别人读、拒绝别人写），与玩家侧触发条件同形；
    ///   负例乙 = 正在运行的**本壳 exe**（被系统当镜像占用），与「游戏 DLL 被占用」同形。
    ///
    /// 为什么没有 `NUL`：2026-10-19 实测 `Check("NUL")` 判成降级（那步开流在设备上失败），
    /// 设备不能当正例，只挪进畸形输入表要求「不抛 + 落在两态之一」。
    /// 这是这条测试原本写法失败后留下的结论，别再改回去。
    /// </summary>
    internal static class TestsLogGuard
    {
        public static void Run()
        {
            Constants();
            DegeneratePaths();
            WritableRealFileIsNotDowngraded();
            HeldByAnotherProgram();
            ImageInUseIsDowngraded();
            HostileInputsNeverThrow();
        }

        private static void Constants()
        {
            Check.Section("日志自保闸常量（0=可写 / 1=改走默认 handler，两个值必须不同）");
            Check.Equal("Writable", 0, LogFileGuard.Writable);
            Check.Equal("UseDefaultHandler", 1, LogFileGuard.UseDefaultHandler);
            Check.True("两个取值不相等（相等就是「恒真」判据，修不了任何东西）",
                LogFileGuard.Writable != LogFileGuard.UseDefaultHandler);
        }

        /// <summary>①：拿不到路径、空串/空白、指向目录、父级是个文件 —— 一律降级，且不许抛。</summary>
        private static void DegeneratePaths()
        {
            Check.Section("日志自保闸：坏路径必须降级（钉住：降级要覆盖「取不到路径」和「目录建不出来」）");
            Check.Equal("null 路径 → 降级", LogFileGuard.UseDefaultHandler, LogFileGuard.Check(null));
            Check.Equal("空串 → 降级", LogFileGuard.UseDefaultHandler, LogFileGuard.Check(""));
            Check.Equal("纯空白 → 降级", LogFileGuard.UseDefaultHandler, LogFileGuard.Check("   "));
            Check.Equal("指向一个目录 → 降级", LogFileGuard.UseDefaultHandler,
                LogFileGuard.Check(AppDomain.CurrentDomain.BaseDirectory));

            string csproj = FindShellProject();
            if (csproj != null)
            {
                // 父级是文件 → Directory.CreateDirectory 必然失败（框架 Open() 的第一句就是它）
                Check.Equal("父级是个文件 → 降级", LogFileGuard.UseDefaultHandler,
                    LogFileGuard.Check(Path.Combine(csproj, "AccessAnarchyMod.log")));
            }
            else
            {
                Check.True("找不到 T3.csproj 当夹具（父级是文件这条判定不了）", false);
            }
        }

        /// <summary>②：可写文件必须判成可写，且探查**不许动文件内容**（反证桩实现 + 反证用了 FileMode.Create）。</summary>
        private static void WritableRealFileIsNotDowngraded()
        {
            Check.Section("日志自保闸：可写文件判成可写 + 探查不改写它（钉住：绝不能用 FileMode.Create 去试）");
            string csproj = FindShellProject();
            if (csproj == null)
            {
                Check.True("找不到 T3.csproj 当夹具（正例与不改写两条判定不了）", false);
                return;
            }
            byte[] before = File.ReadAllBytes(csproj);
            int result = LogFileGuard.Check(csproj);
            byte[] after = File.ReadAllBytes(csproj);

            Check.Equal("未被占用的真实文件 → 可写（恒返回降级就是桩）", LogFileGuard.Writable, result);
            Check.True("探查前后字节数不变（" + before.Length + " → " + after.Length + "，截断=清空玩家日志）",
                before.Length == after.Length);
            bool same = before.Length == after.Length;
            for (int i = 0; same && i < before.Length; i++)
            {
                same = before[i] == after[i];
            }
            Check.True("探查前后内容逐字节相同", same);
        }

        /// <summary>③甲：被别的程序以「只读共享」占着 —— 玩家那台机器的原始条件，必须降级。</summary>
        private static void HeldByAnotherProgram()
        {
            Check.Section("日志自保闸：被别的程序占着要降级（钉住玩家侧触发条件，FileShare.Read）");
            string csproj = FindShellProject();
            if (csproj == null)
            {
                Check.True("找不到 T3.csproj 当夹具（占用场景判定不了）", false);
                return;
            }
            using (new FileStream(csproj, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                // 先证明夹具不是空的：同一个句柄确实挡住了「以写方式开」（框架 Open() 就是这么开的）。
                bool blocked;
                try
                {
                    using (new FileStream(csproj, FileMode.Append, FileAccess.Write, FileShare.ReadWrite)) { }
                    blocked = false;
                }
                catch (Exception)
                {
                    blocked = true;
                }
                Check.True("夹具有效性：该锁确实挡住了写入方（模拟不是空的）", blocked);
                Check.Equal("被占用 → 降级", LogFileGuard.UseDefaultHandler, LogFileGuard.Check(csproj));
            }
        }

        /// <summary>③乙：文件被系统当镜像占用（运行中的本壳 exe）—— 与「游戏 DLL 被占用」同形。</summary>
        private static void ImageInUseIsDowngraded()
        {
            Check.Section("日志自保闸：文件被当镜像占用要降级（运行中的本壳 exe，同「游戏 DLL 被占用」形状）");
            string exe = System.Reflection.Assembly.GetExecutingAssembly().Location;
            if (string.IsNullOrEmpty(exe) || !File.Exists(exe))
            {
                Check.True("拿不到本壳 exe 路径（镜像占用这条判定不了）", false);
                return;
            }
            Check.Equal("运行中的 exe → 降级", LogFileGuard.UseDefaultHandler, LogFileGuard.Check(exe));
        }

        /// <summary>判据本身不许成为新的崩溃点：任何畸形输入只允许返回两个取值之一。
        ///
        /// ⚠ 这里还要守**零写盘**：`Check` 的探查是 `FileMode.Append`（与框架 `Open()` 同参数），
        /// 而 `Append` 会**创建**不存在的文件——生产里唯一入参是 `log.logPath`，创建它正是框架下一步要干的事，
        /// 无害；但畸形输入表里若有「规范化后仍可创建」的相对路径，就会在**工作目录留下垃圾文件**。
        /// 2026-10-19 真踩到两次：`"   NUL   "` 被 Win32 吃掉尾部空格变成普通文件名、在仓库根落了个 0 字节文件；
        /// 换上的 `"nul.log"` 更是普通文件名（只有裸 `NUL` 这类才解析成设备，带扩展名的照样建文件）。
        /// 两次都被下面的零写盘金丝雀逮住 ⇒ 新增输入前先想一遍：它规范化之后**还能不能创建**。</summary>
        private static void HostileInputsNeverThrow()
        {
            Check.Section("日志自保闸：畸形输入既不许抛也不许返回第三态，且不许落下文件");
            string[] nasty =
            {
                null, "", " ", "\t", "\0",
                // 设备名：裸 `NUL` / 尾部只剩空格（Win32 吃掉空格后又落回设备名）都开不出来，安全。
                "NUL", "NUL   ", "con", "aux", "com1",
                "C:\\a\0b.log",
                "\\\\?\\",
                ">>><<<",
                Path.Combine(AppDomain.CurrentDomain.BaseDirectory, new string('x', 300) + ".log"),
            };
            string[] cwdBefore = SnapshotDirectory(Environment.CurrentDirectory);
            string[] baseBefore = SnapshotDirectory(AppDomain.CurrentDomain.BaseDirectory);

            foreach (string input in nasty)
            {
                string label = input == null ? "null" : (input.Length > 24 ? input.Substring(0, 24) + "…" : input);
                int got;
                try
                {
                    got = LogFileGuard.Check(input);
                }
                catch (Exception ex)
                {
                    Check.True("输入「" + label + "」不许抛异常，却抛了 " + ex.GetType().Name, false);
                    continue;
                }
                Check.True("输入「" + label + "」返回值只能是 0 或 1（实际 " + got + "）",
                    got == LogFileGuard.Writable || got == LogFileGuard.UseDefaultHandler);
            }

            // 零写盘金丝雀：畸形输入不许在 cwd / 壳目录里留下任何新文件。
            Check.True("畸形输入跑完，工作目录没多出文件（零写盘）", SameSet(cwdBefore, SnapshotDirectory(Environment.CurrentDirectory)));
            Check.True("畸形输入跑完，壳目录没多出文件（零写盘）", SameSet(baseBefore, SnapshotDirectory(AppDomain.CurrentDomain.BaseDirectory)));
        }

        private static string[] SnapshotDirectory(string dir)
        {
            try
            {
                string[] files = Directory.GetFiles(dir);
                Array.Sort(files, StringComparer.OrdinalIgnoreCase);
                return files;
            }
            catch (Exception)
            {
                return new string[0];
            }
        }

        private static bool SameSet(string[] a, string[] b)
        {
            if (a.Length != b.Length)
            {
                return false;
            }
            for (int i = 0; i < a.Length; i++)
            {
                if (!string.Equals(a[i], b[i], StringComparison.OrdinalIgnoreCase))
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>定位本壳自己的 T3.csproj：从 exe 所在目录往上找（bin 与 obj 两种深度都试）。</summary>
        private static string FindShellProject()
        {
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            string[] candidates =
            {
                Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "T3.csproj")),
                Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "..", "T3.csproj")),
                Path.GetFullPath(Path.Combine(Environment.CurrentDirectory, "tests", "t3", "T3.csproj")),
                Path.GetFullPath(Path.Combine(Environment.CurrentDirectory, "T3.csproj")),
            };
            foreach (string c in candidates)
            {
                if (File.Exists(c))
                {
                    return c;
                }
            }
            return null;
        }
    }
}
