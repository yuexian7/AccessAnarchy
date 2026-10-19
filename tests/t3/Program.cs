using System;

namespace AccessAnarchy.T3
{
    /// <summary>
    /// Access Anarchy 的 T3 离线回归壳（开发技能 §8「不开游戏能测的全先测」）。
    ///
    /// 为什么值得为它单独建一个工程：一次实机回归 = 启动 70~95 秒 + 在 57km 存档里手动操作 +
    /// 4 小时级的会话，而这里全部断言跑完不到 0.1 秒。v0.8.11 那一轮四处改动**全部是判据算术**
    /// （恢复双尺子 / 心跳上报节奏 / 漏口连击计数），第四轮复查就是在 ledger 那一格里找到
    /// 两个方向都反的缺陷——这种形状用真值表当场就能钉死，不该留给实机轮次去发现。
    ///
    /// 硬约束（见 T3.csproj）：零游戏 DLL、零联网、零写盘、不碰部署目录；被测的
    /// Systems\RestoreJudgment.cs 一个游戏类型都不碰，模组 csproj 与本壳编同一份文件。
    ///
    /// 退出码：0 = 全绿，1 = 有失败。用法：
    ///   dotnet build tests\t3\T3.csproj -c Release
    ///   tests\t3\bin\Release\net48\AccessAnarchy.T3.exe [--verbose]
    /// 或 `node scripts\verify.mjs --t3-only`。
    /// </summary>
    internal static class Program
    {
        private static int Main(string[] argv)
        {
            bool canary = false;
            foreach (string a in argv)
            {
                if (string.Equals(a, "--verbose", StringComparison.OrdinalIgnoreCase)
                    || string.Equals(a, "-v", StringComparison.OrdinalIgnoreCase))
                {
                    Check.Verbose = true;
                }
                else if (string.Equals(a, "--canary", StringComparison.OrdinalIgnoreCase))
                {
                    canary = true;
                }
            }
            TryUtf8Output();
            Console.WriteLine("Access Anarchy T3 —— Systems/RestoreJudgment.cs 纯判据回归（零游戏 DLL / 零联网 / 不开游戏）");
            Console.WriteLine("被测源码与模组 csproj 同一份文件：Systems\\RestoreJudgment.cs");

            if (canary)
            {
                // 自检：壳必须**能红**。故意断一条错的，只有恰好记到 1 条 FAIL 才算壳本身是活的
                // （"永远全绿的壳"和"没有壳"等价——这是本项目被幻觉壳骗过一次的原因）。
                Check.Section("canary（故意失败，用来证明断言台会红）");
                Check.Equal("canary：1 应该等于 2", 1, 2);
                bool ok = Check.Fail == 1;
                Console.WriteLine("CANARY " + (ok ? "OK（失败确实被记录，退出码本应为 1）" : "BROKEN（断言台没把失败记下来！）"));
                Console.WriteLine("TOTAL " + Check.Total + " PASS " + Check.Pass + " FAIL " + Check.Fail);
                return ok ? 1 : 2;
            }

            TestsJudgment.Run();
            TestsLogGuard.Run();

            Check.DumpFailures();
            Console.WriteLine("TOTAL " + Check.Total + " PASS " + Check.Pass + " FAIL " + Check.Fail);
            return Check.Fail == 0 ? 0 : 1;
        }

        /// <summary>控制台默认按 GBK 解码，中文会糊成乱码；这里尽力切 UTF-8，失败也不影响判定。</summary>
        private static void TryUtf8Output()
        {
            try
            {
                Console.OutputEncoding = System.Text.Encoding.UTF8;
            }
            catch (Exception)
            {
            }
        }
    }
}
