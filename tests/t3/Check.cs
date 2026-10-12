using System;
using System.Collections.Generic;

namespace AccessAnarchy.T3
{
    /// <summary>
    /// 极简断言台：只做「计数 + 失败打一行 + 决定退出码」这一件事。
    /// 形状沿用兄弟模组的 T3 壳（ZoneSnapper / BridgeTheLanguageGap），同一套门禁脚本能同样地解析它。
    ///
    /// 口径：
    ///  · 失败行必须带「用例名 / 期望 / 实际」，不看代码也能读懂；
    ///  · 断言本身抛异常 = 这条 FAIL（不是整个壳崩）——"判据在某个输入下抛"与"判据算错了"
    ///    是同等重要的缺陷，绝不能因为一条崩掉就丢掉后面的信息；
    ///  · 默认只打失败行 + 最后一行总计（`TOTAL n PASS m FAIL k`），`--verbose` 才逐条打。
    /// </summary>
    internal static class Check
    {
        public static bool Verbose;
        public static string Group = "";
        private static int _pass;
        private static int _fail;
        private static int _total;
        private static readonly List<string> Failures = new List<string>();

        public static int Pass => _pass;
        public static int Fail => _fail;
        public static int Total => _total;

        public static void Section(string name)
        {
            Group = name;
            if (Verbose)
            {
                Console.WriteLine("== " + name);
            }
        }

        public static void True(string what, bool ok)
        {
            Assert(what, "true", ok ? "true" : "false", ok);
        }

        public static void Equal(string what, object expected, object actual)
        {
            bool ok = expected == null ? actual == null : expected.Equals(actual);
            Assert(what, Convert.ToString(expected, System.Globalization.CultureInfo.InvariantCulture),
                Convert.ToString(actual, System.Globalization.CultureInfo.InvariantCulture), ok);
        }

        /// <summary>字符串前缀断言：用于「成立的是哪条腿」这类带数字尾巴的日志标签。</summary>
        public static void Starts(string what, string prefix, string actual)
        {
            bool ok = actual != null && actual.StartsWith(prefix, StringComparison.Ordinal);
            Assert(what, prefix + "*", actual, ok);
        }

        private static void Assert(string what, string expected, string actual, bool ok)
        {
            _total++;
            if (ok)
            {
                _pass++;
                if (Verbose)
                {
                    Console.WriteLine("  PASS " + what);
                }
                return;
            }
            _fail++;
            string line = "  FAIL [" + Group + "] " + what + " | expected=" + expected + " actual=" + actual;
            Failures.Add(line);
            Console.WriteLine(line);
        }

        public static void DumpFailures()
        {
            if (Failures.Count == 0)
            {
                return;
            }
            Console.WriteLine("---- 失败清单（" + Failures.Count + "）----");
            foreach (string f in Failures)
            {
                Console.WriteLine(f.Trim());
            }
        }
    }
}
