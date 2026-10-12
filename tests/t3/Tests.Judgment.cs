using System;
using AccessAnarchy.Systems;

namespace AccessAnarchy.T3
{
    /// <summary>
    /// Systems\RestoreJudgment.cs 的真值表。每个分组对应 v0.8.11 里一处「只有实机才看得见」的判据，
    /// 现在全部压回离线层（用例后面标了它钉住的是哪个缺陷）。
    ///
    /// 作者实机基线（2026-10-12 会话，57km 存档，出入口模式）用来校准量级：
    /// 全城行人条目基线 2796（横道 1677）、恢复后非横道 1127、集合读数 787~921 条车道 / 1189~1353 条。
    /// </summary>
    internal static class TestsJudgment
    {
        public static void Run()
        {
            Constants();
            RestoreEnoughLegs();
            RestoreEnoughProperties();
            LedgerTrigger();
            LedgerProperties();
            LeakStreak();
            RestoreTarget();
        }

        private static void Constants()
        {
            Check.Section("阈值常量（实机验收清单引用的就是这几个数，改动必须显式）");
            Check.Equal("SetLeakConfirm", 2, RestoreJudgment.SetLeakConfirm);
            Check.Equal("RestoreMinRecover", 64, RestoreJudgment.RestoreMinRecover);
            Check.Equal("TrackedLogMin", 1024, RestoreJudgment.TrackedLogMin);
            Check.Equal("TrackedLogShift", 4, RestoreJudgment.TrackedLogShift);
            Check.Equal("TrackedLogStepAbs", 256, RestoreJudgment.TrackedLogStepAbs);
        }

        private struct Case
        {
            public string What;
            public bool Taken;
            public int Baseline;
            public int Disable;
            public int Entries;
            public bool Want;
            public string Leg;

            public Case(string what, bool taken, int baseline, int disable, int entries, bool want, string leg)
            {
                What = what;
                Taken = taken;
                Baseline = baseline;
                Disable = disable;
                Entries = entries;
                Want = want;
                Leg = leg;
            }
        }

        private static void RestoreEnoughLegs()
        {
            Check.Section("恢复判据双尺子 RestoreEnough（钉住：撕裂态仍必须被挡住 / 基线过期要能救回来）");
            Case[] cases = new[]
            {
                // 没有基线（读档后没跑到启用态）→ 保持"算够"的旧行为
                new Case("无基线：算够", false, 0, -1, 0, true, "no-baseline"),
                new Case("基线为 0：算够", true, 0, -1, 5, true, "no-baseline"),
                // 腿①：回到启用时基线的 8 成（gate = 基线*4/5，整数下取整）
                new Case("实机形态：2846 ≥ gate 2236", true, 2796, 1700, 2846, true, "baseline80"),
                new Case("恰好等于 gate", true, 2796, -1, 2236, true, "baseline80"),
                new Case("gate 差 1 条", true, 2796, -1, 2235, false, "baseline-only"),
                // 腿②：比关闭态多恢复 max(64, 基线/8) 条 —— 实机阈值 = max(64,349) = 349
                new Case("城市缩水后已恢复：+400 ≥ 349", true, 2796, 1700, 2100, true, "disable-delta"),
                new Case("临界：正好 +349", true, 2796, 1700, 2049, true, "disable-delta"),
                new Case("差 1 条：+348", true, 2796, 1700, 2048, false, "neither"),
                // 撕裂态（我们删的一条都没回来）：两把尺子都不能放行
                new Case("撕裂态：读数=关闭态", true, 2796, 1700, 1700, false, "neither"),
                new Case("撕裂态 + 城市自然长回 300 条", true, 2796, 1700, 2000, false, "neither"),
                // 小基线小图：阈值取绝对下限 64
                new Case("小图：+70 ≥ 64", true, 200, 60, 130, true, "disable-delta"),
                new Case("小图撕裂态", true, 200, 60, 60, false, "neither"),
                // 关闭态读数没采到（全城扫描失败）→ ②自动停用，退回 v0.7.8 单尺子
                new Case("缺关闭态读数：只看基线腿", true, 2796, -1, 2000, false, "baseline-only(missed-disable-reading)"),
                new Case("缺关闭态读数但够 8 成", true, 2796, -1, 2790, true, "baseline80"),
            };
            foreach (Case c in cases)
            {
                string leg;
                bool got = RestoreJudgment.RestoreEnough(c.Taken, c.Baseline, c.Disable, c.Entries, out leg);
                Check.Equal(c.What + " → 判定", c.Want, got);
                Check.Starts(c.What + " → 成立的腿", c.Leg, leg);
            }
            // 边界：gate 的算法就是「基线*4/5 下取整」，别让"8 成"变成四舍五入
            Check.Equal("gate(10)=8", true, RestoreJudgment.RestoreEnough(true, 10, -1, 8, out _));
            Check.Equal("gate(10)=8 差 1", false, RestoreJudgment.RestoreEnough(true, 10, -1, 7, out _));
            // 第二把腿在 S（我们实际删掉的条数）低于阈值时是**惰性**的：总量几乎没动，①本来就会成立。
            // 这不是缺陷，但实机日志必须能把它和"②真的起了作用"区分开——所以判据会把成立的腿打出来。
            string legInert;
            Check.Equal("S 很小：恢复成功照样算够", true, RestoreJudgment.RestoreEnough(true, 2796, 2700, 2796, out legInert));
            Check.Starts("S 很小：成立的是基线腿（②惰性）", "baseline80", legInert);
            // 反向：S 大 + 城市缩水 ⇒ ①够不到、只有②能救（②存在的意义，上面已有临界两例）
        }

        private static void RestoreEnoughProperties()
        {
            Check.Section("RestoreEnough 单调性（判据绝不能「恢复得更多反而算不够」）");
            for (int baseline = 1; baseline <= 20000; baseline += 137)
            {
                int disable = baseline / 2;
                bool prev = false;
                for (int entries = disable; entries <= baseline + 1; entries += 11)
                {
                    bool now = RestoreJudgment.RestoreEnough(true, baseline, disable, entries, out _);
                    if (!prev && now)
                    {
                        prev = true;
                    }
                    else if (prev && !now)
                    {
                        Check.True("baseline=" + baseline + " entries=" + entries + " 单调不回退", false);
                        return;
                    }
                }
            }
            Check.True("全部步长单调（1~20000 抽样）", true);

            // 关闭态读数更高（城市在关闭前膨胀过）也不能把已恢复判成没恢复：entries 够 8 成就必须过
            Check.Equal("关闭态读数高于基线仍按基线腿放行", true, RestoreJudgment.RestoreEnough(true, 2796, 3000, 2800, out _));
            // 负数输入不能炸（读数字段允许 -1 语义，其它值来自容器计数）
            string legNeg;
            RestoreJudgment.RestoreEnough(true, 2796, -1, -1, out legNeg);
            Check.Equal("entries=-1 不抛、判不够", false, RestoreJudgment.RestoreEnough(true, 2796, -1, -1, out _));
        }

        private static void LedgerTrigger()
        {
            Check.Section("Stripped-lane ledger 触发（钉住 v0.8.11 的两个反向缺陷：≥1024 每心跳重复 / 打完 0 后永远沉默）");
            Check.Equal("本局第一次（哪怕读数是 0）", true, RestoreJudgment.ShouldLogLedger(-1, 0));
            Check.Equal("本局第一次（大清单）", true, RestoreJudgment.ShouldLogLedger(-1, 5000));
            Check.Equal("一字不变 → 不写", false, RestoreJudgment.ShouldLogLedger(1500, 1500));
            Check.Equal("⚠ 缺陷 1 回归锁：稳态 ≥1024 不再每个心跳重复", false, RestoreJudgment.ShouldLogLedger(1024, 1030));
            Check.Equal("⚠ 缺陷 1 回归锁：1500→1500 不重复", false, RestoreJudgment.ShouldLogLedger(1500, 1501));
            Check.Equal("⚠ 缺陷 2 回归锁：上次 0、现在涨到 900 必须报", true, RestoreJudgment.ShouldLogLedger(0, 900));
            Check.Equal("上次 0、现在还是 0 → 不写", false, RestoreJudgment.ShouldLogLedger(0, 0));
            Check.Equal("清单被清空（起轮/定向恢复排空）是事件", true, RestoreJudgment.ShouldLogLedger(900, 0));
            Check.Equal("小清单 1% 抖动 → 不写", false, RestoreJudgment.ShouldLogLedger(900, 910));
            Check.Equal("小清单 20% → 不写（未到 1/4）", false, RestoreJudgment.ShouldLogLedger(300, 360));
            Check.Equal("小清单 25% → 写", true, RestoreJudgment.ShouldLogLedger(300, 380));
            Check.Equal("绝对步长兜底：900→1200（+300≥256）", true, RestoreJudgment.ShouldLogLedger(900, 1200));
            Check.Equal("跨过 1024 那一次 → 写", true, RestoreJudgment.ShouldLogLedger(1000, 1024));
            Check.Equal("已经在阈值上、只挪 6 条 → 不写", false, RestoreJudgment.ShouldLogLedger(1024, 1030));
            Check.Equal("大幅回落也要报（1500→1023，-477）", true, RestoreJudgment.ShouldLogLedger(1500, 1023));
            Check.Equal("软上限被清（触发 TrimWrittenLanes）→ 报 0", true, RestoreJudgment.ShouldLogLedger(131072, 0));
        }

        private static void LedgerProperties()
        {
            Check.Section("ledger 触发的两条不变量");
            // ①「没变就绝不写」：任何 (last, same) 都不能触发
            bool anySameTriggers = false;
            for (int v = 0; v <= 4000; v += 7)
            {
                if (RestoreJudgment.ShouldLogLedger(v, v))
                {
                    anySameTriggers = true;
                }
            }
            Check.Equal("tracked == 上次值 时永不触发", false, anySameTriggers);

            // ②刷屏上限：稳态不变 → 一次都不写；模拟 60 个心跳里清单在 ±2% 抖动
            int last = 2000;
            int lines = 0;
            int[] wobble = new int[] { 2000, 2030, 1970, 2040, 1960, 2010, 1990 };
            for (int i = 0; i < 60; i++)
            {
                int tracked = wobble[i % wobble.Length];
                if (RestoreJudgment.ShouldLogLedger(last, tracked))
                {
                    lines++;
                    last = tracked;
                }
            }
            Check.True("±2% 抖动 60 个心跳最多写 " + lines + " 行（应远小于心跳数）", lines <= 4);
        }

        private static void LeakStreak()
        {
            Check.Section("漏口连击计数 NextLeakStreak / LeakHealDue（自愈补剥的节奏，钉住「读干净必须立刻归零」）");
            Check.Equal("干净 + 计数 0 → 0", 0, RestoreJudgment.NextLeakStreak(false, 0));
            Check.Equal("干净 + 计数 1 → 立刻归零", 0, RestoreJudgment.NextLeakStreak(false, 1));
            Check.Equal("残留 + 计数 0 → 1", 1, RestoreJudgment.NextLeakStreak(true, 0));
            Check.Equal("残留 + 计数 1 → 2", 2, RestoreJudgment.NextLeakStreak(true, 1));
            Check.Equal("一次读数不足以自愈", false, RestoreJudgment.LeakHealDue(1));
            Check.Equal("连续两次才自愈", true, RestoreJudgment.LeakHealDue(2));
            Check.Equal("调度失败后计数留着，下一次直接达标", true, RestoreJudgment.LeakHealDue(3));
            // 交错形态：残留→干净→残留 必须重新数两次（这是"本步 ECB 回放前"那条时序的防护）
            int seen = 0;
            seen = RestoreJudgment.NextLeakStreak(true, seen);
            seen = RestoreJudgment.NextLeakStreak(false, seen);
            seen = RestoreJudgment.NextLeakStreak(true, seen);
            Check.Equal("交错：残留-干净-残留 不触发自愈", false, RestoreJudgment.LeakHealDue(seen));
        }

        private static void RestoreTarget()
        {
            Check.Section("定向恢复标的选型 ChooseRestoreTarget（P2 遗留：陈旧实体不能投进 ECB）");
            // 现实里能出现的形态（主线程在 WaitForJobs 之后采的事实，按 index+version 判存在性）
            Check.Equal("车道已删 → 放弃", RestoreTargetChoice.Skip, RestoreJudgment.ChooseRestoreTarget(false, false, false));
            Check.Equal("车道已删、Owner 引用还记着 → 仍放弃（车道死了一切免谈）", RestoreTargetChoice.Skip, RestoreJudgment.ChooseRestoreTarget(false, true, true));
            Check.Equal("车道活着、没有 Owner → 标车道自己", RestoreTargetChoice.Lane, RestoreJudgment.ChooseRestoreTarget(true, false, false));
            Check.Equal("车道活着、Owner 已被拆 → 退回标车道自己", RestoreTargetChoice.Lane, RestoreJudgment.ChooseRestoreTarget(true, true, false));
            Check.Equal("车道活着、Owner 也活着 → 标 Owner", RestoreTargetChoice.Owner, RestoreJudgment.ChooseRestoreTarget(true, true, true));
            // 传参矛盾形态（主线程不可能产生，但函数不能靠"调用者一定对"来保证安全）
            Check.Equal("没有 Owner 引用却传 ownerExists → 仍标车道", RestoreTargetChoice.Lane, RestoreJudgment.ChooseRestoreTarget(true, false, true));

            // 全 8 种输入的三条不变量
            for (int i = 0; i < 8; i++)
            {
                bool laneExists = (i & 1) != 0;
                bool hasOwnerRef = (i & 2) != 0;
                bool ownerExists = (i & 4) != 0;
                RestoreTargetChoice got = RestoreJudgment.ChooseRestoreTarget(laneExists, hasOwnerRef, ownerExists);
                // ① 车道死了就必须 Skip：ECB 对死实体操作是 SDK 不保证的行为，任何"试试看"都不行
                Check.Equal("车道已死 ⇒ Skip（输入 " + i + "）", !laneExists, got == RestoreTargetChoice.Skip);
                // ② Owner 只有在三条件齐全时才出现
                Check.Equal("Owner 需三条件齐全（输入 " + i + "）", laneExists && hasOwnerRef && ownerExists, got == RestoreTargetChoice.Owner);
                // ③ 降级链：owner 从活变死，结果只能 Owner → Lane，绝不能退到 Skip（车道不能因为 owner 死了而丢失）
                RestoreTargetChoice degraded = RestoreJudgment.ChooseRestoreTarget(laneExists, hasOwnerRef, false);
                Check.True("owner 死了也不丢车道（输入 " + i + "）", !laneExists || degraded != RestoreTargetChoice.Skip);
            }
        }
    }
}
