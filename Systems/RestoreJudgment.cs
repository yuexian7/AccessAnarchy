using System;

namespace AccessAnarchy.Systems
{
    /// <summary>
    /// v0.8.11 收尾补丁（2026-10-13 第四轮复查 + P2 遗留）：把四处**纯判据**从 <see cref="AccessZoneOverlapSystem"/>
    /// 里抽出来放在这里——这个类一个游戏类型都不碰（硬约束，见 <c>tests\t3\T3.csproj</c> 的注释：
    /// 一旦有人往这里加一句 `Unity.Mathematics.math` 或 `ComponentLookup`，离线壳会直接编译失败，
    /// 而不是静默把覆盖降级掉）。
    ///
    /// 为什么值得单独成文件：这一版的改动全部是"判据算术"（恢复判据双尺子 / 心跳上报节奏 /
    /// 漏口自愈计数 / 定向恢复标的选型），而它们此前只能靠 4.5 小时的实机会话去验。第四轮复查就是在
    /// ledger 那一格里找到两个方向都反的缺陷（大清单每心跳重复、小清单打完 0 之后永远沉默），
    /// 这种形状用真值表 0.1 秒就能钉死，不该占用实机轮次。
    ///
    /// 同一份源文件被模组 csproj（默认通配 `Systems\**\*.cs`）与 `tests\t3\T3.csproj`
    /// （`<Compile Include>`）两边编，不存在"两份拷贝漂移"这个最大的假绿来源。
    /// 阈值常量在这里是**唯一真源**；系统里如今只剩 `kSetLeakConfirm` 一个别名（它要在
    /// `<see cref>` 与日志插值里被引用），其余常量在系统侧直接用本类全名。
    /// </summary>
    internal static class RestoreJudgment
    {
        /// <summary>恢复判据第二把尺子的绝对下限（条）。见 <see cref="RestoreEnough"/>。</summary>
        internal const int RestoreMinRecover = 64;

        /// <summary>定向恢复清单（m_WrittenLanes）的心跳上报阈值——"跨过"它的**那一次**才写。</summary>
        internal const int TrackedLogMin = 1024;

        /// <summary>相对变化幅度分母（1/4 = 变化 25% 起报）。</summary>
        internal const int TrackedLogShift = 4;

        /// <summary>小清单的比例兜底：以 25% 算比例永远够不到时，用这个绝对步长起报
        /// （几十条的清单每次心跳抖三四条，按 25% 判就永远不打——"恢复成本可见"就落空了）。</summary>
        internal const int TrackedLogStepAbs = 256;

        /// <summary>`Access set audit` 连续几次读到残留才现场补剥一次。</summary>
        internal const int SetLeakConfirm = 2;

        /// <summary>
        /// 恢复判据的"总量"口径，两条腿并列：
        /// ① 回到**启用时**基线的 8 成（v0.7.8 原判据，留 2 成给期间修路/拆路）；
        /// ② 比**关闭那一刻**的实测条目多出 `max(<see cref="RestoreMinRecover"/>, 基线/8)` 条——
        ///    治"玩家中途修路/拆建之后 ① 那把尺子过期"（城市缩小时 ① 会把已恢复误判成没恢复）。
        ///
        /// 三态与措辞（第四轮复查把话讲准）：
        ///  · ②只会让判据**变松**（OR 的一支），它不可能造成新的"误判成已恢复"以外的后果；
        ///  · 撕裂态（我们删的条目一条都没回来）下 ② 一定不成立；此时能不能继续挡住，
        ///    取决于 ① 这把旧尺子——与 v0.7.8 的防护强度**完全相同**（没有放松，也没有加强）；
        ///  · `disableEntries &lt; 0` = 关闭态那份读数没采到（全城扫描失败），②自动停用，退回旧行为。
        ///
        /// 可测性说明：这里刻意只吃四个整数，不读任何模组状态，所以真值表能整条覆盖。
        /// </summary>
        /// <param name="leg">人可读的"是哪条腿成立"，直接进日志——实机才能一眼判断 ② 到底有没有用上。</param>
        internal static bool RestoreEnough(bool baselineTaken, int baselineEntries, int disableEntries, int entries, out string leg)
        {
            if (!baselineTaken || baselineEntries <= 0)
            {
                leg = "no-baseline";
                return true;
            }
            int baselineGate = (baselineEntries * 4) / 5;
            if (entries >= baselineGate)
            {
                leg = "baseline80(" + entries + "/" + baselineGate + ")";
                return true;
            }
            if (disableEntries >= 0)
            {
                int minRecover = Math.Max(RestoreMinRecover, baselineEntries / 8);
                int delta = entries - disableEntries;
                if (delta >= minRecover)
                {
                    leg = "disable-delta(+" + delta + "/" + minRecover + ")";
                    return true;
                }
                leg = "neither(baseline" + entries + "/" + baselineGate + " disable+" + delta + "/" + minRecover + ")";
                return false;
            }
            leg = "baseline-only(missed-disable-reading)";
            return false;
        }

        /// <summary>
        /// 心跳里 `Stripped-lane ledger` 该不该写这一行。四个口径，全部是**事件**而不是状态：
        /// ① 本局还没写过；② 清单被清空 / 从零重新起账（这两件都是真事件）；
        /// ③ 相对变化 ≥ 1/<see cref="TrackedLogShift"/>，小清单用 <see cref="TrackedLogStepAbs"/> 绝对步长兜底；
        /// ④ **跨过** <see cref="TrackedLogMin"/> 的那一次。
        ///
        /// 为什么不是"≥1024 就打"（v0.8.11 原实现）：那是状态不是事件——全局模式的清单
        /// 从不按轮清空（见系统里 `ClearWrittenLanes` 只在出入口起轮处调用），一旦越过 1024 就会
        /// **每个心跳**重复同一行直到本局结束（作者实机一个心跳 = 1800 步 ≈ 4~8 秒）；
        /// 而反向的缺陷同样真实：出入口稳态清单常在几百条，第一次打了 0 之后
        /// "上次值 = 0" 会让所有相对判据永远不成立，之后再涨到几百条也一行不打，上报目标直接落空。
        /// </summary>
        internal static bool ShouldLogLedger(int lastLogged, int tracked)
        {
            if (lastLogged < 0)
            {
                return true;
            }
            if (tracked == lastLogged)
            {
                return false;
            }
            if (tracked == 0 || lastLogged == 0)
            {
                return true;
            }
            int delta = tracked > lastLogged ? tracked - lastLogged : lastLogged - tracked;
            if (delta >= TrackedLogStepAbs)
            {
                return true;
            }
            int lo = tracked < lastLogged ? tracked : lastLogged;
            if (delta * TrackedLogShift >= lo)
            {
                return true;
            }
            return tracked >= TrackedLogMin && lastLogged < TrackedLogMin;
        }

        /// <summary>漏口连续计数怎么走：读到残留 +1，读干净立刻归零（一次读数不证明漏口持续存在）。</summary>
        internal static int NextLeakStreak(bool leakSeen, int seen)
        {
            return leakSeen ? seen + 1 : 0;
        }

        /// <summary>攒够 <see cref="SetLeakConfirm"/> 次连续读数就该现场补剥一次。</summary>
        internal static bool LeakHealDue(int seen)
        {
            return seen >= SetLeakConfirm;
        }

        /// <summary>
        /// 定向恢复标的的选型（P2 遗留 + 2026-10-13 收尾补丁）。判断规则见
        /// <see cref="RestoreTargetChoice"/> 的注释；三个输入是主线程在 WaitForJobs() 之后采的
        /// 可信事实（EntityManager.Exists 按 index+version 判），所以这张真值表可以被离线壳整条钉死。
        /// </summary>
        internal static RestoreTargetChoice ChooseRestoreTarget(bool laneExists, bool hasOwnerRef, bool ownerExists)
        {
            if (!laneExists)
            {
                return RestoreTargetChoice.Skip;
            }
            if (!hasOwnerRef)
            {
                return RestoreTargetChoice.Lane;
            }
            return ownerExists ? RestoreTargetChoice.Owner : RestoreTargetChoice.Lane;
        }
    }

    /// <summary>
    /// 定向恢复里"这个标的要不要标、标在哪"（P2 遗留 + 2026-10-13 收尾补丁）。
    /// 陈旧实体问题的根因：`m_WrittenLanes` 是跨帧累计的（入口处进图时就在记），
    /// 玩家"标记后立刻拆路/拆建筑"再关模组，清单里就是一堆死引用；直接把死引用投进 ECB，
    /// 帧末回放对不存在实体做操作是 SDK 不保证的行为（轻则游戏打一条 Critical、重则整批回放中断）。
    /// <see cref="RestoreJudgment.ChooseRestoreTarget"/> 的三个输入都是主线程在 `WaitForJobs()`
    /// 之后采的**可信事实**（EntityManager.Exists 按 index+version 判，不是"看起来还在"）：被删的
    /// 车道直接放弃；车道活着但没有可用的 Owner ⇒ 标车道自己；Owner 已被拆 ⇒ 退回标车道自己。
    /// 唯一没被覆盖的窗口是"过滤后、帧末回放前"被同帧拆除——那是游戏自己的拆除序列，与本模组无关，
    /// 而且现实中一次模拟步内先拆后开关的概率趋近于零。
    /// </summary>
    internal enum RestoreTargetChoice
    {
        Skip,
        Lane,
        Owner,
    }
}
