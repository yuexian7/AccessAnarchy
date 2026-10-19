#!/usr/bin/env node
/*
 * Access Anarchy 离线门禁（开发技能 §8：能在下层测出来的绝不带到上层）。
 *
 * 用法：
 *   node scripts/verify.mjs                 完整：版本一致性 → T1 构建 → T2 产物字面量 → T3 回归壳 → 发布素材 → PDB 卫生
 *   node scripts/verify.mjs --t3-only       只跑 T3（不开游戏、不构建模组，约 2 秒）
 *   node scripts/verify.mjs --skip-build    跳过 dotnet build，直接扫上一次构建的产物
 *
 * 退出码非 0 = 门禁没过，**别发布**。每条检查失败都会说清楚期望与实际。
 *
 * 设计口径：
 *  · T2 只扫**工作区内**的 bin\Release\net48\AccessAnarchy.dll（工作树即真值）。
 *    部署由 msbuild 的 DeployWIP 完成，脚本只从构建回显里确认那句 "Copy output to deploy directory"
 *    出现过——不去读游戏目录（那是工作区外的路径，agent 环境里可能被权限拦，且 inode 与联接目标同一份）。
 *  · 字面量检查必须带一根「肯定存在」的对照探针（本模组名），否则整段扫不到东西时会"假绿"。
 *  · DLL 字符串分两个堆：字面量 #US 是 UTF-16LE（**奇偶两个对齐都要扫**），符号名 #Strings 是 UTF-8。
 *    普通 grep 两头都漏，所以这里显式两种编码各找一次。
 */
import { execFileSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const args = process.argv.slice(2);
const T3_ONLY = args.includes("--t3-only");
const SKIP_BUILD = args.includes("--skip-build") || T3_ONLY;

let failures = 0;
function ok(msg) { console.log("  PASS  " + msg); }
function bad(msg) { failures++; console.log("  FAIL  " + msg); }
function section(t) { console.log("\n== " + t); }

function readText(p) { return fs.readFileSync(p, "utf8"); }

/** 双堆双对齐扫字面量：#US（UTF-16LE，偏移 0/1 各解一次）+ 符号名（UTF-8）。 */
function dllHas(buf, text) {
  for (const enc of ["utf16le", "utf8"]) {
    const needle = Buffer.from(text, enc);
    if (buf.indexOf(needle) >= 0) return true;
    if (enc === "utf16le") {
      // #US 堆里的字符串可以落在奇数字节偏移上：单对齐解码会系统性假阴性
      const off = buf.subarray(1).indexOf(needle);
      if (off >= 0) return true;
    }
  }
  return false;
}

/* ---------- 1. 版本一致性（N 处必须同源，不靠人记） ---------- */
section("1. 版本字面量一致性");
const modInfo = readText(path.join(ROOT, "ModInfo.cs"));
const kv = modInfo.match(/kVersion\s*=\s*"([^"]+)"/);
if (!kv) bad("ModInfo.cs 里找不到 kVersion");
const version = kv ? kv[1] : "unknown";
const xml = readText(path.join(ROOT, "Properties", "PublishConfiguration.xml"));
const mv = xml.match(/<ModVersion Value="([^"]+)"\s*\/>/);
const clHead = xml.match(/<ChangeLog>\s*[\r\n]+v([0-9][^\r\n]*)/);
console.log(`  kVersion=${version} ModVersion=${mv ? mv[1] : "缺"} ChangeLog头=v${clHead ? clHead[1].trim() : "缺"}`);
if (!mv) bad("PublishConfiguration.xml 缺 <ModVersion>");
else if (mv[1] !== version) bad(`ModVersion ${mv[1]} != kVersion ${version}（发布时线上版本号会按 xml 走，启动日志按 kVersion 走 → 玩家报的版本对不上）`);
else ok("ModVersion == kVersion");
if (!clHead) bad("ChangeLog 第一行不是 vX.Y.Z 形式的版本头");
else if (clHead[1].trim() !== version) bad(`ChangeLog 头 v${clHead[1].trim()} != 当前版本 ${version}（线上一个平台版本一条条目）`);
else ok("ChangeLog 头 == 当前版本");
if (!/<ModId Value="158842"/.test(xml)) bad("<ModId> 丢了/被改了——它是发布身份，丢了会导致重复上架");
else ok("ModId 158842 在位");
if (/\r/.test(xml)) bad("PublishConfiguration.xml 含 CR（工作区 XML 要保持 LF）");
else ok("xml 纯 LF");

/* ---------- 2. T1 构建 ---------- */
const PRODUCT = path.join(ROOT, "bin", "Release", "net48", "AccessAnarchy.dll");
let buildOut = "";
if (!SKIP_BUILD) {
  section("2. T1 dotnet build -c Release");
  try {
    buildOut = execFileSync("dotnet", ["build", "-c", "Release", "--nologo"], {
      cwd: ROOT, encoding: "utf8", maxBuffer: 32 * 1024 * 1024,
    });
  } catch (e) {
    bad("构建失败（退出码非 0）：" + String(e.message).split("\n")[0]);
    buildOut = String(e.stdout || "") + String(e.stderr || "");
  }
  const warn = (buildOut.match(/(\d+)\s*个?警告/) || [])[1];
  const err = (buildOut.match(/(\d+)\s*个?错误/) || [])[1];
  if (err === undefined) bad("构建回显里读不到错误数，判定不了（别当成功）");
  else if (err !== "0") bad(`构建有 ${err} 个错误`);
  else if (warn !== "0") bad(`构建 0 错误但 ${warn} 个警告（本项目基线是 0 警告）`);
  else ok("0 错误 0 警告");
  if (/Copy output to deploy directory/.test(buildOut)) ok("DeployWIP 已把产物复制到游戏 Mods 目录（按构建回显判定）");
  else if (err === "0") bad("构建成功但回显里没有部署行——游戏目录里可能还是旧版（先关游戏再构建）");
} else {
  section("2. T1 构建（已跳过）");
}

/* ---------- 3. T2 产物字面量 ---------- */
if (fs.existsSync(PRODUCT)) {
  section("3. T2 产物字面量（bin/Release/net48/AccessAnarchy.dll，工作树即真值）");
  const buf = fs.readFileSync(PRODUCT);
  const ageMin = (Date.now() - fs.statSync(PRODUCT).mtimeMs) / 60000;
  console.log(`  ${buf.length} 字节，mtime ${fs.statSync(PRODUCT).mtime.toISOString()}（${ageMin.toFixed(1)} 分钟前）`);
  if (!dllHas(buf, "Access Anarchy")) bad("对照探针缺失：连模组名都扫不到 = 编码/路径找错，本轮结论不可信");
  else ok("对照探针「Access Anarchy」在位");
  if (!dllHas(buf, version)) bad(`本版字面量 ${version} 不在产物里（产物不是本次源码编的？）`);
  else ok(`本版字面量 ${version} 在位`);
  // 上一版残留检查：只在"上一版号能推出来"时做
  const parts = version.split(".");
  if (parts.length === 3 && Number(parts[2]) > 0) {
    const prev = `${parts[0]}.${parts[1]}.${Number(parts[2]) - 1}`;
    if (dllHas(buf, `v${prev}`)) bad(`产物里还有上一版字面量 v${prev}（日志指纹会串版本）`);
    else ok(`上一版 v${prev} 不在产物里`);
  }
  // 本版新增的日志字面量（实机验收就靠 grep 这几个串）
  const literals = [
    "Access set audit", "Stripped-lane ledger", "Restore baseline:",
    "self-heal", "real leak", "enough via",
    "Restore targeted: skipped", // P2：陈旧实体过滤（标记后立刻拆路再关模组）
    "the mod log file is not writable right now", // v0.8.11：日志自保闸降级（Player.log 里应看到这句而不是成串 NRE）
  ];
  for (const lit of literals) {
    if (dllHas(buf, lit)) ok(`字面量「${lit}」在位`);
    else bad(`字面量「${lit}」缺失——实机日志里 grep 不到它，验收会误判`);
  }
  // 壳绝不能被编进产物（tests\** 已在 csproj 排除）
  if (dllHas(buf, "AccessAnarchy.T3") || dllHas(buf, "canary")) bad("T3 壳的类型被编进模组产物了：检查 AccessAnarchy.csproj 的 <Compile Remove=\"tests\\**\" />");
  else ok("T3 壳没有被编进模组产物");
} else if (!SKIP_BUILD) {
  section("3. T2 产物");
  bad(`找不到产物：${PRODUCT}`);
}

/* ---------- 4. T3 无游戏回归壳 ---------- */
section("4. T3 纯判据回归壳（零游戏 DLL / 零联网 / 不开游戏）");
const SHELL = path.join(ROOT, "tests", "t3", "T3.csproj");
const SHELL_EXE = path.join(ROOT, "tests", "t3", "bin", "Release", "net48", "AccessAnarchy.T3.exe");
let shellBuilt = false;
try {
  execFileSync("dotnet", ["build", SHELL, "-c", "Release", "--nologo"], { cwd: ROOT, encoding: "utf8", stdio: "pipe" });
  shellBuilt = true;
  ok("壳构建通过（被测的 Systems\\*.cs 确实一个游戏类型都不碰）");
} catch (e) {
  // 直接把 error 行端出来：只打末尾几行时能看到"6 个错误"却看不到是哪 6 条（2026-10-19 踩过）。
  const txt = String(e.stdout || "") + "\n" + String(e.stderr || "") + "\n" + String(e.message || "");
  const errs = txt.split(/\r?\n/).filter(l => /error CS\d+|个错误/.test(l)).slice(0, 8);
  bad("壳构建失败——多半是有人往 Systems/RestoreJudgment.cs 或 Systems/LogFileGuard.cs 里加了游戏类型，或壳自身语法错：" + (errs.join(" | ") || txt.split(/\r?\n/).slice(-4).join(" / ")));
}
if (!shellBuilt) {
  // 假绿比没壳更危险：编译都失败了还去跑上一次留下的 exe，报出来的"全绿"是旧源码的成绩。
  bad("壳构建失败，本轮**不运行**旧产物（上一次的 exe 全绿不代表现在的源码全绿）");
} else if (fs.existsSync(SHELL_EXE)) {
  let out = "";
  let code = 0;
  try {
    out = execFileSync(SHELL_EXE, [], { encoding: "utf8" });
  } catch (e) { code = e.status === undefined ? -1 : e.status; out = String(e.stdout || ""); }
  const summary = (out.match(/TOTAL (\d+) PASS (\d+) FAIL (\d+)/) || []).slice(1).map(Number);
  if (!summary.length) bad("壳没有打出 TOTAL/PASS/FAIL 汇总行，判定不了");
  else {
    console.log(`  断言 ${summary[0]} 条：PASS ${summary[1]}，FAIL ${summary[2]}`);
    if (code === 0 && summary[2] === 0 && summary[0] > 20) ok("T3 全绿");
    else if (summary[0] <= 20) bad(`断言总数只有 ${summary[0]} 条，比上一轮还少——壳被降级了（假绿比没壳更危险）`);
    else bad(`T3 有 ${summary[2]} 条红（退出码 ${code}）：${out.split("\n").filter(l => l.includes("FAIL [")).slice(0, 6).join(" | ")}`);
  }
  // canary：证明断言台**能红**（"永远全绿的壳"和"没有壳"等价）
  let canaryCode = 0;
  let canaryOut = "";
  try {
    canaryOut = execFileSync(SHELL_EXE, ["--canary"], { encoding: "utf8" });
  } catch (e) { canaryCode = e.status === undefined ? -1 : e.status; canaryOut = String(e.stdout || ""); }
  if (canaryCode === 1 && /CANARY OK/.test(canaryOut)) ok("canary：故意失败的那条确实被记录（壳会红）");
  else bad("canary 没红 —— 断言台坏了，现在的全绿毫无意义（退出码 " + canaryCode + "）");
} else {
  bad("找不到壳可执行文件：" + SHELL_EXE);
}

/* ---------- 5. 发布素材（只读检查，不碰线上） ---------- */
section("5. 发布素材与身份（只读；上传/发布只由作者执行）");
const thumb = path.join(ROOT, "Properties", "Thumbnail.jpg");
if (fs.existsSync(thumb)) {
  const sz = fs.statSync(thumb).size;
  if (sz > 2.1 * 1024 * 1024) bad(`Thumbnail.jpg ${(sz / 1048576).toFixed(2)}MB 超过 2.1MB`);
  else ok(`Thumbnail.jpg ${(sz / 1024).toFixed(0)}KB（指的就是作者海报，别改回占位图）`);
} else bad("Properties/Thumbnail.jpg 缺失——发布时 ModPublisher 会**静默**退回官方占位图，把线上封面覆盖掉");
const shots = [...xml.matchAll(/<Screenshot Value="([^"]+)"\s*\/>/g)].map(m => m[1]);
for (const s of shots) {
  const p = path.join(ROOT, s.replace(/\\/g, "/"));
  if (!fs.existsSync(p)) bad(`Screenshot 指向的文件不存在：${s}（同样会被静默丢图）`);
  else if (fs.statSync(p).size > 2.1 * 1024 * 1024) bad(`Screenshot 超过 2.1MB：${s}`);
  else ok(`Screenshot 在位：${s}`);
}
if (fs.existsSync(path.join(ROOT, "bin", "Release", "net48", "0Harmony.dll"))) bad("产物里出现了 0Harmony.dll——本模组是零 Harmony 路线，不该有它");
else ok("零 Harmony：产物里没有 0Harmony.dll");

/* ---------- 6. 托管 PDB 卫生（路径泄露归零，行号定位能力保留） ---------- */
// 玩家报错日志里那串「D:\WorkSpace\CS2MOD\AccessAnarchy\...」不是玩家磁盘上的路径，
// 而是编译期烙进 AccessAnarchy.pdb 的源文件路径——DeployWIP 把 OutDir 全量复制进
// 部署目录、发布原样上传，玩家侧 Mono 按 PDB 文档表打印「路径:行号」（与玩家磁盘无关）。
// csproj 用 PathMap 把工程根重映射成中性前缀 /src：文件名与行号原样保留（崩溃按行号
// 定位的能力不丢），只有绝对路径前缀变成中性。SourceLink（SDK 默认开）会把仓库
// URL+commit 一起烙进 PDB，模组发布用不上，已在 csproj 关掉。
// 三平台原生产物里残留的 C:\Users\<用户名>\... 由 Burst 链接器写入，工具链行为，本项目
// 不改（只进崩溃 dump 的符号化路径，不进 Player.log）。详见 开发笔记 §1。
if (fs.existsSync(path.join(ROOT, "bin", "Release", "net48", "AccessAnarchy.pdb"))) {
  section("6. 托管 PDB 卫生（路径泄露归零，行号定位能力保留）");
  const pb = fs.readFileSync(path.join(ROOT, "bin", "Release", "net48", "AccessAnarchy.pdb"));
  // 对照探针：源文件名必须还在（证明 PDB 仍带源文档、行号表没被一并清掉；扫不到 = 假绿）
  if (!dllHas(pb, "AccessZoneOverlapSystem.cs")) bad("对照探针缺失：源文件名都扫不到 = PDB 被清空/路径找错，本轮泄露判定不可信");
  else ok("对照探针「AccessZoneOverlapSystem.cs」在位（源文档与行号表健在）");
  // 禁止随包上传的机器专属串：编译期绝对路径、用户名、SourceLink 仓库 URL 一律归零
  const forbidden = ["WorkSpace", "CS2MOD", "Users\\", "10975", "raw.github", "githubusercontent"];
  const leaked = forbidden.filter(f => dllHas(pb, f));
  if (leaked.length) bad(`托管 PDB 仍带机器专属串：${leaked.join("、")}（玩家日志会泄露作者磁盘布局——检查 csproj 的 PathMap/SourceLink）`);
  else ok("托管 PDB 已无机器专属串（WorkSpace/用户名/仓库 URL 均归零）");
} else if (!SKIP_BUILD) {
  section("6. 托管 PDB 卫生");
  bad(`找不到托管 PDB：bin\\Release\\net48\\AccessAnarchy.pdb（构建没出调试符号？检查 csproj 的 DebugType）`);
}

console.log("\n----");
if (failures === 0) console.log("门禁全绿 ✓");
else console.log(`门禁有 ${failures} 条红 ✗（退出码 1：修完再谈发布）`);
process.exit(failures === 0 ? 0 : 1);
