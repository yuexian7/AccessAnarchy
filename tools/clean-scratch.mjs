#!/usr/bin/env node
// Access Anarchy 构建/草稿自动清理（构建末尾由 csproj 的 CleanAgentScratch 目标调用，也可手动跑）。
//
// 为什么需要：本仓库的调研与草稿目录（.local/、Burst 的 Temp）会在多轮迭代里积累**可再生成**的大文件，
// 而正式产物目录（bin/Release/net48、部署到游戏 Mods 目录的 dll/so/bundle）一个都不能碰。
// 所以这里只用**白名单**：命中白名单根目录 + 满足保留规则才删，其它一律跳过并打印原因。
//
// 明确不在白名单里（永远不删）：research\（反编译件，重跑 ilspycmd 要几小时）、template\、Properties\、
// 任何 Mods\AccessAnarchy 部署目录、任何 *.coc（玩家设置档）、游戏 Logs\（作者要靠它定位 bug）。
//
// 用法：node tools/clean-scratch.mjs [--dry-run] [--quiet]
import fs from 'node:fs';
import path from 'node:path';

const ROOT = path.resolve(import.meta.dirname, '..');
const args = new Set(process.argv.slice(2));
const DRY = args.has('--dry-run');
const QUIET = args.has('--quiet');

// 每条规则：dir = 扫描目录，test = 命中可删文件，keep = 同组保留个数（0 = 全删），why = 说明。
const RULES = [
    {
        why: 'Burst 编译中间目录（每次构建自己生成）',
        paths: ['bin/Release/net48/Temp', 'obj/Release/net48/Temp'].map((p) => path.join(ROOT, p)),
        dir: true,
    },
    {
        why: 'Debug 构建树（本项目只用 `-c Release`，Debug 产物是纯再生成物；等价于 `dotnet clean -c Debug`）',
        paths: ['bin/Debug', 'obj/Debug'].map((p) => path.join(ROOT, p)),
        dir: true,
        // 只在真正的交付物 bin/Release 还在的时候才清 Debug，避免哪天构建配置改了把唯一产物删掉。
        requires: ['bin/Release'],
    },
    {
        why: '官方 Locale.cok 的解压副本，随时可从游戏目录重解（见 memory: cs2-official-locale-data）',
        paths: [path.join(ROOT, '.local/locale/Locale.zip')],
    },
    {
        why: '补丁脚本自留的 .before-* 备份：每个源文件只留最近 2 份',
        dir: false,
        scan: path.join(ROOT, '.local/agent-work'),
        test: (name) => name.includes('.before-'),
        group: (name) => name.split('.before-')[0],
        keep: 2,
    },
];

let freed = 0;
const lines = [];

function sizeOf(p) {
    const st = fs.statSync(p);
    if (st.isFile()) return st.size;
    let total = 0;
    for (const e of fs.readdirSync(p, { withFileTypes: true })) {
        total += sizeOf(path.join(p, e.name));
    }
    return total;
}

function remove(p, isDir, why) {
    const bytes = sizeOf(p);
    freed += bytes;
    lines.push(`${DRY ? 'would free' : 'freed'} ${(bytes / 1048576).toFixed(2)} MB  ${path.relative(ROOT, p)}  (${why})`);
    if (!DRY) {
        if (isDir) fs.rmSync(p, { recursive: true, force: true });
        else fs.rmSync(p, { force: true });
    }
}

for (const rule of RULES) {
    if (rule.paths) {
        if (rule.requires && rule.requires.some((p) => !fs.existsSync(path.join(ROOT, p)))) {
            lines.push(`skipped: ${rule.why}（前置条件 ${rule.requires.join(', ')} 不成立）`);
            continue;
        }
        for (const p of rule.paths) {
            if (!fs.existsSync(p)) continue;
            const isDir = rule.dir && fs.statSync(p).isDirectory();
            remove(p, isDir, rule.why);
        }
        continue;
    }
    if (!fs.existsSync(rule.scan)) continue;
    const hits = fs.readdirSync(rule.scan, { withFileTypes: true })
        .filter((e) => e.isFile() && rule.test(e.name))
        .map((e) => ({ name: e.name, mtime: fs.statSync(path.join(rule.scan, e.name)).mtimeMs }))
        .sort((a, b) => b.mtime - a.mtime);
    const byGroup = new Map();
    for (const h of hits) {
        const g = rule.group(h.name);
        if (!byGroup.has(g)) byGroup.set(g, []);
        byGroup.get(g).push(h);
    }
    for (const [, list] of byGroup) {
        for (const h of list.slice(rule.keep)) {
            remove(path.join(rule.scan, h.name), false, rule.why);
        }
    }
}

if (!QUIET) {
    console.log(lines.length ? lines.join('\n') : 'nothing to clean');
    console.log(`${DRY ? 'DRY RUN' : 'cleaned'}: ${(freed / 1048576).toFixed(2)} MB`);
}
