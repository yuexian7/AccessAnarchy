// 用法：node tools/crash-triage.mjs "C:/Users/<用户>/AppData/Local/CrashDumps/Cities2.exe.12880.dmp"
// 输出：异常码/读写类型/被访问地址、崩溃指令所在的**完整模块路径**+偏移、崩溃线程寄存器、
//       栈上落在各模块内的返回地址与按模块计数（用来判断本模组的 Burst 模块在不在现场）。
// 配套：node tools/pe-func.mjs <RVA十六进制>  —— 把 lib_burst_generated.dll 的 RVA 映射到函数区间/导出名。
// 背景：2026-10-07 作者两次"鼠标移进地图显示预览模型即闪退"，两次都停在
//       Cities2_Data\Plugins\x86_64\lib_burst_generated.dll + 0x164e3ea 读 0x100000000，
//       栈上 0 次命中 AccessAnarchy_win_x86_64.dll（详见开发笔记 §⑱）。
// 崩溃线程：栈内存 + 模块返回地址扫描（按本转储实测的 MINIDUMP_THREAD 48 字节布局：
//   +0 ThreadId +4 Suspend +8 Priority +12 pad +16 Teb +24 Stack.Start(RSP) +32 Stack.Size(u32)+36 Stack.Rva(u32) +40 CtxDataSize +44 CtxRva）
import fs from 'node:fs';

const file = process.argv[2];
const fd = fs.openSync(file, 'r');
const size = fs.fstatSync(fd).size;
const readAt = (rva, len) => {
	if (rva == null || rva < 0 || len <= 0 || rva + len > size) return null;
	const b = Buffer.alloc(len);
	const got = fs.readSync(fd, b, 0, len, rva);
	return b.slice(0, got);
};
const H = (v) => '0x' + BigInt(v).toString(16);
const head = readAt(0, 32);
const sc = head.readUInt32LE(8);
const dir = readAt(head.readUInt32LE(12), sc * 12);
const st = {};
for (let i = 0; i < sc; i++) { const t = dir.readUInt32LE(i * 12), s = dir.readUInt32LE(i * 12 + 4), r = dir.readUInt32LE(i * 12 + 8); if (t || s) st[t] = { s, r }; }
const e = readAt(st[6].r, 160);
const tid = e.readUInt32LE(0), ip = e.readBigUInt64LE(24), code = e.readUInt32LE(8), op = e.readBigUInt64LE(40), target = e.readBigUInt64LE(48);

const mc = readAt(st[4].r, 4).readUInt32LE(0);
const mraw = readAt(st[4].r + 4, mc * 108);
const mods = [];
for (let i = 0; i < mc; i++) {
	const o = i * 108, base = mraw.readBigUInt64LE(o), sz = mraw.readUInt32LE(o + 8), nr = mraw.readUInt32LE(o + 20);
	const nb = readAt(nr, 4); if (!nb) continue;
	mods.push({ base, sz, nm: readAt(nr + 4, nb.readUInt32LE(0)).toString('utf16le') });
}
const at = (a) => mods.find(m => a >= m.base && a < m.base + BigInt(m.sz));
const sn = (p) => p.split(/[\\\\]/).pop();

const thCount = readAt(st[3].r, 4).readUInt32LE(0);
const thRaw = readAt(st[3].r + 4, Math.min((thCount + 4) * 48, st[3].s - 4));
let rec = null;
for (let i = 0; i < Math.floor(thRaw.length / 48); i++) {
	const o = i * 48;
	if (thRaw.readUInt32LE(o) !== tid) continue;
	rec = { i, start: thRaw.readBigUInt64LE(o + 24), len: thRaw.readUInt32LE(o + 32), rva: thRaw.readUInt32LE(o + 36), ctx: thRaw.readUInt32LE(o + 44), ctxSize: thRaw.readUInt32LE(o + 40) };
	break;
}
console.log(`\n### ${sn(file)}  UTC ${new Date(head.readUInt32LE(20) * 1000).toISOString()}`);
console.log(`异常 code=0x${code.toString(16)} ${op === 0n ? 'READ' : op === 1n ? 'WRITE' : 'EXEC'} ${H(target)}  IP=${H(ip)} @ ${(() => { const m = at(ip); return m ? `${m.nm} +0x${(ip - m.base).toString(16)}` : '不在模块内'; })()}`);
if (!rec) { console.log('线程记录未找到'); process.exit(0); }
console.log(`线程 #${rec.i} tid=${tid} RSP=${H(rec.start)} 栈长=${rec.len} 文件偏移=${H(rec.rva)} CONTEXT ${H(rec.ctx)}(${rec.ctxSize})`);
const ctx = readAt(rec.ctx, rec.ctxSize);
if (ctx) {
	const names = ['Rax', 'Rcx', 'Rdx', 'Rbx', 'Rsp', 'Rbp', 'Rsi', 'Rdi', 'R8', 'R9', 'R10', 'R11', 'R12', 'R13', 'R14', 'R15'];
	console.log('  寄存器：' + names.map((n, k) => `${n}=${H(ctx.readBigUInt64LE(0x78 + k * 8))}`).join(' '));
	console.log(`  Rip=${H(ctx.readBigUInt64LE(0xF8))}（这是 WER 抓异常时的位置：${(() => { const m = at(ctx.readBigUInt64LE(0xF8)); return m ? sn(m.nm) + '+0x' + (ctx.readBigUInt64LE(0xF8) - m.base).toString(16) : '?'; })()}）`);
}
const stack = readAt(rec.rva, rec.len);
if (!stack) { console.log('栈内存不在转储里'); process.exit(0); }
const hits = [];
for (let o = 0; o + 8 <= stack.length; o += 8) {
	const v = stack.readBigUInt64LE(o);
	const m = at(v);
	if (m) hits.push({ o, nm: sn(m.nm), d: v - m.base });
}
const tally = {};
for (const h of hits) tally[`${h.nm}`] = (tally[`${h.nm}`] || 0) + 1;
console.log(`栈内模块内地址 ${hits.length} 个（深→浅前 40 条）：`);
console.log('  ' + hits.slice(0, 40).map(h => `${h.nm}+0x${h.d.toString(16)}`).join('\n  '));
console.log('\n按模块计数：');
Object.entries(tally).sort((a, b) => b[1] - a[1]).slice(0, 14).forEach(([k, v]) => console.log(`   ${String(v).padStart(3)}  ${k}`));
console.log(`\n本模组 Burst 模块命中：${(tally['AccessAnarchy_win_x86_64.dll'] || 0)}`);
