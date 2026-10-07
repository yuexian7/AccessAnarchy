// 用 .pdata（RUNTIME_FUNCTION 表）定位崩溃 RVA 所属函数起点，再用导出表拿 Burst 函数名。
import fs from 'node:fs';

const dll = 'F:/SteamLibrary/steamapps/common/Cities Skylines II/Cities2_Data/Plugins/x86_64/lib_burst_generated.dll';
const target = BigInt('0x' + (process.argv[2] || '164e3ea'));
const fd = fs.openSync(dll, 'r');
const size = fs.fstatSync(fd).size;
const rd = (p, n) => { if (p == null || p < 0 || n <= 0 || p + n > size) return null; const b = Buffer.alloc(n); const g = fs.readSync(fd, b, 0, n, p); return b.slice(0, g); };

const peOff = rd(0, 64).readUInt32LE(0x3C);
const head2 = rd(peOff, 24);
const nt = rd(peOff + 4, 24 + 240);
const nSec = head2.readUInt16LE(6), optSize = head2.readUInt16LE(20);
const secRaw = rd(peOff + 24 + optSize, nSec * 40);
const secs = [];
for (let i = 0; i < nSec; i++) {
	const o = i * 40;
	secs.push({ nm: secRaw.toString('ascii', o, o + 8).replace(/\0/g, ''), vaddr: secRaw.readUInt32LE(o + 12), vsize: secRaw.readUInt32LE(o + 8), rawSize: secRaw.readUInt32LE(o + 16), raw: secRaw.readUInt32LE(o + 20) });
}
const toFile = (rva) => { const s = secs.find(x => rva >= x.vaddr && rva < x.vaddr + Math.max(x.vsize, x.rawSize)); return s ? rva - s.vaddr + s.raw : null; };

// 导出表：rva -> name
const dirBuf = rd(toFile(nt.readUInt32LE(20 + 112)), 40);
const nFunc = dirBuf.readUInt32LE(20), nNames = dirBuf.readUInt32LE(24);
const addrs = rd(toFile(dirBuf.readUInt32LE(28)), nFunc * 4);
const nameRefs = rd(toFile(dirBuf.readUInt32LE(32)), nNames * 4);
const ords = rd(toFile(dirBuf.readUInt32LE(36)), nNames * 2);
const nameByIdx = new Map();
for (let k = 0; k < nNames; k++) {
	const idx = ords.readUInt16LE(k * 2);
	const nb = rd(toFile(nameRefs.readUInt32LE(k * 4)), 300);
	if (nb) nameByIdx.set(idx, nb.toString('ascii').split('\0')[0]);
}
const byRva = new Map();
for (let i = 0; i < nFunc; i++) { const r = addrs.readUInt32LE(i * 4); if (!byRva.has(r)) byRva.set(r, nameByIdx.get(i) ?? `(ordinal ${i})`); }
console.log(`导出：函数 ${nFunc} 名称 ${nNames} 去重后 rva ${byRva.size}`);

// .pdata：RUNTIME_FUNCTION{Begin(4) End(4) UnwindInfo(4)}
const pdata = secs.find(s => s.nm === '.pdata');
const pr = rd(pdata.raw, pdata.rawSize);
const rfCount = Math.floor(pr.length / 12);
let hit = null;
for (let i = 0; i < rfCount; i++) {
	const b = pr.readUInt32LE(i * 12), e = pr.readUInt32LE(i * 12 + 4);
	if (Number(target) >= b && Number(target) < e) { hit = { b, e, i }; break; }
}
if (!hit) { console.log('RUNTIME_FUNCTION 未覆盖该 RVA'); process.exit(0); }
console.log(`\n崩溃 RVA 0x${target.toString(16)} 属于函数 [0x${hit.b.toString(16)} .. 0x${hit.e.toString(16)})  长度 ${hit.e - hit.b} 字节  函数内偏移 +0x${(Number(target) - hit.b).toString(16)}`);
console.log(`函数名：${byRva.get(hit.b) ?? '(该函数无导出名)'}`);

// 附近函数，便于确认命名风格
const near = [...byRva.entries()].filter(([r]) => Math.abs(r - hit.b) < 0x20000).sort((a, b) => a[0] - b[0]);
console.log(`\n±0x20000 内的导出函数（${near.length} 个）：`);
for (const [r, n] of near.slice(0, 24)) console.log(`  0x${r.toString(16)}  ${n}`);

// 函数头 96 字节 + 崩溃点前后 48 字节
const fh = rd(toFile(hit.b), 64);
console.log(`\n函数头指令字节：${fh ? fh.toString('hex').replace(/(..)/g, '$1 ').trim() : '?'}`);
const around = rd(toFile(Number(target)) - 32, 64);
console.log(`崩溃点 -32..+32：${around ? around.toString('hex').replace(/(..)/g, '$1 ').trim() : '?'}`);
