# Demo B:体素世界 → 暴露面 cuboid collider 的组织策略对比

对应架构问题(REQ-001 激活层):体素数组是唯一真相,物理 collider 是派生缓存;
暴露面(6 邻居判据)→ 静态 cuboid collider(禁用 `Voxels` 形状,有 panic bug)。
本实验对比三种 collider 组织策略,回答"collider 怎么组织、chunk 换装多贵"。

## 怎么跑

```bash
.venv/Scripts/python.exe demos/voxel-collider/main.py
```

场景:48×32×48 体素场(30.3% solid),sin/cos 起伏地形 + 一栋带门洞的空心房;
chunk = x/z 16×16 列全高(9 个 chunk);100 个球从空中落下,稳定后测 300 tick 平均 step 耗时。

## 结果(rapier3d 0.36.0,Windows)

| 策略 | collider/形状数 | 面提取 | collider 构建 | 插入世界 | step 均耗时 | 抽样穿深 |
| --- | --- | --- | --- | --- | --- | --- |
| 逐面薄片(挂 chunk 静态体) | 9488 | 3ms | 13ms | 4ms | 0.201 ms | 0 |
| 贪心合并(chunk 内) | 2085(压缩 4.8x) | 3ms | 65ms | 1ms | 0.187 ms | 0 |
| compound(每 chunk 1 个) | 9 | 3ms | 20ms | 0ms | 0.191 ms | 0 |

换装测试(逐面策略,卸 5 个 chunk 再装回,含 Python 侧开销):**卸载 0.3ms / 装载 1.4ms**。

## 结论

1. **静态 collider 慷慨原则再次证实**:9488 个静态 collider + 100 动态球,step 仅 ~0.2ms,
   三种策略几乎无差(静态体走 BVH broad-phase,数量在万级不敏感)。collider 数量不是优化目标。
2. **结构上推荐:每 chunk 一个静态刚体 + `Collider.compound`(每 chunk 一个 collider 句柄)**:
   chunk 换装 = 1 个 body 的 remove/add(实测亚毫秒到低毫秒级,与任务书"换装下一帧 ~2ms BVH
   增量更新"一致);插入耗时随句柄数下降;激活/休眠管理粒度天然对齐。
3. **贪心合并(压缩 4.8x)是可选优化而非必需**:step 收益 ~7%,Python 实现反而更贵(65ms);
   若做,放 Rust 侧。注意 naive 合并必须**按 chunk 界内进行**(本 demo 已按 chunk 分组合并),
   否则一个 cuboid 跨界,chunk 卸载会带走邻居的碰撞几何。
4. 体素→暴露面提取本身极便宜(numpy 全场 3ms;Rust 侧增量提取更便宜),支持"collider 是
   派生缓存、方块变更时只重建受影响 chunk"的设计。

## 对 REQ-001 的具体建议

- 激活档 ②(collider 插入)定为:chunk 静态体 + compound,collider 内容 = `f(体素数组)`
  纯函数输出(呼应 demo A 的接缝结论:同一体素数据在任何分片必须生成相同 collider)。
- 方块增删 → 只重建该 chunk 的 compound(毫秒级),不需要细粒度 collider 增删。
- chunk 换装预算可以放宽:真正要吝啬的是动态体(任务书:1 万动态体同醒 = 44 步/秒)。
