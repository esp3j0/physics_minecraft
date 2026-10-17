//! app::net::chunk_stream —— 每玩家 chunk 流(REQ-003 §4.3;任务书 req003-net/02)。
//!
//! 职责:连接层持 chunk 流状态(center 合并 pending 单槽、视距、loaded 集合),
//! pump cadence(独立 interval task,默认 10 Hz)做视距差分:
//! `desired − loaded → to_load`(切比雪夫 center-out 序,截 16)、
//! `loaded − desired → to_unload`(同序,截 16),经 [`DriverCommand::StreamChunks`]
//! 往驱动取数(驱动经 ChunkProvider:持久层优先、生成器兜底),回执 chunk 逐个
//! 入出站 World 队列(ChunkData);unload 侧先清 loaded 再发 ChunkUnload。
//!
//! v2 语义(调研 §6,照搬):**socket 读路径不触发 chunk 工作**——只更新合并的
//! pending 中心([`ChunkStreamState::set_center`],零 IO 零分配);pump 每轮从
//! 最新 center 重算、不保留工作队列(latest-wins);回执失败(驱动关停)=
//! 连接终止。chunk 的生成/编码在模拟侧,连接层永不自己取数。
//!
//! 断连即停:泵随连接任务而生(`run_pump` 在连接 task 内驱动),连接终止时
//! 泵的循环自然结束——无孤儿任务。

use std::collections::BTreeSet;

use types::ChunkPos;

// ---- 常量(v2 数值,调研 §6) ----

/// 每泵装载数上限(v2 `MAX_CHUNK_UPDATE_LOADS_PER_UPDATE` = 16)。
pub const MAX_LOADS_PER_PUMP: usize = 16;
/// 每泵卸载数上限(v2 `MAX_CHUNK_UNLOADS_PER_UPDATE` = 16;单泵 ≤32 次 chunk 变更)。
pub const MAX_UNLOADS_PER_PUMP: usize = 16;

/// 两坐标的切比雪夫距离(`max(|dx|, |dz|)`;棋盘距离)。
pub fn chebyshev(a: ChunkPos, b: ChunkPos) -> i32 {
    (a.x - b.x).abs().max((a.z - b.z).abs())
}

/// 视距方形(切比雪夫距离 ≤ `view_distance` 的全部 chunk;
/// `(2r+1)²` 个,含 center 本身)。升序(与差分排序同键,断言友好)。
pub fn view_square(center: ChunkPos, view_distance: u8) -> BTreeSet<ChunkPos> {
    let r = view_distance as i32;
    (center.x - r..=center.x + r)
        .flat_map(|x| (center.z - r..=center.z + r).map(move |z| ChunkPos::new(x, z)))
        .collect()
}

/// 切比雪夫 center-out 升序键(v2 `to_load.sort_by_key` 同式):
/// `(chebyshev(center, pos), pos.x, pos.z)`。
fn center_out_key(center: ChunkPos, pos: ChunkPos) -> (i32, i32, i32) {
    (chebyshev(center, pos), pos.x, pos.z)
}

// ---- 连接态(照 v2 chunk_stream) ----

/// 每玩家 chunk 流状态(连接层持有):合并 pending 中心 + 订阅视距 + 已装载集。
#[derive(Debug, Clone)]
pub struct ChunkStreamState {
    /// 玩家所在 chunk(读路径 [`Self::set_center`] 只写本槽,合并中间态——
    /// socket 读路径不触发 chunk 工作,v2 单槽合并语义)。
    center: ChunkPos,
    /// 视距(chunk;Subscribe 时钳 ≤ [`VIEW_DISTANCE_MAX`] 后定格)。
    view_distance: u8,
    /// 已对客户端发出且未卸载的 chunk 集合(差分基准)。
    loaded: BTreeSet<ChunkPos>,
}

impl ChunkStreamState {
    /// 订阅时建态:center 取玩家起始位置所在 chunk,视距为已钳制值。
    pub fn new(center: ChunkPos, view_distance: u8) -> Self {
        Self {
            center,
            view_distance,
            loaded: BTreeSet::new(),
        }
    }

    /// 当前中心(测试与 pump 排序用)。
    pub fn center(&self) -> ChunkPos {
        self.center
    }

    /// 视距(测试断言钳制生效用)。
    pub fn view_distance(&self) -> u8 {
        self.view_distance
    }

    /// 已装载集只读视图(测试与断连清理用)。
    pub fn loaded(&self) -> &BTreeSet<ChunkPos> {
        &self.loaded
    }

    /// 读路径:center 合并 pending 单槽——只写字段,不触发任何 chunk 工作
    /// (v2 pending_position 语义;工作由 pump cadence 统一消化)。
    pub fn set_center(&mut self, center: ChunkPos) {
        self.center = center;
    }

    /// 一轮泵(纯函数,无 IO):视距差分。
    ///
    /// - `to_load = desired − loaded`,按 `(chebyshev(center), x, z)` 升序,截
    ///   [`MAX_LOADS_PER_PUMP`];
    /// - `to_unload = loaded − desired`,同序,截 [`MAX_UNLOADS_PER_PUMP`]。
    ///
    /// v2 latest-wins:**每轮从最新 center 全量重算、不保留跨轮队列**——中间态
    /// 被新中心合并,过时的装卸意图自然消散。
    pub fn diff(&mut self) -> StreamDiff {
        let desired = view_square(self.center, self.view_distance);
        let to_load: Vec<ChunkPos> = desired
            .difference(&self.loaded)
            .copied()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        let mut to_load = to_load;
        to_load.sort_by_key(|pos| center_out_key(self.center, *pos));
        to_load.truncate(MAX_LOADS_PER_PUMP);

        let mut to_unload: Vec<ChunkPos> = self
            .loaded
            .difference(&desired)
            .copied()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        to_unload.sort_by_key(|pos| center_out_key(self.center, *pos));
        to_unload.truncate(MAX_UNLOADS_PER_PUMP);

        self.loaded.extend(to_load.iter().copied());
        for pos in &to_unload {
            self.loaded.remove(pos);
        }
        StreamDiff { to_load, to_unload }
    }

    /// 视距换挡(重复 Subscribe / 显式换挡;新视距须 ≤ [`super::VIEW_DISTANCE_MAX`]
    /// ——由调用方钳制后传入):**立刻替换**生效值,外环由泵差分自然卸出
    /// (latest-wins,不保留跨轮队列)。
    pub fn resubscribe(&mut self, view_distance: u8) {
        self.view_distance = view_distance;
    }

    /// 装载收账回滚(评审修复③):泵 `try_send` 失败(上行通道满/关)时,
    /// 撤销 [`Self::diff`] 对本轮 `to_load` 的收账——这些 chunk 从未发出,
    /// 不回滚会被差分永久排除(账面已装、实际未发)。只撤本轮回滚清单中
    /// 的键(不整集重置:并发窗口内的其它状态不受影响)。
    pub fn rollback_load(&mut self, to_load: &[ChunkPos]) {
        for pos in to_load {
            self.loaded.remove(pos);
        }
    }

    /// 连接终止后的状态丢弃(当前 loaded 集合;断连清理语义——非模拟侧
    /// 卸载,观察者的模拟激活/票据由 REQ-002 观察者生命周期管理)。
    pub fn take_loaded(&mut self) -> BTreeSet<ChunkPos> {
        std::mem::take(&mut self.loaded)
    }
}

/// 一轮泵的差分结果(纯数据;调用方发 `StreamChunks` 用)。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct StreamDiff {
    /// 本泵要装载的 chunk(已截 16,center-out 序)。
    pub to_load: Vec<ChunkPos>,
    /// 本泵要卸载的 chunk(已截 16,center-out 序)。
    pub to_unload: Vec<ChunkPos>,
}

// ---- 泵任务 ----

/// 泵配置(`NetConfig` 增量):泵周期(默认 10 Hz = 100ms)、单 pump 间隔最大
/// 快进轮数(暂停时钟测试用,语义 = 最多消化这么多个 cadence 间隔)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PumpConfig {
    /// 泵周期毫秒数(REQ-003 §4.3:默认 10 Hz)。
    pub interval_ms: u64,
    /// 单个 interval 到期最多快进的轮数(防暂停时钟下活锁;语义为消化
    /// 积压的 cadence 间隔,上限内逐轮 `diff`)。
    pub max_catchup_rounds: u32,
}

impl Default for PumpConfig {
    fn default() -> Self {
        Self {
            interval_ms: 100,
            max_catchup_rounds: 4,
        }
    }
}

/// 泵任务(连接 task 的伴生 future;连接终止即停,无孤儿任务)。
///
/// 周期性对 `state` 做差分,把装卸清单连同 oneshot 回执发给驱动;回执到达后:
/// load 的 chunk 逐个入出站 World 队列(`ChunkData`,经 [`WorldWriter`],单
/// socket 写者 = 连接任务),unload 侧发 `ChunkUnload`。泵循环 `select!` 监听
/// `shutdown`(连接关机广播)——断连即停。
///
/// 驱动关停 = 回执 `Err`(oneshot 发送端弃发)→ **连接终止**:泵停止(任务书:
/// 回执失败 = 连接终止;连接侧的 Left 上报由其读循环对端关断路径收口)。
pub(crate) async fn run_pump(
    state: std::sync::Arc<tokio::sync::Mutex<ChunkStreamState>>,
    cfg: PumpConfig,
    mut shutdown: tokio::sync::broadcast::Receiver<()>,
    up_tx: tokio::sync::mpsc::Sender<super::NetUp>,
    mut outbound: super::WorldWriter,
    player: messaging::InputSource,
) {
    // StreamChunks 请求通道(泵 → 驱动上行):= 连接的 NetUp 通道(有界),
    // 泵永不无界积压(差分每轮重算,拒绝 = 本轮丢给下一轮 latest-wins,无害)。
    // 请求以 `NetUp::DriverCommand` 上行——驱动侧经 NetUp 路由后持有 oneshot
    // 回执(REQ-003 §4.4)。
    let interval = std::time::Duration::from_millis(cfg.interval_ms.max(1));
    let mut ticker = tokio::time::interval(interval);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    // 消化 tokio interval 的立即首枚(与 driver.run 同因,README 决策 2)。
    ticker.tick().await;

    loop {
        tokio::select! {
            biased;

            _ = shutdown.recv() => return,

            _ = ticker.tick() => {
                // 单个 interval 至多快进 max_catchup_rounds 轮(暂停时钟下
                // interval 恒到期;有界快进防活锁,语义 = 消化积压 cadence)。
                for _ in 0..cfg.max_catchup_rounds {
                    // 差分锁窗口:只做纯计算+集合维护,零 await(短临界区)。
                    let diff = {
                        let mut guard = state.lock().await;
                        guard.diff()
                    };
                    // 卸载侧(零等待):先清账再发 ChunkUnload 给客户端。
                    for pos in &diff.to_unload {
                        outbound.push_world(super::S2C::ChunkUnload { pos: *pos });
                    }
                    if diff.to_load.is_empty() {
                        break; // 稳态:无装载差分,等下一 cadence
                    }
                    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
                    let request = super::DriverCommand::StreamChunks {
                        load: diff.to_load.clone(),
                        unload: diff.to_unload.clone(),
                        reply: reply_tx,
                    };
                    if up_tx
                        .try_send(super::NetUp::DriverCommand {
                            player,
                            cmd: request,
                        })
                        .is_err()
                    {
                        // 通道满/关:本轮放弃(latest-wins)。**回滚 loaded**
                        // (评审修复③):diff() 已对本轮 to_load 收账,不撤销
                        // 的话下轮差分不再含这些 chunk——被记为已装载却从未
                        // 发出,永久丢失。回滚后下轮 diff 重算重发(卸载侧
                        // 已发 ChunkUnload 无需回滚:账已清 = 语义成立)。
                        let mut guard = state.lock().await;
                        guard.rollback_load(&diff.to_load);
                        break;
                    }
                    // 回执:Ok = 驱动已取数(逐个入 World 队列,ChunkData);
                    // Err = 驱动关停 → 连接终止(泵收口)。
                    match reply_rx.await {
                        Ok(chunks) => {
                            for (pos, chunk) in chunks {
                                outbound
                                    .push_world(super::S2C::ChunkData { pos, chunk });
                            }
                        }
                        Err(_dropped) => return, // 驱动关停 → 连接终止
                    }
                }
            }
        }
    }
}

// ---- 测试清单(任务书 02;纯函数层在此,集成层在 net::tests) ----

#[cfg(test)]
mod tests {
    use super::*;

    /// 差分序键纯函数:同泵内首装必为 center(0 距),次环按 (距离, x, z) 升序。
    #[test]
    fn chebyshev_orders_center_out() {
        let center = ChunkPos::new(10, -4);
        // 单点距离。
        assert_eq!(chebyshev(center, center), 0);
        assert_eq!(chebyshev(center, ChunkPos::new(12, -6)), 2);
        assert_eq!(chebyshev(center, ChunkPos::new(7, -4)), 3);
        // view_square 大小 = (2r+1)²。
        assert_eq!(view_square(center, 2).len(), 25);
        assert_eq!(view_square(center, 8).len(), 289);
        // 升序键:同距按 x 再 z。
        let mut positions: Vec<ChunkPos> = view_square(center, 2).into_iter().collect();
        positions.sort_by_key(|pos| center_out_key(center, *pos));
        assert_eq!(positions[0], center, "首 = center");
        assert_eq!(chebyshev(center, positions[1]), 1, "次环 = 距 1");
        let keys: Vec<(i32, i32, i32)> = positions
            .iter()
            .map(|pos| center_out_key(center, *pos))
            .collect();
        let mut sorted = keys.clone();
        sorted.sort();
        assert_eq!(keys, sorted, "全表按 (距离, x, z) 升序");
    }

    /// 首轮差分:整片视距按 center-out 升序,恰截 16;loaded 即刻收账;
    /// 多泵补齐(49 = 16×3+1)。
    #[test]
    fn first_diff_truncates_sixteen_center_out() {
        let center = ChunkPos::new(0, 0);
        let mut state = ChunkStreamState::new(center, 3);
        let diff = state.diff();
        assert_eq!(diff.to_load.len(), MAX_LOADS_PER_PUMP, "首泵恰 16");
        assert!(diff.to_unload.is_empty(), "无卸载");
        // 序:0 距 1 个 + 1 距 8 个 + 2 距环前 7 = 16。
        assert_eq!(diff.to_load[0], center);
        assert_eq!(
            chebyshev(center, diff.to_load[8]),
            1,
            "前 9 个含全部 1 距环"
        );
        assert_eq!(chebyshev(center, diff.to_load[9]), 2, "第 10 个起 = 2 距环");
        let keys: Vec<(i32, i32, i32)> = diff
            .to_load
            .iter()
            .map(|pos| center_out_key(center, *pos))
            .collect();
        let mut sorted = keys.clone();
        sorted.sort();
        assert_eq!(keys, sorted, "装载清单升序");
        // loaded 收账(截多少收多少);余量由后续泵补齐(不保留跨轮队列,但
        // 集合差分每轮重算——下一轮继续装剩余的)。
        assert_eq!(state.loaded().len(), 16);
        let second = state.diff();
        assert_eq!(second.to_load.len(), 16, "次泵续装 16(多泵补齐)");
        assert_eq!(
            second.to_load[0],
            ChunkPos::new(0, -2),
            "续泵仍 center-out 续序(2 距环剩余首个)"
        );
        let third = state.diff();
        assert_eq!(third.to_load.len(), 16);
        let fourth = state.diff();
        assert_eq!(fourth.to_load.len(), 1, "49 = 16×3 + 1");
        let settle = state.diff();
        assert!(
            settle.to_load.is_empty() && settle.to_unload.is_empty(),
            "补齐后稳态"
        );
        assert_eq!(state.loaded().len(), 49, "视距 3 全量");
    }

    /// 中心平移 1:只装新列、只卸旧列;待卸不在本轮装载清单里。
    #[test]
    fn center_move_yields_minimal_diff() {
        let mut state = ChunkStreamState::new(ChunkPos::new(0, 0), 1);
        let first = state.diff();
        assert_eq!(first.to_load.len(), 9, "视距 1 = 3×3 方形");
        assert_eq!(state.loaded().len(), 9);

        state.set_center(ChunkPos::new(1, 0));
        let diff = state.diff();
        // 新列:x=2 的 z ∈ [-1,1];旧列:x=-1 同列。
        assert_eq!(
            diff.to_load,
            vec![
                ChunkPos::new(2, -1),
                ChunkPos::new(2, 0),
                ChunkPos::new(2, 1)
            ],
            "只装新进入的列"
        );
        assert_eq!(
            diff.to_unload,
            vec![
                ChunkPos::new(-1, -1),
                ChunkPos::new(-1, 0),
                ChunkPos::new(-1, 1)
            ],
            "只卸离开的列"
        );
        assert_eq!(state.loaded().len(), 9, "总持有不变(9 换 9)");
    }

    /// 卸载也截 16(远遁:49 全在 desired 外,首泵卸 16;装卸并行——同泵
    /// 新位视距也在装入;余下逐泵清)。
    #[test]
    fn unloads_truncate_sixteen_per_pump() {
        let old_center = ChunkPos::new(0, 0);
        let new_center = ChunkPos::new(100, 100);
        let mut state = ChunkStreamState::new(old_center, 3);
        for _ in 0..4 {
            state.diff(); // 装满 49(16×3+1)
        }
        assert_eq!(state.loaded().len(), 49);
        state.set_center(new_center);
        let diff = state.diff();
        assert_eq!(diff.to_unload.len(), MAX_UNLOADS_PER_PUMP, "卸载截 16");
        assert_eq!(
            diff.to_load.len(),
            MAX_LOADS_PER_PUMP,
            "新位置视距 49 个全部待装,本泵亦截 16(装卸并行)"
        );
        // 收账:49 − 16(卸)+ 16(装)= 49;旧集开始退场。
        assert_eq!(state.loaded().len(), 49);
        let old_left = state
            .loaded()
            .iter()
            .filter(|p| chebyshev(old_center, **p) <= 3)
            .count();
        assert_eq!(old_left, 33, "旧集余 33(49 − 16)");
        // 逐泵:旧集 33 → 16 → 0;新集补齐。终态 = 恰新位置视距方形。
        let mut rounds = 0;
        loop {
            let next = state.diff();
            if next.to_unload.is_empty() && next.to_load.is_empty() {
                break;
            }
            rounds += 1;
            assert!(rounds < 20, "有界收敛(防死循环)");
            assert!(next.to_unload.len() <= MAX_UNLOADS_PER_PUMP, "每泵卸 ≤16");
            assert!(next.to_load.len() <= MAX_LOADS_PER_PUMP, "每泵装 ≤16");
        }
        assert_eq!(
            state.loaded(),
            &view_square(new_center, 3),
            "终态:loaded 恰为新位置视距方形"
        );
        assert!(
            !state
                .loaded()
                .iter()
                .any(|p| chebyshev(old_center, *p) <= 3 && chebyshev(new_center, *p) > 3),
            "旧集(与新位不相交部分)已全部卸账"
        );
    }

    /// 视距换挡:新视距即刻生效(差分按新方形算;收窄 = 逐泵卸除外环 16)。
    #[test]
    fn resubscribe_replaces_view_distance() {
        let mut state = ChunkStreamState::new(ChunkPos::new(0, 0), 2);
        for _ in 0..2 {
            state.diff(); // 装满 25
        }
        assert_eq!(state.loaded().len(), 25);
        state.resubscribe(1);
        assert_eq!(state.view_distance(), 1);
        let diff = state.diff();
        assert!(diff.to_load.is_empty(), "收窄无装载");
        assert_eq!(
            diff.to_unload.len(),
            16,
            "外环卸载截 16(待卸恰 16 = 25 − 9)"
        );
        let settle = state.diff();
        assert!(settle.to_unload.is_empty(), "收口:全卸完(loaded = 3×3)");
        assert_eq!(state.loaded().len(), 9);
    }

    /// 视距 8 泵尽总轮数:289 = 16×18 + 1 → 19 轮(≥ ⌈289/16⌉,测试清单
    /// per_pump_limits_enforced 的纯函数口径)。
    #[test]
    fn view_distance_eight_pumps_to_completion() {
        let mut state = ChunkStreamState::new(ChunkPos::new(5, -7), 8);
        assert_eq!(view_square(ChunkPos::new(5, -7), 8).len(), 289);
        let mut first_pump = None;
        let mut pumps = 0;
        loop {
            let diff = state.diff();
            if diff.to_load.is_empty() {
                break;
            }
            if first_pump.is_none() {
                first_pump = Some(diff.to_load.len());
            }
            assert!(diff.to_load.len() <= MAX_LOADS_PER_PUMP, "每泵 ≤ 16");
            pumps += 1;
            assert!(pumps < 100, "泵数有界(防死循环)");
        }
        assert_eq!(first_pump, Some(16), "首泵恰 16");
        assert_eq!(state.loaded().len(), 289, "全部装完");
        assert_eq!(pumps, 19, "总泵数 19 = ⌈289/16⌉(≥ 19 ✓)");
    }

    /// 评审修复③(REQ-003):泵 `try_send` 失败 → `rollback_load` 撤销本轮
    /// 收账 → 下轮差分重发同一批(不丢 chunk)。模拟泵循环:diff → 发送
    /// 失败 → 回滚 → 下轮 diff。
    #[test]
    fn rollback_load_restores_diff_after_failed_send() {
        let mut state = ChunkStreamState::new(ChunkPos::new(0, 0), 2); // 25 chunk
        let first = state.diff();
        assert_eq!(first.to_load.len(), MAX_LOADS_PER_PUMP);
        assert_eq!(state.loaded().len(), 16, "diff 即收账(发送前)");

        // 通道满/关 = 发送失败:泵回滚本轮 to_load。
        state.rollback_load(&first.to_load);
        assert!(
            state
                .loaded()
                .is_disjoint(&first.to_load.iter().copied().collect()),
            "回滚后:本轮清单不再在账(可重发)"
        );

        // 下轮差分:重算出同一批(顺序一致 = 请求不丢不重)。
        let retry = state.diff();
        assert_eq!(retry.to_load, first.to_load, "回滚后下轮重发同一批");
        assert_eq!(state.loaded().len(), 16, "重发轮重新收账");
    }

    /// 回滚只撤指定键:回滚 A 批不影响已在途的 B 批收账(泵在途 ≤1,理论
    /// 单批;防御口径——部分回滚/并发窗口不误伤)。
    #[test]
    fn rollback_load_only_removes_given_keys() {
        let mut state = ChunkStreamState::new(ChunkPos::new(0, 0), 1); // 9 chunk
        let all = state.diff();
        assert_eq!(state.loaded().len(), 9);
        // 只回滚其中 3 个:其余 6 个保持在账。
        let subset: Vec<ChunkPos> = all.to_load[..3].to_vec();
        state.rollback_load(&subset);
        assert_eq!(state.loaded().len(), 6, "只撤给定键");
        assert!(
            subset.iter().all(|p| !state.loaded().contains(p)),
            "给定键全部撤账"
        );
        // 下轮差分恰好补回这 3 个。
        let retry = state.diff();
        assert_eq!(retry.to_load.len(), 3, "差分只补被撤的 3 个");
        assert_eq!(state.loaded().len(), 9, "账面复原");
    }
}
