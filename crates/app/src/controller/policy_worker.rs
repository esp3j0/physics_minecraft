//! 策略推理工作线程(REQ-012 D01b-2 §1.1,AC-D3):推理不阻塞 tick。
//!
//! 只有一个工作线程,请求先进先出(`std::sync::mpsc`)。驱动侧每拍把到控制拍
//! 的策略主体的观测快照 `submit` 进来,在 `wait_budget` 内 `recv_until` 收回复;
//! 超时未回沿用上次目标(内核保持电机目标),由 `ControllerHost` 记遥测。
//!
//! 整张 `PolicyTable` 的所有权在线程里(`act` 需要 `&mut Policy` 复用观测缓冲);
//! startup 挂载用的元数据在交出所有权前经 `PolicyTable::meta()` 抽出(D01b-2
//! 裁定方案 2)。`debug_delay` 只给测试用:每次推理前人为睡这么久,模拟慢推理。

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::time::{Duration, Instant};

use policy::AgentSnapshot;
use types::EntityId;

use super::PolicyKey;
use super::policy_table::PolicyTable;

/// 工作线程参数(D01b-2 §1.4 由配置构造)。
pub struct WorkerOptions {
    /// 每拍最多等回复多久。
    pub wait_budget: Duration,
    /// 每次推理前睡多久(只给测试用,默认 0)。
    pub debug_delay: Duration,
}

/// 一次推理请求。
pub struct InferRequest {
    pub seq: u64,
    pub entity: EntityId,
    pub key: PolicyKey,
    pub snapshot: AgentSnapshot,
}

/// 一次推理回复(`Err` 已转成字符串;key 不在表里也回 `Err`)。
pub struct InferReply {
    pub seq: u64,
    pub entity: EntityId,
    pub result: Result<policy::Action, String>,
    /// 本次 `policy.act` 推理耗时(微秒;策略键缺失的错误路径为 0,REQ-013 C01b-3 §1.2)。
    pub infer_us: u64,
}

/// 测量单次策略推理耗时(微秒):墙钟只用于驱动侧遥测(非确定性遥测面,与 `recv_until` 同口径)。
#[allow(clippy::disallowed_methods)]
fn timed_act<F, T>(f: F) -> (T, u64)
where
    F: FnOnce() -> T,
{
    let t0 = Instant::now();
    let res = f();
    let us = u64::try_from(t0.elapsed().as_micros()).unwrap_or(u64::MAX);
    (res, us)
}

/// 策略工作线程宿主:请求端、回复端、线程句柄、参数、累计发出请求数。
pub struct PolicyWorker {
    /// `Option` 以便 `Drop` 里先丢弃请求端再 join(线程靠请求端断开退出)。
    requests: Option<Sender<InferRequest>>,
    replies: Receiver<InferReply>,
    handle: Option<std::thread::JoinHandle<()>>,
    opts: WorkerOptions,
    requests_sent: u64,
}

impl PolicyWorker {
    /// 起一个线程(名字 "policy-worker"),把整张 `PolicyTable` 的所有权交给它。
    /// 空表也照常起线程(空闲不占 CPU)。
    pub fn spawn(table: PolicyTable, opts: WorkerOptions) -> Self {
        let (req_tx, req_rx) = mpsc::channel::<InferRequest>();
        let (rep_tx, rep_rx) = mpsc::channel::<InferReply>();
        let debug_delay = opts.debug_delay;
        let handle = std::thread::Builder::new()
            .name("policy-worker".to_owned())
            .spawn(move || {
                let mut table = table;
                while let Ok(req) = req_rx.recv() {
                    if !debug_delay.is_zero() {
                        std::thread::sleep(debug_delay);
                    }
                    let (result, infer_us) = match table.get_mut(req.key) {
                        Some(entry) => {
                            timed_act(|| entry.policy.act(&req.snapshot).map_err(|e| e.to_string()))
                        }
                        None => (Err(format!("策略键不在表中:{:?}", req.key)), 0),
                    };
                    if rep_tx
                        .send(InferReply {
                            seq: req.seq,
                            entity: req.entity,
                            result,
                            infer_us,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
            })
            .expect("策略工作线程可起(fail-stop)");
        Self {
            requests: Some(req_tx),
            replies: rep_rx,
            handle: Some(handle),
            opts,
            requests_sent: 0,
        }
    }

    /// 发一条推理请求;`requests_sent += 1`。请求通道断开 = 线程已退出 → fail-stop。
    pub fn submit(&mut self, req: InferRequest) {
        self.requests
            .as_ref()
            .expect("请求端在(drop 前)")
            .send(req)
            .expect("策略工作线程已退出(fail-stop)");
        self.requests_sent += 1;
    }

    /// 不等,取一条已就绪的回复;没有 → None。回复通道断开 → fail-stop。
    pub fn try_recv(&mut self) -> Option<InferReply> {
        match self.replies.try_recv() {
            Ok(reply) => Some(reply),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => panic!("策略工作线程已退出(fail-stop)"),
        }
    }

    /// 等到 `deadline` 取一条回复;到点仍无 → None。回复通道断开 → fail-stop。
    // 墙钟只用于驱动侧等待预算(AC-D3 非确定性遥测面),与 telemetry 计时同口径。
    #[allow(clippy::disallowed_methods)]
    pub fn recv_until(&mut self, deadline: Instant) -> Option<InferReply> {
        match self.replies.try_recv() {
            Ok(reply) => return Some(reply),
            Err(TryRecvError::Disconnected) => panic!("策略工作线程已退出(fail-stop)"),
            Err(TryRecvError::Empty) => {}
        }
        let now = Instant::now();
        if now >= deadline {
            return None;
        }
        match self.replies.recv_timeout(deadline - now) {
            Ok(reply) => Some(reply),
            Err(RecvTimeoutError::Timeout) => None,
            Err(RecvTimeoutError::Disconnected) => panic!("策略工作线程已退出(fail-stop)"),
        }
    }

    /// 每拍最多等回复多久。
    pub fn wait_budget(&self) -> Duration {
        self.opts.wait_budget
    }

    /// 累计发出的推理请求数(遥测)。
    pub fn requests_sent(&self) -> u64 {
        self.requests_sent
    }
}

/// 策略执行器边界(REQ-014 F9;M05b 扩池换实现的缝)。
pub trait PolicyExecutor {
    fn submit(&mut self, req: InferRequest);
    fn try_recv(&mut self) -> Option<InferReply>;
    fn recv_until(&mut self, deadline: std::time::Instant) -> Option<InferReply>;
    fn wait_budget(&self) -> std::time::Duration;
}

impl PolicyExecutor for PolicyWorker {
    fn submit(&mut self, req: InferRequest) {
        PolicyWorker::submit(self, req);
    }

    fn try_recv(&mut self) -> Option<InferReply> {
        PolicyWorker::try_recv(self)
    }

    fn recv_until(&mut self, deadline: std::time::Instant) -> Option<InferReply> {
        PolicyWorker::recv_until(self, deadline)
    }

    fn wait_budget(&self) -> std::time::Duration {
        PolicyWorker::wait_budget(self)
    }
}

impl Drop for PolicyWorker {
    fn drop(&mut self) {
        // 先丢弃请求端:线程 recv 返回 Err 退出循环;再 join。
        drop(self.requests.take());
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}
