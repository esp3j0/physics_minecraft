//! 场景脚本解析(REQ-008 §3.1–§3.3;AC-S1)。
//!
//! 错误要带步骤序号(从 1 起),整体 serde 反序列化拿不到,故按放行评论风险 1 的建议:
//! 先解析为 [`toml::Table`] 再逐步骤手工校验;每个动作表用 `toml::de::Value` 逐键读取,
//! 未知键显式拒绝。零 macroquad、零恐慌路径(数值转换错误一律返回 `ScriptError`)。

use types::glam::Vec3;
use types::{BlockId, BlockPos};
use voxel::{BlockRegistry, SimpleRegistry};

use super::{Action, Check, Cmp, DetonateAt, Scenario, ScriptError, SpawnAt, Step};

/// `[viewer]` 下已知的键。
const VIEWER_KEYS: [&str; 1] = ["view_distance"];
/// 顶层已知键(`step` 是 `[[step]]` 数组;`server`/`viewer` 是表,单独处理)。
const TOP_KEYS: [&str; 7] = [
    "name",
    "req",
    "description",
    "timeout_secs",
    "server",
    "viewer",
    "step",
];
/// 每步已知键(动作键之外)。
const STEP_KEYS: [&str; 1] = ["label"];
/// 全部动作键(与 [`Action`] 一一对应)。
const ACTION_KEYS: [&str; 12] = [
    "camera",
    "look_at",
    "fly",
    "couple",
    "spawn",
    "detonate",
    "set_block",
    "fill",
    "wait_ticks",
    "wait_until",
    "check",
    "screenshot",
];

fn top_err(message: impl Into<String>) -> ScriptError {
    ScriptError {
        step: None,
        message: message.into(),
    }
}

fn step_err(step: usize, message: impl Into<String>) -> ScriptError {
    ScriptError {
        step: Some(step),
        message: message.into(),
    }
}

/// 读坐标数组:恰好 3 个数,整数可读作浮点(细则 12)。
fn vec3(value: &toml::Value, what: &str) -> Result<Vec3, String> {
    let a = value
        .as_array()
        .ok_or_else(|| format!("{what} 应为 [x, y, z] 数组"))?;
    if a.len() != 3 {
        return Err(format!("{what} 应恰好 3 个数,实为 {} 个", a.len()));
    }
    let mut v = [0.0_f32; 3];
    for (i, item) in a.iter().enumerate() {
        v[i] = item
            .as_float()
            .or_else(|| item.as_integer().map(|n| n as f64))
            .map(|f| f as f32)
            .ok_or_else(|| format!("{what} 第 {} 项应为数", i + 1))?;
    }
    Ok(Vec3::from_array(v))
}

/// 读整数坐标数组(BlockPos;同细则 12:恰好 3 个数,浮点可读作整数——整数位置写成
/// 浮点是合法 TOML)。
fn block_pos(value: &toml::Value, what: &str) -> Result<BlockPos, String> {
    let a = value
        .as_array()
        .ok_or_else(|| format!("{what} 应为 [x, y, z] 数组"))?;
    if a.len() != 3 {
        return Err(format!("{what} 应恰好 3 个数,实为 {} 个", a.len()));
    }
    let mut v = [0_i32; 3];
    for (i, item) in a.iter().enumerate() {
        let f = item
            .as_float()
            .or_else(|| item.as_integer().map(|n| n as f64))
            .ok_or_else(|| format!("{what} 第 {} 项应为数", i + 1))?;
        if !f.is_finite() || f.fract() != 0.0 {
            return Err(format!("{what} 第 {} 项应为整数坐标", i + 1));
        }
        v[i] = f as i32;
    }
    Ok(BlockPos::new(v[0], v[1], v[2]))
}

/// 拒绝表中的未知键。
fn reject_unknown(table: &toml::Table, known: &[&str], what: &str) -> Result<(), String> {
    for key in table.keys() {
        if !known.contains(&key.as_str()) {
            return Err(format!("{what} 含未知键 \"{key}\""));
        }
    }
    Ok(())
}

/// 比较项 `{ eq = n }` / `{ le = n }` / `{ ge = n }`,恰好一个(细则 12)。
fn cmp(value: &toml::Value, what: &str) -> Result<Cmp, String> {
    let t = value
        .as_table()
        .ok_or_else(|| format!("{what} 应为 {{ eq = n }} 形式的表"))?;
    let keys: Vec<&String> = t.keys().collect();
    if keys.len() != 1 {
        return Err(format!(
            "{what} 应恰好给出 eq/le/ge 之一,实为 {} 个",
            keys.len()
        ));
    }
    let key = keys[0].as_str();
    let n = t
        .get(key)
        .and_then(toml::Value::as_integer)
        .ok_or_else(|| format!("{what} 的 {key} 应为非负整数"))?;
    let n = u64::try_from(n).map_err(|_| format!("{what} 的 {key} 应为非负整数"))?;
    match key {
        "eq" => Ok(Cmp::Eq(n)),
        "le" => Ok(Cmp::Le(n)),
        "ge" => Ok(Cmp::Ge(n)),
        other => Err(format!("{what} 的键应为 eq/le/ge,实为 \"{other}\"")),
    }
}

/// 方块名 → ID(经内置注册表;未知名字报错,不当成空气)。
fn block_id(value: &toml::Value, registry: &SimpleRegistry, what: &str) -> Result<BlockId, String> {
    let name = value
        .as_str()
        .ok_or_else(|| format!("{what} 应为方块名字符串"))?;
    registry
        .id_of(name)
        .ok_or_else(|| format!("{what}:未知方块名 \"{name}\""))
}

fn f32_of(value: &toml::Value, what: &str) -> Result<f32, String> {
    value
        .as_float()
        .or_else(|| value.as_integer().map(|n| n as f64))
        .map(|f| f as f32)
        .ok_or_else(|| format!("{what} 应为数"))
}

fn u64_of(value: &toml::Value, what: &str) -> Result<u64, String> {
    let n = value
        .as_integer()
        .ok_or_else(|| format!("{what} 应为非负整数"))?;
    u64::try_from(n).map_err(|_| format!("{what} 应为非负整数"))
}

fn str_of(value: &toml::Value, what: &str) -> Result<String, String> {
    value
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{what} 应为字符串"))
}

fn bool_of(value: &toml::Value, what: &str) -> Result<bool, String> {
    value.as_bool().ok_or_else(|| format!("{what} 应为布尔值"))
}

/// 解析一个动作表(键名 + 值)为 [`Action`]。
fn parse_action(
    key: &str,
    value: &toml::Value,
    step: usize,
    registry: &SimpleRegistry,
) -> Result<Action, ScriptError> {
    let e = |m: String| step_err(step, m);
    let t = value
        .as_table()
        .ok_or_else(|| e(format!("动作 {key} 应为内联表(如 {key} = {{ .. }})")))?;
    // 先拒绝未知键,再读字段。
    let known: &[&str] = match key {
        "camera" => &["pos", "yaw", "pitch"],
        "look_at" => &["target"],
        "fly" => &["to", "speed"],
        "couple" => &["on"],
        "spawn" => &[
            "at", "ahead", "lift", "count", "spacing", "velocity", "label",
        ],
        "detonate" => &["at", "nearest", "power", "radius"],
        "set_block" => &["pos", "block"],
        "fill" => &["from", "to", "block"],
        "wait_ticks" => &["n"],
        "wait_until" => &[
            "live_entities",
            "labeled_alive",
            "block",
            "chunks_loaded",
            "no_floating",
            "timeout_ticks",
        ],
        "check" => &[
            "live_entities",
            "labeled_alive",
            "block",
            "chunks_loaded",
            "no_floating",
        ],
        "screenshot" => &["name", "expect", "hud"],
        other => return Err(e(format!("未知动作 \"{other}\""))),
    };
    reject_unknown(t, known, &format!("动作 {key}")).map_err(e)?;

    match key {
        "camera" => {
            let pos = t
                .get("pos")
                .ok_or_else(|| e("动作 camera 缺必填参数 pos".into()))?;
            let yaw = t
                .get("yaw")
                .ok_or_else(|| e("动作 camera 缺必填参数 yaw".into()))?;
            let pitch = t
                .get("pitch")
                .ok_or_else(|| e("动作 camera 缺必填参数 pitch".into()))?;
            Ok(Action::Camera {
                pos: vec3(pos, "camera.pos").map_err(e)?,
                yaw: f32_of(yaw, "camera.yaw").map_err(e)?,
                pitch: f32_of(pitch, "camera.pitch").map_err(e)?,
            })
        }
        "look_at" => {
            let target = t
                .get("target")
                .ok_or_else(|| e("动作 look_at 缺必填参数 target".into()))?;
            Ok(Action::LookAt {
                target: vec3(target, "look_at.target").map_err(e)?,
            })
        }
        "fly" => {
            let to = t
                .get("to")
                .ok_or_else(|| e("动作 fly 缺必填参数 to".into()))?;
            let speed = match t.get("speed") {
                Some(v) => f32_of(v, "fly.speed").map_err(e)?,
                None => 16.0,
            };
            Ok(Action::Fly {
                to: vec3(to, "fly.to").map_err(e)?,
                speed,
            })
        }
        "couple" => {
            let on = t
                .get("on")
                .ok_or_else(|| e("动作 couple 缺必填参数 on".into()))?;
            Ok(Action::Couple {
                on: bool_of(on, "couple.on").map_err(e)?,
            })
        }
        "spawn" => {
            let at = match (t.get("at"), t.get("ahead")) {
                (Some(v), None) => SpawnAt::At(vec3(v, "spawn.at").map_err(e)?),
                (None, Some(ahead)) => SpawnAt::Ahead {
                    ahead: f32_of(ahead, "spawn.ahead").map_err(e)?,
                    lift: match t.get("lift") {
                        Some(v) => f32_of(v, "spawn.lift").map_err(e)?,
                        None => return Err(e("动作 spawn 给了 ahead 就必须给 lift".into())),
                    },
                },
                (Some(_), Some(_)) => {
                    return Err(e("动作 spawn 的 at 与 ahead/lift 只能给一组".into()));
                }
                (None, None) => {
                    return Err(e("动作 spawn 缺生成位置:给 at 或 ahead/lift".into()));
                }
            };
            let count = match t.get("count") {
                Some(v) => u64_of(v, "spawn.count").map_err(e)?,
                None => 1,
            };
            let count =
                u32::try_from(count).map_err(|_| e("动作 spawn 的 count 应为非负整数".into()))?;
            let spacing = match t.get("spacing") {
                Some(v) => f32_of(v, "spawn.spacing").map_err(e)?,
                None => 1.6,
            };
            let velocity = match t.get("velocity") {
                Some(v) => vec3(v, "spawn.velocity").map_err(e)?,
                None => Vec3::ZERO,
            };
            let label = match t.get("label") {
                Some(v) => Some(str_of(v, "spawn.label").map_err(e)?),
                None => None,
            };
            Ok(Action::Spawn {
                at,
                count,
                spacing,
                velocity,
                label,
            })
        }
        "detonate" => {
            let at = match (t.get("at"), t.get("nearest")) {
                (Some(v), None) => DetonateAt::At(vec3(v, "detonate.at").map_err(e)?),
                (None, Some(n)) => {
                    if bool_of(n, "detonate.nearest").map_err(e)? {
                        DetonateAt::Nearest
                    } else {
                        return Err(e(
                            "动作 detonate 的 nearest = false 无意义:给 at 或 nearest = true"
                                .into(),
                        ));
                    }
                }
                (Some(_), Some(_)) => {
                    return Err(e("动作 detonate 的 at 与 nearest 只能给一组".into()));
                }
                (None, None) => {
                    return Err(e("动作 detonate 缺爆心:给 at 或 nearest = true".into()));
                }
            };
            let power = t
                .get("power")
                .ok_or_else(|| e("动作 detonate 缺必填参数 power".into()))?;
            let radius = t
                .get("radius")
                .ok_or_else(|| e("动作 detonate 缺必填参数 radius".into()))?;
            Ok(Action::Detonate {
                at,
                power: f32_of(power, "detonate.power").map_err(e)?,
                radius: f32_of(radius, "detonate.radius").map_err(e)?,
            })
        }
        "set_block" => {
            let pos = t
                .get("pos")
                .ok_or_else(|| e("动作 set_block 缺必填参数 pos".into()))?;
            let block = t
                .get("block")
                .ok_or_else(|| e("动作 set_block 缺必填参数 block".into()))?;
            Ok(Action::SetBlock {
                pos: block_pos(pos, "set_block.pos").map_err(e)?,
                block: block_id(block, registry, "set_block.block").map_err(e)?,
            })
        }
        // **总架构师接口桩(REQ-009 单元 01)**:由实施方实现(缺参、类型错误、未知方块、
        // 超过 FILL_MAX_CELLS 都报带步骤序号的错误)。
        "fill" => unimplemented!("REQ-009 单元 01:由实施方实现"),
        "wait_ticks" => {
            let n = t
                .get("n")
                .ok_or_else(|| e("动作 wait_ticks 缺必填参数 n".into()))?;
            Ok(Action::WaitTicks {
                n: u64_of(n, "wait_ticks.n").map_err(e)?,
            })
        }
        "wait_until" => {
            let timeout = t
                .get("timeout_ticks")
                .ok_or_else(|| e("动作 wait_until 缺必填参数 timeout_ticks".into()))?;
            let checks = parse_checks(t, "wait_until", registry).map_err(e)?;
            Ok(Action::WaitUntil {
                checks,
                timeout_ticks: u64_of(timeout, "wait_until.timeout_ticks").map_err(e)?,
            })
        }
        "check" => {
            let checks = parse_checks(t, "check", registry).map_err(e)?;
            if checks.is_empty() {
                return Err(e("动作 check 至少要有一个检查项".into()));
            }
            Ok(Action::Check { checks })
        }
        "screenshot" => {
            let name = t
                .get("name")
                .ok_or_else(|| e("动作 screenshot 缺必填参数 name".into()))?;
            let expect = t
                .get("expect")
                .ok_or_else(|| e("动作 screenshot 缺必填参数 expect".into()))?;
            let hud = match t.get("hud") {
                Some(v) => bool_of(v, "screenshot.hud").map_err(e)?,
                None => false,
            };
            Ok(Action::Screenshot {
                name: str_of(name, "screenshot.name").map_err(e)?,
                expect: str_of(expect, "screenshot.expect").map_err(e)?,
                hud,
            })
        }
        // reject_unknown + 上面的 match 已穷尽 ACTION_KEYS。
        other => Err(e(format!("未知动作 \"{other}\""))),
    }
}

/// 解析检查项表:键是检查项名,值是参数表。
fn parse_checks(
    t: &toml::Table,
    what: &str,
    registry: &SimpleRegistry,
) -> Result<Vec<Check>, String> {
    let mut checks = Vec::new();
    for (key, value) in t {
        match key.as_str() {
            "live_entities" => checks.push(Check::LiveEntities(cmp(value, "live_entities")?)),
            "labeled_alive" => {
                let ct = value.as_table().ok_or_else(|| {
                    "labeled_alive 应为 { label = \"..\", .. } 形式的表".to_owned()
                })?;
                reject_unknown(ct, &["label", "eq", "le", "ge"], "labeled_alive")?;
                let label = ct
                    .get("label")
                    .ok_or_else(|| "labeled_alive 缺 label".to_owned())
                    .and_then(|v| str_of(v, "labeled_alive.label"))?;
                let cmp = cmp_of_inline(ct, "labeled_alive")?;
                checks.push(Check::LabeledAlive { label, cmp });
            }
            "block" => {
                let bt = value
                    .as_table()
                    .ok_or_else(|| "block 应为 { pos = [..], is = \"..\" } 形式的表".to_owned())?;
                reject_unknown(bt, &["pos", "is"], "block")?;
                let pos = bt
                    .get("pos")
                    .ok_or_else(|| "block 缺 pos".to_owned())
                    .and_then(|v| block_pos(v, "block.pos"))?;
                let is = bt
                    .get("is")
                    .ok_or_else(|| "block 缺 is".to_owned())
                    .and_then(|v| block_id(v, registry, "block.is"))?;
                checks.push(Check::Block { pos, is });
            }
            "chunks_loaded" => checks.push(Check::ChunksLoaded(cmp(value, "chunks_loaded")?)),
            "no_floating" => {
                let nt = value.as_table().ok_or_else(|| {
                    "no_floating 应为 { height = .., still_ticks = .. } 形式的表".to_owned()
                })?;
                reject_unknown(nt, &["height", "still_ticks"], "no_floating")?;
                let height = nt
                    .get("height")
                    .ok_or_else(|| "no_floating 缺 height".to_owned())
                    .and_then(|v| f32_of(v, "no_floating.height"))?;
                let still_ticks = nt
                    .get("still_ticks")
                    .ok_or_else(|| "no_floating 缺 still_ticks".to_owned())
                    .and_then(|v| u64_of(v, "no_floating.still_ticks"))?;
                checks.push(Check::NoFloating {
                    height,
                    still_ticks,
                });
            }
            // timeout_ticks 只属于 wait_until 本身,不是检查项。
            "timeout_ticks" if what == "wait_until" => {}
            other => return Err(format!("{what} 含未知检查项 \"{other}\"")),
        }
    }
    Ok(checks)
}

/// 从 `labeled_alive` 这类内联表里读恰好一个 eq/le/ge(与 [`cmp`] 不同的形态:键与 label 平级)。
fn cmp_of_inline(t: &toml::Table, what: &str) -> Result<Cmp, String> {
    let mut got = None;
    for key in ["eq", "le", "ge"] {
        if let Some(v) = t.get(key) {
            if got.is_some() {
                return Err(format!("{what} 应恰好给出 eq/le/ge 之一"));
            }
            let n = v
                .as_integer()
                .ok_or_else(|| format!("{what} 的 {key} 应为非负整数"))?;
            let n = u64::try_from(n).map_err(|_| format!("{what} 的 {key} 应为非负整数"))?;
            got = Some(match key {
                "eq" => Cmp::Eq(n),
                "le" => Cmp::Le(n),
                _ => Cmp::Ge(n),
            });
        }
    }
    got.ok_or_else(|| format!("{what} 缺比较项(eq/le/ge 之一)"))
}

/// 解析场景文本(REQ-008 §3.1–§3.3)。
pub(super) fn parse_scenario(text: &str) -> Result<Scenario, ScriptError> {
    let root: toml::Table =
        toml::from_str(text).map_err(|e| top_err(format!("TOML 解析失败:{e}")))?;
    reject_unknown(&root, &TOP_KEYS, "场景顶层").map_err(top_err)?;

    let name = root
        .get("name")
        .ok_or_else(|| top_err("场景缺必填键 name"))?;
    let name = str_of(name, "name").map_err(top_err)?;

    let req = match root.get("req") {
        Some(v) => Some(str_of(v, "req").map_err(top_err)?),
        None => None,
    };
    let description = match root.get("description") {
        Some(v) => str_of(v, "description").map_err(top_err)?,
        None => String::new(),
    };
    let timeout_secs = match root.get("timeout_secs") {
        Some(v) => u64_of(v, "timeout_secs").map_err(top_err)?,
        None => 180,
    };
    let server = match root.get("server") {
        Some(v) => v
            .as_table()
            .cloned()
            .ok_or_else(|| top_err("[server] 应为表"))?,
        None => toml::Table::new(),
    };
    let view_distance = match root.get("viewer") {
        Some(v) => {
            let vt = v.as_table().ok_or_else(|| top_err("[viewer] 应为表"))?;
            reject_unknown(vt, &VIEWER_KEYS, "[viewer]").map_err(top_err)?;
            match vt.get("view_distance") {
                Some(n) => {
                    let n = u64_of(n, "viewer.view_distance").map_err(top_err)?;
                    let n = u8::try_from(n)
                        .map_err(|_| top_err("viewer.view_distance 应为 0–255 的整数"))?;
                    Some(n)
                }
                None => None,
            }
        }
        None => None,
    };

    let registry = SimpleRegistry::builtin();
    let mut steps = Vec::new();
    for (i, value) in root
        .get("step")
        .and_then(toml::Value::as_array)
        .map(Vec::as_slice)
        .unwrap_or(&[])
        .iter()
        .enumerate()
    {
        let step = i + 1; // 步骤序号从 1 开始(细则 1)。
        let t = value
            .as_table()
            .ok_or_else(|| step_err(step, "[[step]] 应为表"))?;
        // 步骤表已知键 = label + 全部动作键;动作键之外再逐一校验。
        let step_known: Vec<&str> = STEP_KEYS
            .iter()
            .copied()
            .chain(ACTION_KEYS.iter().copied())
            .collect();
        reject_unknown(t, &step_known, &format!("第 {step} 步")).map_err(|m| step_err(step, m))?;
        let label = match t.get("label") {
            Some(v) => Some(str_of(v, "step.label").map_err(|m| step_err(step, m))?),
            None => None,
        };
        let action_keys: Vec<&String> = t.keys().filter(|k| k.as_str() != "label").collect();
        if action_keys.len() != 1 {
            return Err(step_err(
                step,
                format!("每步应恰好一个动作键,实为 {} 个", action_keys.len()),
            ));
        }
        let key = action_keys[0].as_str();
        if !ACTION_KEYS.contains(&key) {
            return Err(step_err(step, format!("未知动作 \"{key}\"")));
        }
        let action = parse_action(key, &t[key], step, &registry)?;
        steps.push(Step { label, action });
    }

    Ok(Scenario {
        name,
        req,
        description,
        timeout_secs,
        server,
        view_distance,
        steps,
    })
}

#[cfg(test)]
#[path = "parse_tests.rs"]
mod tests;
