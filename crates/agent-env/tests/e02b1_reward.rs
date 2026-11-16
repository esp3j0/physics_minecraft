//! REQ-012 E02b-1 §1:`reward_columns` 纯 Rust 测试(键序、宽度、数值逐位来自
//! `RewardInputs`)。不依赖 python 特性。

use agent_env::{RewardInputs, reward_columns, reward_key_widths};

fn ri(base: f32) -> RewardInputs {
    RewardInputs {
        base_linvel_body: [base + 1.0, base + 2.0, base + 3.0],
        base_angvel_body: [base + 4.0, base + 5.0, base + 6.0],
        projected_gravity: [base + 7.0, base + 8.0, base + 9.0],
        root_height: base + 10.0,
        up_y: base + 11.0,
        joint_pos: vec![base + 12.0, base + 13.0],
        joint_vel: vec![base + 14.0, base + 15.0],
        action: vec![base + 16.0, base + 17.0],
        prev_action: vec![base + 18.0, base + 19.0],
        targets_clamped: (base + 20.0) as u32,
        intent: [base + 21.0, base + 22.0, base + 23.0],
    }
}

/// R1:键序与宽度等于固定表(dof = 2)。
#[test]
fn r1_key_order_and_widths() {
    let cols = reward_columns(&[ri(0.0), ri(100.0)]);
    let want = reward_key_widths(2);
    assert_eq!(cols.len(), 11);
    for (i, (name, k, _)) in cols.iter().enumerate() {
        assert_eq!((*name, *k), want[i], "第 {i} 键");
    }
    let names: Vec<&str> = cols.iter().map(|(n, _, _)| *n).collect();
    assert_eq!(
        names,
        [
            "base_linvel_body",
            "base_angvel_body",
            "projected_gravity",
            "root_height",
            "up_y",
            "joint_pos",
            "joint_vel",
            "action",
            "prev_action",
            "targets_clamped",
            "intent",
        ]
    );
}

/// R2:每键数据按环境下标拼接,逐位等于 `RewardInputs` 对应字段。
#[test]
fn r2_values_bit_exact() {
    let cols = reward_columns(&[ri(0.0), ri(100.0)]);
    // 每键的期望列(env0 段 + env1 段),手填,不走被测函数。
    let expect: Vec<Vec<f32>> = vec![
        vec![1.0, 2.0, 3.0, 101.0, 102.0, 103.0],
        vec![4.0, 5.0, 6.0, 104.0, 105.0, 106.0],
        vec![7.0, 8.0, 9.0, 107.0, 108.0, 109.0],
        vec![10.0, 110.0],
        vec![11.0, 111.0],
        vec![12.0, 13.0, 112.0, 113.0],
        vec![14.0, 15.0, 114.0, 115.0],
        vec![16.0, 17.0, 116.0, 117.0],
        vec![18.0, 19.0, 118.0, 119.0],
        vec![20.0, 120.0],
        vec![21.0, 22.0, 23.0, 121.0, 122.0, 123.0],
    ];
    for (i, (_, k, data)) in cols.iter().enumerate() {
        assert_eq!(data.len(), 2 * k, "第 {i} 键长度");
        for (j, (&got, &want)) in data.iter().zip(&expect[i]).enumerate() {
            assert_eq!(got.to_bits(), want.to_bits(), "第 {i} 键第 {j} 个");
        }
    }
    // action 与 prev_action 是不同列(对调变异在此露馅)。
    let action = &cols[7].2;
    let prev = &cols[8].2;
    assert_ne!(action, prev, "action 与 prev_action 不应相同");
    assert_eq!(action[0].to_bits(), ri(0.0).action[0].to_bits());
    assert_eq!(prev[0].to_bits(), ri(0.0).prev_action[0].to_bits());
}

/// R3:空切片 → 11 键、各键空列、dof 相关宽度 0。
#[test]
fn r3_empty() {
    let cols = reward_columns(&[]);
    assert_eq!(cols.len(), 11);
    for (name, k, data) in &cols {
        assert!(data.is_empty(), "{name} 应空");
        assert_eq!(
            *k,
            reward_key_widths(0)
                .iter()
                .find(|(n, _)| n == name)
                .unwrap()
                .1
        );
    }
}
