# Unitree Go2 机器人描述文件(第三方)

- **文件**:`go2_description.urdf`,原样未改。
- **来源**:<https://github.com/unitreerobotics/unitree_ros>,路径 `robots/go2_description/urdf/go2_description.urdf`,提交 `a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76`(2025-06-19)。
- **许可证**:BSD 3-Clause,版权归 HangZhou YuShu TECHNOLOGY CO.,LTD.("Unitree Robotics")。全文见同目录 `LICENSE`。核实日期 2026-11-16(仓库根的 `LICENSE` 文件与 GitHub 的许可证标识一致)。
- **用途**:REQ-012 AC-C3,验证 URDF 导入工具能处理真实模型。`assets/bodies/go2.body.toml` 由 `cargo xtask import-body` 从这份文件生成，属于衍生文件，同样受上述许可证约束。
- **没有带进来的东西**:URDF 里视觉用的 `.dae` 网格。本项目只用碰撞形状(这份 URDF 的碰撞形状全是盒、圆柱、球),视觉元素在导入时被忽略。
