# demos

Python 验证性 demo,配合 [需求驱动开发流程](../docs/process.md) 使用。

## 约定

- 一个 demo 对应一条需求,独立子目录:`demos/<主题>/`
- 目的是低成本验证"需求是否成立、算法是否可行",**不是**生产代码:允许脏、允许硬编码
- 验证通过后,把思路迁移到 `crates/` 的 Rust 实现;发现的问题回写需求文档
- 依赖装在虚拟环境里(`.venv/`、`__pycache__/` 已 gitignore)

## 目录结构示例

```
demos/
└── falling-block/        # 目录名即主题
    ├── README.md         # 对应哪条 REQ、怎么跑、结论
    ├── main.py
    └── ...
```
