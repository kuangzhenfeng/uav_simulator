# 领域文档

说明工程类 skill 在探索代码库时如何消费本仓库的领域文档。

## 探索前先读取

- 根目录的 **`CONTEXT.md`**,或
- 根目录的 **`CONTEXT-MAP.md`**(若存在)—— 它指向每个上下文各自的 `CONTEXT.md`。读取与主题相关的每一份。
- **`docs/adr/`** —— 读取与你即将处理区域相关的 ADR。多上下文仓库中,还需检查 `src/<context>/docs/adr/` 中的上下文级决策。

如果上述任一文件不存在,**静默继续**。不要提示其缺失,也不要建议预先创建。生产者 skill(`/grill-with-docs`)会在术语或决策实际定型时**懒创建**它们。

## 文件结构

单上下文仓库(大多数仓库):

```
/
├── CONTEXT.md
├── docs/adr/
│   ├── 0001-event-sourced-orders.md
│   └── 0002-postgres-for-write-model.md
└── src/
```

多上下文仓库(根目录存在 `CONTEXT-MAP.md`):

```
/
├── CONTEXT-MAP.md
├── docs/adr/                          ← 系统级决策
└── src/
    ├── ordering/
    │   ├── CONTEXT.md
    │   └── docs/adr/                  ← 上下文级决策
    └── billing/
        ├── CONTEXT.md
        └── docs/adr/
```

## 使用术语表中的词汇

当你的输出命名某个领域概念时(在 issue 标题、重构提案、假设、测试名中),使用 `CONTEXT.md` 中定义的术语。不要漂移到术语表明确避免的同义词。

如果你需要的概念尚不在术语表中,这是一个信号 —— 要么你在发明项目并不使用的语言(重新考虑),要么存在真实的缺口(标注给 `/grill-with-docs` 处理)。

## 标记 ADR 冲突

如果你的输出与某个已有 ADR 矛盾,请明确指出,而非静默覆盖:

> _与 ADR-0007(事件溯源订单)冲突 —— 但值得重开,因为…_
