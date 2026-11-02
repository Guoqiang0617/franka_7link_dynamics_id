# 原 FFS 仿真结果归档

本目录保存用户指定原目录的两张图及配套材料：

```text
ffs_data/src/franka_identification_controllers/mujoco_validation/results/ffs_simulation/
```

原生成器是 `src/franka_identification_controllers/mujoco_validation/scripts/5_ffs_simulation.py`，归档名为 `original_5_ffs_simulation.py`。旧辨识和验证脚本、`pc_irls_improved_params_simple.npz` 也保存用于查证参数来源。`PROVENANCE.json` 列出来源目录、逐文件 SHA-256 和轨迹匹配证据。

历史 NPZ 中的期望位置与测试 JSON **350** 相同，最大差为 0；旧脚本默认写 391，不能据此判断原图用的是哪条轨迹。

这些是原始快照，保持其数值和图片不变。脚本含旧路径、旧数据结构和仿真近似，仅供阅读，不作为可移植执行入口。新程序的结果位于主工程 `results/`，原图与新图差别见 [模型导出与仿真](../../docs/MODEL_EXPORT_AND_SIMULATION.md)。

旧参数可能与新文件采用不同惯性参考点/元信息约定；新下游读取器会拒绝缺少排列和参考点标签的 NPZ。应从包内已有数据重新运行新辨识流程。
