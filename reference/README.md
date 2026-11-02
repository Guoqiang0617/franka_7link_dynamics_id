# 历史参考与模型依据

此目录用于追溯，不属于默认执行路径。

- `legacy_ffs_simulation/`：原两张图、原仿真时序/XML、旧 PC-IRLS/验证/仿真脚本、旧参数与哈希清单。见其 [说明](legacy_ffs_simulation/README.md)。
- `fr3_description/`：FR3 名义模型的 YAML、原扩展 URDF 和上游 LICENSE/NOTICE。当前工程保留并引用这些来源；主模型已移除外部 mesh 文件依赖。
- `trajectory_generation/`：从原 `src/franka_ffs_controllers/scripts/ffs_trajectory_generator.py` 归档的旧 ROS 生成器。新生成入口统一为主工程的 `scripts/generate_ffs_trajectories.py`。

归档脚本可能含旧机器绝对路径、ROS 导入、旧格式惯性约定或已修正的仿真处理。不要把它们当成当前 Windows 的入口，不要直接把旧 NPZ 改名代替新 `identified_params.npz`。新流程对应关系见 [项目说明](../docs/PROJECT_MAP.md)。

这里没有归档末端外置力传感器 + IMU + 待辨识零件的算法和数据。
