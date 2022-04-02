小车硬件相关代码，已大体上使用 MVC 架构重构，
- `utils.py` 包含一些工具和获取数据的类，可以看作是 MVC 中的 Model；
- `views.py` 包含 GUI 相关组件；
- `robot.py` 为小车运动控制和算法核心逻辑，也有 MVC 中 Controller 的作用（更新 `ControlPanel` 的函数写在了 `Robot` 类里，主要是为了省事）；
- `state.py` 包含了状态机的抽象类。
