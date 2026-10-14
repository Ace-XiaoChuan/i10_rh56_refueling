"""任务与离线工具入口；仅在请求编排类型时加载状态机。"""

__all__ = ["CapTask", "State"]


def __getattr__(name):
    """保留公开接口，让旧 Python 的独立优化环境只加载兼容模块。"""
    if name in __all__:
        from .state_machine import CapTask, State
        return {"CapTask": CapTask, "State": State}[name]
    raise AttributeError(name)
