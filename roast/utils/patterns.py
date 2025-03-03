"""Design patterns for Roast"""
import threading
from typing import Dict


class Singleton(type):
    """Singleton pattern implementation"""

    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls, *args, **kwargs)
        return cls._instance


class ThreadSafeSingleton(type):
    """Thread-safe Singleton pattern implementation"""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super(ThreadSafeSingleton, cls).__new__(
                        cls, *args, **kwargs
                    )
        return cls._instance


class SingletonMetaclass(type):
    """Singleton pattern implementation for same set of arguments"""

    _instances: Dict[str, object] = {}

    def __call__(cls, *args, **kwargs):
        # FIXME: kwargs frozenset issue
        key = (cls, args, frozenset(kwargs))
        if key not in cls._instances:
            cls._instances[key] = super(SingletonMetaclass, cls).__call__(
                *args, **kwargs
            )
        return cls._instances[key]


class ThreadSafeSingletonMetaclass(type):
    """Thread-safe Singleton pattern implementation for same set of arguments"""

    _instances: Dict[str, object] = {}
    _lock = threading.RLock()

    def __call__(cls, *args, **kwargs):
        key = (cls, args, frozenset(kwargs))
        with cls._lock:
            if key not in cls._instances:
                cls._instances[key] = super(ThreadSafeSingletonMetaclass, cls).__call__(
                    *args, **kwargs
                )
        return cls._instances[key]
