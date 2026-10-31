#!/usr/bin/env python3
"""Run the real master_controller against a shifted clock.

argv[1] = path of the node file. $SIM_CLOCK is a file with the offset (s) from
the real clock; the driver rewrites it to jump in time. Time still flows at
real speed. TZ comes from the environment, so DST is the real thing.
"""
import datetime as _dt
import importlib.machinery
import importlib.util
import sys
import time as _time

CLOCK_FILE = __import__('os').environ['SIM_CLOCK']
_cache = [0.0, 0.0]


def offset():
    t = _time.time()
    if t - _cache[0] > 0.1:
        _cache[0] = t
        try:
            _cache[1] = float(open(CLOCK_FILE).read())
        except (OSError, ValueError):
            pass
    return _cache[1]


class FakeTime(object):
    def __getattr__(self, name):
        return getattr(_time, name)

    @staticmethod
    def time():
        return _time.time() + offset()


class FakeDatetimeClass(_dt.datetime):
    @classmethod
    def now(cls, tz=None):
        d = _dt.datetime.fromtimestamp(FakeTime.time())
        return cls(d.year, d.month, d.day, d.hour, d.minute, d.second, d.microsecond)


class FakeDatetimeModule(object):
    datetime = FakeDatetimeClass
    date = _dt.date
    time = _dt.time
    timedelta = _dt.timedelta


path = sys.argv[1]
sys.argv = [path] + sys.argv[2:]
loader = importlib.machinery.SourceFileLoader('master_under_sim', path)
spec = importlib.util.spec_from_loader(loader.name, loader)
M = importlib.util.module_from_spec(spec)
loader.exec_module(M)
M.time = FakeTime()
M.datetime = FakeDatetimeModule()
controller = M.MasterController()
controller.run()
