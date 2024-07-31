from enum import Enum

class MouseMode(Enum):
    NONE = 0
    MOVE_ALL = 1
    SELECT_AREA = 2

class ScaleMode(Enum):
    NONE = 0
    SCALE_ALL = 1

class FIXMode(Enum):
    NONE = 0
    FIX_SHOWING_POS = 1