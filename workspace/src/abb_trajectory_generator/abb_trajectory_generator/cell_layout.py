"""Geometry of the work cell, in the world frame (= robot base), metres.

    tool table  (y < 0): mill and laser standing in holders
    mill table  (x > 0): workpiece for milling
    laser table (y > 0): sheet for laser cutting

Tools hang along +z of tool0, which points down while working
(TOOL_DOWN_RPY_DEG).
"""

from dataclasses import dataclass, field

TABLE_HEIGHT = 0.4
TOOL_DOWN_RPY_DEG = (180.0, 0.0, -90.0)

GREY = (0.55, 0.55, 0.58, 1.0)
DARK = (0.2, 0.2, 0.22, 1.0)
STEEL = (0.75, 0.75, 0.78, 1.0)


@dataclass
class Box:
    name: str
    center: tuple
    size: tuple
    color: tuple


@dataclass
class ToolPart:
    shape: str           # 'cylinder' (size = radius) or 'box' (size = side)
    size: float
    z_from: float        # along tool0 +z
    z_to: float
    color: tuple


@dataclass
class Tool:
    name: str
    label: str
    slot: tuple          # (x, y) of the holder on the tool table
    parts: list = field(default_factory=list)

    @property
    def tip(self):
        """Distance from tool0 to the working point."""
        return max(p.z_to for p in self.parts)

    @property
    def grasp_z(self):
        """tool0 height when the tool stands in its holder."""
        return TABLE_HEIGHT + HOLDER_CLEARANCE + self.tip


HOLDER_CLEARANCE = 0.01
HOLDER_RADIUS = 0.06
HOLDER_HEIGHT = 0.06

TABLES = [
    Box('tool_table', (0.55, -0.75, TABLE_HEIGHT / 2), (0.6, 0.35, TABLE_HEIGHT), GREY),
    Box('mill_table', (0.95, 0.0, TABLE_HEIGHT / 2), (0.4, 0.6, TABLE_HEIGHT), GREY),
    Box('laser_table', (0.55, 0.75, TABLE_HEIGHT / 2), (0.5, 0.4, TABLE_HEIGHT), GREY),
    # workpieces
    Box('mill_workpiece', (0.95, 0.0, TABLE_HEIGHT + 0.03), (0.25, 0.4, 0.06),
        (0.8, 0.82, 0.85, 1.0)),
    Box('laser_sheet', (0.55, 0.75, TABLE_HEIGHT + 0.0025), (0.4, 0.3, 0.005),
        (0.6, 0.62, 0.66, 1.0)),
]

MILL = Tool('mill', 'фреза', (0.42, -0.75), [
    ToolPart('cylinder', 0.05, 0.0, 0.02, DARK),               # changer adapter
    ToolPart('cylinder', 0.04, 0.02, 0.18, (0.1, 0.3, 0.7, 1.0)),  # spindle
    ToolPart('cylinder', 0.015, 0.18, 0.21, STEEL),            # collet
    ToolPart('cylinder', 0.005, 0.21, 0.26, (0.85, 0.7, 0.2, 1.0)),  # end mill
])

LASER = Tool('laser', 'лазер', (0.68, -0.75), [
    ToolPart('cylinder', 0.05, 0.0, 0.02, DARK),               # changer adapter
    ToolPart('box', 0.07, 0.02, 0.16, (0.7, 0.1, 0.1, 1.0)),   # laser head
    ToolPart('cylinder', 0.012, 0.16, 0.20, (0.8, 0.5, 0.2, 1.0)),  # nozzle
])

TOOLS = {MILL.name: MILL, LASER.name: LASER}

# Milling: raster over the workpiece top, end mill touching the surface
MILL_SURFACE_Z = TABLE_HEIGHT + 0.06
MILL_AREA = ((0.87, 1.03), (-0.15, 0.15))   # x range, y range
MILL_STEP = 0.02

# Laser: contour above the sheet
LASER_SURFACE_Z = TABLE_HEIGHT + 0.005
LASER_STANDOFF = 0.01
LASER_CENTER = (0.55, 0.75)
LASER_RADIUS = 0.1

APPROACH_HEIGHT = 0.12   # above grasp / work start, reached with a collision-free motion
