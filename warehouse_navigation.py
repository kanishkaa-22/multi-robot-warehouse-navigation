"""
╔══════════════════════════════════════════════════════════════════════════════╗
║        AI Multi-Robot Warehouse Navigation System                          ║
║        ─────────────────────────────────────────────                       ║
║  • A* Search with Space-Time conflict avoidance                            ║
║  • BFS comparison                                                          ║
║  • Priority-based Cooperative Pathfinding                                  ║
║  • Tkinter GUI with live animation                                         ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import random
from collections import deque
from typing import List, Tuple, Dict, Optional, Set

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS & THEME
# ─────────────────────────────────────────────────────────────────────────────

# Dark Navy theme
C_BG        = "#0A1628"   # main background
C_PANEL     = "#0F2044"   # left panel
C_CARD      = "#162840"   # card / row background
C_ACCENT    = "#00D4FF"   # cyan accent
C_GREEN     = "#02C39A"   # success green
C_PURPLE    = "#7B2FBE"   # purple
C_AMBER     = "#FFB347"   # amber
C_RED       = "#FF6B6B"   # red / warning
C_TEXT      = "#B0C4DE"   # body text
C_MUTED     = "#647A8F"   # muted text
C_GRID_FREE = "#0D1E36"   # free cell
C_GRID_WALL = "#2A3A4A"   # obstacle cell
C_GOAL_BG   = "#0D2A1E"   # goal cell tint

# Robot colours (up to 6 robots)
ROBOT_COLORS = ["#00D4FF", "#CC44FF", "#02C39A", "#FFB347", "#FF6B6B", "#FFD700"]

# Fonts
F_HEAD  = ("Trebuchet MS", 11, "bold")
F_BODY  = ("Calibri", 10)
F_SMALL = ("Calibri", 9)
F_STAT  = ("Arial Black", 13)
F_MONO  = ("Consolas", 9)


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: Grid
# ─────────────────────────────────────────────────────────────────────────────

class Grid:
    """
    Represents the warehouse floor as a 2D grid.
    0 = free cell, 1 = obstacle/wall.
    """

    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self.obstacles: Set[Tuple[int, int]] = set()

    # ── Obstacle management ────────────────────────────────────────────────

    def add_obstacle(self, row: int, col: int):
        """Add an obstacle at (row, col)."""
        if self.is_valid(row, col):
            self.obstacles.add((row, col))

    def remove_obstacle(self, row: int, col: int):
        """Remove an obstacle at (row, col)."""
        self.obstacles.discard((row, col))

    def reset(self):
        """Clear all obstacles."""
        self.obstacles.clear()

    # ── Query methods ──────────────────────────────────────────────────────

    def is_valid(self, row: int, col: int) -> bool:
        """Check bounds."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_obstacle(self, row: int, col: int) -> bool:
        return (row, col) in self.obstacles

    def is_free(self, row: int, col: int) -> bool:
        return self.is_valid(row, col) and not self.is_obstacle(row, col)

    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """Return the 4-directional free neighbours of (row, col)."""
        return [
            (row + dr, col + dc)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]
            if self.is_free(row + dr, col + dc)
        ]

    def all_free_cells(self) -> List[Tuple[int, int]]:
        return [
            (r, c)
            for r in range(self.rows)
            for c in range(self.cols)
            if self.is_free(r, c)
        ]


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: AStar  (Space-Time A*)
# ─────────────────────────────────────────────────────────────────────────────

class AStar:
    """
    A* Search Algorithm with optional Space-Time reservation table.

    Normal mode  : finds the shortest path ignoring other robots.
    Space-Time   : avoids cells already reserved by higher-priority robots
                   at specific time steps, preventing collisions.
    """

    @staticmethod
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """Manhattan distance — admissible heuristic for grid movement."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def search(
        grid: Grid,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        reserved: Dict[Tuple[int, int, int], bool] = None,
        max_time: int = 300,
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Space-Time A* search.

        Args:
            grid     : the warehouse grid
            start    : (row, col) start position
            goal     : (row, col) goal position
            reserved : {(row, col, t): True} — cells already taken at time t
            max_time : upper bound on time steps to prevent infinite loops

        Returns:
            List of (row, col) positions from start to goal, or None.
        """
        if reserved is None:
            reserved = {}

        # Initial state: (row, col, time=0)
        h0 = AStar.heuristic(start, goal)
        # Heap entries: (f, g, row, col, time, path)
        open_heap = [(h0, 0, start[0], start[1], 0, [start])]
        visited: Dict[Tuple[int, int, int], int] = {}  # state -> best g

        while open_heap:
            f, g, row, col, t, path = heapq.heappop(open_heap)

            state = (row, col, t)
            if state in visited and visited[state] <= g:
                continue
            visited[state] = g

            # ── Goal check ────────────────────────────────────────────────
            if (row, col) == goal:
                return path

            if t >= max_time:
                continue

            # ── Expand neighbours (4 moves + wait-in-place) ───────────────
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]:
                nr, nc, nt = row + dr, col + dc, t + 1

                if not grid.is_free(nr, nc):
                    continue

                # Space-time collision: another robot is here at time nt
                if (nr, nc, nt) in reserved:
                    continue

                # Swap collision: two robots passing through each other
                if (nr, nc, t) in reserved and (row, col, nt) in reserved:
                    continue

                new_state = (nr, nc, nt)
                new_g = g + 1
                if new_state in visited and visited[new_state] <= new_g:
                    continue

                new_f = new_g + AStar.heuristic((nr, nc), goal)
                heapq.heappush(
                    open_heap,
                    (new_f, new_g, nr, nc, nt, path + [(nr, nc)]),
                )

        return None  # No path found


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: BFS  (for comparison)
# ─────────────────────────────────────────────────────────────────────────────

class BFS:
    """
    Breadth-First Search — guarantees shortest path in terms of step count
    but does NOT account for other robots (no space-time awareness).
    Used to benchmark A* path length.
    """

    @staticmethod
    def search(
        grid: Grid,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        """Standard BFS without space-time constraints."""
        if start == goal:
            return [start]

        queue: deque = deque([(start, [start])])
        visited: Set[Tuple[int, int]] = {start}

        while queue:
            (row, col), path = queue.popleft()

            for nr, nc in grid.get_neighbors(row, col):
                if (nr, nc) not in visited:
                    new_path = path + [(nr, nc)]
                    if (nr, nc) == goal:
                        return new_path
                    visited.add((nr, nc))
                    queue.append(((nr, nc), new_path))

        return None  # No path found


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: Robot
# ─────────────────────────────────────────────────────────────────────────────

class Robot:
    """
    Represents one warehouse robot.

    Attributes:
        robot_id    : unique integer ID
        start       : (row, col) starting position
        goal        : (row, col) target position
        path        : list of (row, col) waypoints from A* (incl. waits)
        bfs_path    : list of (row, col) waypoints from BFS (no waits)
        current_pos : current position during simulation
        step_index  : current index in path[]
        reached_goal: True when robot has arrived at goal
        wait_count  : number of wait steps inserted for conflict avoidance
        color       : display colour (hex string)
    """

    def __init__(self, robot_id: int, start: Tuple[int, int], goal: Tuple[int, int]):
        self.robot_id   = robot_id
        self.start      = start
        self.goal       = goal
        self.path:      List[Tuple[int, int]] = [start]
        self.bfs_path:  List[Tuple[int, int]] = [start]
        self.current_pos = start
        self.step_index  = 0
        self.reached_goal = False
        self.wait_count  = 0
        self.color       = ROBOT_COLORS[robot_id % len(ROBOT_COLORS)]
        self.path_cost   = 0   # A* cost (steps incl. waits)
        self.bfs_cost    = 0   # BFS cost (pure steps)

    # ── Simulation control ─────────────────────────────────────────────────

    def advance(self):
        """Move one step along the planned path."""
        if self.step_index < len(self.path) - 1:
            prev = self.current_pos
            self.step_index += 1
            self.current_pos = self.path[self.step_index]
            if self.current_pos == prev:
                self.wait_count += 1  # counted as a wait step
        if self.current_pos == self.goal:
            self.reached_goal = True

    def reset(self):
        """Rewind to start for re-simulation."""
        self.current_pos  = self.start
        self.step_index   = 0
        self.reached_goal = False
        self.wait_count   = 0

    # ── Properties ─────────────────────────────────────────────────────────

    @property
    def status_text(self) -> str:
        if self.reached_goal:
            return "✓ Goal reached"
        return f"Step {self.step_index} / {len(self.path) - 1}"

    @property
    def move_steps(self) -> int:
        """Actual movement steps (path cost minus waits)."""
        return self.path_cost - self.wait_count


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: MultiRobotCoordinator
# ─────────────────────────────────────────────────────────────────────────────

class MultiRobotCoordinator:
    """
    Cooperative Pathfinding using Priority-Based Space-Time A*.

    Algorithm:
      1. Sort robots by priority (ID order, lowest first).
      2. For each robot, run Space-Time A* while avoiding cells already
         reserved by higher-priority robots.
      3. After a robot's path is found, add its (row, col, t) triples to
         the global reservation table.
      4. Also run standard BFS for each robot (no constraints) to compare
         path lengths.
    """

    def __init__(self, grid: Grid):
        self.grid = grid

    def plan_all(self, robots: List[Robot]) -> Dict[int, bool]:
        """
        Plan collision-free paths for all robots.
        Returns {robot_id: True/False} indicating success.
        """
        # Global space-time reservation table
        reserved: Dict[Tuple[int, int, int], bool] = {}
        results: Dict[int, bool] = {}

        for robot in robots:
            # ── Space-Time A* ──────────────────────────────────────────────
            path = AStar.search(self.grid, robot.start, robot.goal, reserved)

            if path is None:
                print(f"  [Robot {robot.robot_id}] WARNING: No path found!")
                robot.path      = [robot.start]
                robot.path_cost = 0
                results[robot.robot_id] = False
            else:
                robot.path      = path
                robot.path_cost = len(path) - 1
                results[robot.robot_id] = True

                # Reserve this robot's space-time cells
                for t, (r, c) in enumerate(path):
                    reserved[(r, c, t)] = True

                # Hold the goal position after arrival (prevents others parking there)
                gr, gc = path[-1]
                for extra_t in range(len(path), len(path) + 60):
                    reserved[(gr, gc, extra_t)] = True

            # ── BFS (no constraints — for comparison) ──────────────────────
            bfs = BFS.search(self.grid, robot.start, robot.goal)
            if bfs:
                robot.bfs_path  = bfs
                robot.bfs_cost  = len(bfs) - 1

        return results

    def detect_conflicts(self, robots: List[Robot]) -> List[str]:
        """
        Verify the final paths for any residual conflicts.
        Returns a list of conflict description strings.
        """
        conflicts = []
        if not robots:
            return conflicts

        max_t = max(len(r.path) for r in robots)

        for t in range(max_t):
            occupancy: Dict[Tuple[int, int], int] = {}
            for robot in robots:
                idx = min(t, len(robot.path) - 1)
                pos = robot.path[idx]
                if pos in occupancy:
                    conflicts.append(
                        f"t={t}: Robot {robot.robot_id} & "
                        f"Robot {occupancy[pos]} both at {pos}"
                    )
                else:
                    occupancy[pos] = robot.robot_id

        return conflicts


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: Simulation
# ─────────────────────────────────────────────────────────────────────────────

class Simulation:
    """
    Controls the step-by-step movement simulation.

    Responsibilities:
      • Trigger path planning via MultiRobotCoordinator
      • Advance all robots one step at a time
      • Track global time step and completion status
      • Print a summary of results to the console
    """

    def __init__(self, grid: Grid, robots: List[Robot]):
        self.grid        = grid
        self.robots      = robots
        self.coordinator = MultiRobotCoordinator(grid)
        self.current_step = 0
        self.max_steps    = 0
        self.running      = False
        self.planned      = False
        self.conflicts:  List[str] = []
        self.plan_results: Dict[int, bool] = {}

    def plan(self) -> Dict[int, bool]:
        """Run the cooperative planner and populate robot paths."""
        self.plan_results = self.coordinator.plan_all(self.robots)
        self.conflicts    = self.coordinator.detect_conflicts(self.robots)
        self.max_steps    = max((len(r.path) for r in self.robots), default=1)
        self.planned      = True

        # ── Console output ─────────────────────────────────────────────────
        print("\n" + "═" * 62)
        print("   AI MULTI-ROBOT WAREHOUSE NAVIGATION — PATH PLANNING")
        print("═" * 62)
        for robot in self.robots:
            ok = self.plan_results.get(robot.robot_id, False)
            tag = "OK    " if ok else "FAILED"
            print(f"\n  Robot {robot.robot_id}  [{tag}]  colour: {robot.color}")
            print(f"    Start  : {robot.start}")
            print(f"    Goal   : {robot.goal}")
            print(f"    A* cost: {robot.path_cost} steps  "
                  f"(waits: {robot.path_cost - robot.move_steps})")
            print(f"    BFS cost: {robot.bfs_cost} steps  "
                  f"(overhead: +{max(0, robot.path_cost - robot.bfs_cost)})")
            snippet = " → ".join(str(p) for p in robot.path[:6])
            if len(robot.path) > 6:
                snippet += " → ..."
            print(f"    Path   : {snippet}")

        print()
        if self.conflicts:
            print(f"  ⚠  {len(self.conflicts)} conflict(s) detected:")
            for c in self.conflicts[:5]:
                print(f"     {c}")
        else:
            print("  ✓  Zero conflicts — all paths are collision-free!")
        print("═" * 62 + "\n")

        return self.plan_results

    def step(self):
        """Advance every robot one time step."""
        if not self.planned:
            return
        for robot in self.robots:
            robot.advance()
        self.current_step += 1

    def reset(self):
        """Rewind all robots to their starting positions."""
        for robot in self.robots:
            robot.reset()
        self.current_step = 0
        self.running      = False

    @property
    def all_done(self) -> bool:
        return bool(self.robots) and all(r.reached_goal for r in self.robots)


# ─────────────────────────────────────────────────────────────────────────────
# CLASS: WarehouseGUI
# ─────────────────────────────────────────────────────────────────────────────

class WarehouseGUI:
    """
    Tkinter GUI for the warehouse navigation simulation.

    Layout:
      Left panel  — controls, settings, robot status cards
      Right panel — grid canvas + legend bar
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AI Multi-Robot Warehouse Navigation System")
        self.root.configure(bg=C_BG)

        # State
        self.grid:       Optional[Grid]       = None
        self.robots:     List[Robot]          = []
        self.simulation: Optional[Simulation] = None
        self._anim_id:   Optional[str]        = None
        self._cs  = 40   # cell size in pixels (computed)
        self._ox  = 0    # canvas x offset
        self._oy  = 0    # canvas y offset

        # Tkinter variables
        self.var_rows   = tk.IntVar(value=12)
        self.var_cols   = tk.IntVar(value=15)
        self.var_robots = tk.IntVar(value=3)
        self.var_speed  = tk.IntVar(value=450)
        self.var_paths  = tk.BooleanVar(value=True)
        self.var_bfs    = tk.BooleanVar(value=False)

        self._build_ui()
        self.root.after(80, lambda: [self._new_setup(), self._draw()])

    # ════════════════════════════════════════════════════════════════════════
    # UI CONSTRUCTION
    # ════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        """Assemble all panels and widgets."""
        # ── Left control panel ────────────────────────────────────────────
        self.left = tk.Frame(self.root, bg=C_PANEL, width=270)
        self.left.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 4), pady=10)
        self.left.pack_propagate(False)

        self._build_header()
        self._build_grid_settings()
        self._build_buttons()
        self._build_speed_control()
        self._build_display_options()
        self._build_robot_status()

        # ── Right canvas panel ────────────────────────────────────────────
        right = tk.Frame(self.root, bg=C_BG)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 10), pady=10)

        self.canvas = tk.Canvas(right, bg=C_BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda e: self._draw())

        self._build_legend(right)

    def _sep(self):
        """Thin separator line."""
        ttk.Separator(self.left, orient="horizontal").pack(
            fill=tk.X, padx=12, pady=6
        )

    def _label(self, text, size=10, color=C_ACCENT, bold=True):
        weight = "bold" if bold else "normal"
        tk.Label(
            self.left, text=text, bg=C_PANEL, fg=color,
            font=("Trebuchet MS", size, weight),
        ).pack(anchor="w", padx=14, pady=(6, 2))

    def _build_header(self):
        tk.Label(
            self.left, text="🤖  Warehouse AI",
            bg=C_PANEL, fg=C_ACCENT,
            font=("Trebuchet MS", 15, "bold"),
        ).pack(pady=(14, 2))
        tk.Label(
            self.left, text="Multi-Robot Navigation System",
            bg=C_PANEL, fg=C_TEXT,
            font=("Georgia", 9, "italic"),
        ).pack(pady=(0, 10))
        self._sep()

    def _build_grid_settings(self):
        self._label("GRID SETTINGS")
        frm = tk.Frame(self.left, bg=C_PANEL)
        frm.pack(fill=tk.X, padx=14, pady=4)

        params = [
            ("Rows",   self.var_rows,   6, 22),
            ("Cols",   self.var_cols,   6, 26),
            ("Robots", self.var_robots, 1,  6),
        ]
        for i, (lbl, var, lo, hi) in enumerate(params):
            tk.Label(frm, text=f"{lbl}:", bg=C_PANEL, fg=C_TEXT,
                     font=("Calibri", 10)).grid(row=i, column=0, sticky="w", pady=3)
            sb = tk.Spinbox(
                frm, from_=lo, to=hi, textvariable=var, width=5,
                bg=C_CARD, fg="white", buttonbackground=C_PANEL,
                font=("Calibri", 10), relief="flat",
                highlightbackground=C_ACCENT, highlightthickness=1,
            )
            sb.grid(row=i, column=1, padx=8, pady=3, sticky="w")
        self._sep()

    def _btn(self, text, cmd, bg=C_PANEL, fg=C_ACCENT, pady=7):
        """Styled button helper."""
        b = tk.Button(
            self.left, text=text, command=cmd,
            bg=bg, fg=fg, activebackground=C_CARD, activeforeground=fg,
            font=("Trebuchet MS", 10, "bold"),
            relief="flat", cursor="hand2", pady=pady,
            highlightthickness=1, highlightbackground=C_ACCENT,
        )
        b.pack(fill=tk.X, padx=14, pady=3)
        return b

    def _build_buttons(self):
        self._label("CONTROLS")
        self._btn("⟳  New Random Setup",  self._new_setup,   bg=C_CARD)
        self._btn("▶  Plan Paths (A*)",   self._plan,        bg=C_GREEN,   fg=C_BG)

        frm = tk.Frame(self.left, bg=C_PANEL)
        frm.pack(fill=tk.X, padx=14, pady=3)
        for text, cmd, color in [
            ("▶▶ Run",  self._run,       C_ACCENT),
            ("⏸",       self._pause,     C_CARD),
            ("▶|",      self._step_once, C_CARD),
        ]:
            tk.Button(
                frm, text=text, command=cmd,
                bg=color, fg=C_BG if color == C_ACCENT else C_ACCENT,
                font=("Trebuchet MS", 10, "bold"),
                relief="flat", cursor="hand2", pady=7,
                highlightthickness=1, highlightbackground=C_ACCENT,
            ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self._btn("↺  Reset",    self._reset,  bg=C_RED,  fg="white")
        self._sep()

    def _build_speed_control(self):
        self._label("ANIMATION SPEED")
        tk.Scale(
            self.left, from_=80, to=1400, orient=tk.HORIZONTAL,
            variable=self.var_speed, bg=C_PANEL, fg=C_TEXT,
            troughcolor=C_CARD, activebackground=C_ACCENT,
            highlightthickness=0, showvalue=False,
        ).pack(fill=tk.X, padx=14)
        tk.Label(
            self.left, text="← faster         slower →",
            bg=C_PANEL, fg=C_MUTED, font=("Calibri", 8),
        ).pack()
        self._sep()

    def _build_display_options(self):
        self._label("DISPLAY OPTIONS")
        for text, var in [
            ("Show A* paths",  self.var_paths),
            ("Show BFS paths (dashed)", self.var_bfs),
        ]:
            tk.Checkbutton(
                self.left, text=text, variable=var,
                bg=C_PANEL, fg=C_TEXT, selectcolor=C_CARD,
                activebackground=C_PANEL, activeforeground=C_TEXT,
                font=("Calibri", 10), command=self._draw,
            ).pack(anchor="w", padx=20, pady=2)
        self._sep()

    def _build_robot_status(self):
        self._label("ROBOT STATUS")
        self.robot_status_frame = tk.Frame(self.left, bg=C_PANEL)
        self.robot_status_frame.pack(fill=tk.X, padx=10)

        # Step counter
        self.lbl_step = tk.Label(
            self.left, text="Step: 0 / —",
            bg=C_PANEL, fg=C_GREEN, font=("Arial Black", 12),
        )
        self.lbl_step.pack(pady=(10, 3))

        self.lbl_status = tk.Label(
            self.left, text="Ready — click New Setup",
            bg=C_PANEL, fg=C_TEXT, font=("Georgia", 9, "italic"),
        )
        self.lbl_status.pack(pady=(0, 10))

    def _build_legend(self, parent):
        """Bottom legend bar."""
        bar = tk.Frame(parent, bg=C_PANEL, height=36)
        bar.pack(fill=tk.X, pady=(6, 0))

        items = [
            (C_GRID_WALL, "Shelf/Wall"),
            (C_GRID_FREE, "Free cell"),
            ("#FFD700",   "Goal"),
            (None,        "Robot (circle)"),
            (None,        "A* path →"),
            (None,        "BFS path (dashed)"),
        ]
        for color, label in items:
            f = tk.Frame(bar, bg=C_PANEL)
            f.pack(side=tk.LEFT, padx=10, pady=6)
            if color:
                cv = tk.Canvas(f, width=14, height=14, bg=C_PANEL,
                               highlightthickness=0)
                cv.pack(side=tk.LEFT)
                cv.create_rectangle(1, 1, 13, 13, fill=color, outline=C_ACCENT)
            tk.Label(f, text=label, bg=C_PANEL, fg=C_TEXT,
                     font=("Calibri", 9)).pack(side=tk.LEFT, padx=3)

    # ════════════════════════════════════════════════════════════════════════
    # SETUP & PLANNING
    # ════════════════════════════════════════════════════════════════════════

    def _new_setup(self):
        """Create a fresh grid with warehouse obstacles and random robots."""
        self._cancel_anim()

        rows     = self.var_rows.get()
        cols     = self.var_cols.get()
        n_robots = self.var_robots.get()

        self.grid = Grid(rows, cols)
        self._add_warehouse_layout()

        self.robots     = []
        self.simulation = None
        self._place_robots(n_robots)

        self.simulation = Simulation(self.grid, self.robots)
        self._refresh_robot_cards()
        self._draw()
        self.lbl_step.config(text="Step: 0 / —")
        self.lbl_status.config(text="Click '▶ Plan Paths' to start")

    def _add_warehouse_layout(self):
        """
        Build a realistic warehouse shelf layout:
        horizontal shelf rows with aisle gaps every 4 cells.
        """
        rows = self.grid.rows
        cols = self.grid.cols

        # Shelf rows spaced every 3 rows, leaving top/bottom border clear
        for shelf_r in range(2, rows - 2, 3):
            for c in range(1, cols - 1):
                if c % 4 != 0:   # gap at every 4th column = aisle
                    self.grid.add_obstacle(shelf_r, c)

        # A couple of random scattered boxes
        rng = random.Random(7)
        for _ in range(max(2, (rows * cols) // 35)):
            r = rng.randint(1, rows - 2)
            c = rng.randint(1, cols - 2)
            # Don't block corners (common robot zones)
            if r not in (0, 1, rows - 2, rows - 1):
                self.grid.add_obstacle(r, c)

    def _place_robots(self, n: int):
        """
        Assign start and goal positions for n robots.
        Ensures:
          • All positions are on free cells
          • Start ≠ Goal
          • Manhattan distance ≥ 6 between start and goal
          • No two robots share a start or goal
        """
        free = self.grid.all_free_cells()
        rng  = random.Random(42)
        rng.shuffle(free)

        used: Set[Tuple[int, int]] = set()
        robot_id = 0

        for start in free:
            if robot_id >= n:
                break
            if start in used:
                continue

            # Find a suitable goal
            goal = None
            best_d = 0
            for candidate in free:
                if candidate in used or candidate == start:
                    continue
                d = abs(candidate[0] - start[0]) + abs(candidate[1] - start[1])
                if d >= 6 and d > best_d:
                    best_d = d
                    goal   = candidate

            if goal is None:
                continue

            used.add(start)
            used.add(goal)
            self.robots.append(Robot(robot_id, start, goal))
            robot_id += 1

    def _plan(self):
        """Plan paths for all robots using cooperative A*."""
        if not self.simulation:
            return
        self._cancel_anim()
        self.lbl_status.config(text="Planning…")
        self.root.update_idletasks()

        self.simulation.plan()

        self._refresh_robot_cards()
        self._draw()

        ms = self.simulation.max_steps - 1
        self.lbl_step.config(text=f"Step: 0 / {ms}")

        if self.simulation.conflicts:
            n = len(self.simulation.conflicts)
            self.lbl_status.config(text=f"⚠ {n} conflict(s) found")
            messagebox.showwarning(
                "Conflicts",
                f"{n} conflict(s) detected.\nSee console for details.",
            )
        else:
            self.lbl_status.config(text="✓ Paths ready — click ▶▶ Run")

    # ════════════════════════════════════════════════════════════════════════
    # SIMULATION CONTROL
    # ════════════════════════════════════════════════════════════════════════

    def _run(self):
        """Start or resume animation."""
        if not self.simulation or not self.simulation.planned:
            messagebox.showinfo("Plan first", "Click '▶ Plan Paths' before running.")
            return
        if self.simulation.all_done:
            self._reset()
        self.simulation.running = True
        self.lbl_status.config(text="Simulating…")
        self._tick()

    def _tick(self):
        """One animation frame."""
        if not self.simulation or not self.simulation.running:
            return
        if self.simulation.all_done:
            self.simulation.running = False
            self.lbl_status.config(text="✓ All robots reached their goals!")
            self._draw()
            return

        self.simulation.step()
        self._draw()
        ms = self.simulation.max_steps - 1
        cs = self.simulation.current_step
        self.lbl_step.config(text=f"Step: {cs} / {ms}")
        self._refresh_robot_cards()

        delay = self.var_speed.get()
        self._anim_id = self.root.after(delay, self._tick)

    def _pause(self):
        self._cancel_anim()
        if self.simulation:
            self.simulation.running = False
        self.lbl_status.config(text="Paused — click ▶▶ Run to continue")

    def _step_once(self):
        """Advance exactly one step and pause."""
        if not self.simulation or not self.simulation.planned:
            return
        self._cancel_anim()
        self.simulation.running = False
        self.simulation.step()
        self._draw()
        self._refresh_robot_cards()
        ms = self.simulation.max_steps - 1
        cs = self.simulation.current_step
        self.lbl_step.config(text=f"Step: {cs} / {ms}")

    def _reset(self):
        """Rewind robots to start."""
        self._cancel_anim()
        if self.simulation:
            self.simulation.reset()
        self._draw()
        self._refresh_robot_cards()
        ms = (self.simulation.max_steps - 1) if self.simulation else 0
        self.lbl_step.config(text=f"Step: 0 / {ms}")
        self.lbl_status.config(text="Reset — click ▶▶ Run")

    def _cancel_anim(self):
        if self._anim_id:
            self.root.after_cancel(self._anim_id)
            self._anim_id = None

    # ════════════════════════════════════════════════════════════════════════
    # DRAWING
    # ════════════════════════════════════════════════════════════════════════

    def _draw(self):
        """Full canvas redraw."""
        if not self.grid:
            return

        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 20 or h < 20:
            return

        rows = self.grid.rows
        cols = self.grid.cols

        # Compute cell size so grid fits the canvas with padding
        cs = min((w - 30) // cols, (h - 30) // rows)
        cs = max(cs, 16)
        self._cs = cs

        # Centre the grid on the canvas
        ox = (w - cs * cols) // 2
        oy = (h - cs * rows) // 2
        self._ox = ox
        self._oy = oy

        self._draw_cells(cs, ox, oy)

        planned = self.simulation and self.simulation.planned

        if self.var_bfs.get() and planned:
            for robot in self.robots:
                self._draw_path(robot.bfs_path, robot.color, cs, ox, oy, dashed=True)

        if self.var_paths.get() and planned:
            for robot in self.robots:
                self._draw_path(robot.path, robot.color, cs, ox, oy, dashed=False)

        for robot in self.robots:
            self._draw_goal(robot, cs, ox, oy)
            self._draw_start_marker(robot, cs, ox, oy)

        for robot in self.robots:
            self._draw_robot(robot, cs, ox, oy)

        if planned:
            self._draw_step_badge(w, h)

    def _draw_cells(self, cs, ox, oy):
        """Draw all grid cells (free and obstacle)."""
        for r in range(self.grid.rows):
            for c in range(self.grid.cols):
                x1 = ox + c * cs
                y1 = oy + r * cs
                x2 = x1 + cs
                y2 = y1 + cs

                if self.grid.is_obstacle(r, c):
                    # Shelf / wall cell
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2,
                        fill=C_GRID_WALL, outline="#1C3A50", width=1,
                    )
                    # Shelf lines (decorative)
                    if cs >= 20:
                        for frac in [0.33, 0.66]:
                            yy = int(y1 + frac * cs)
                            self.canvas.create_line(
                                x1 + 3, yy, x2 - 3, yy,
                                fill="#3A5060", width=1,
                            )
                else:
                    # Free cell — subtle grid lines
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2,
                        fill=C_GRID_FREE, outline="#162840", width=1,
                    )

    def _draw_path(self, path, color, cs, ox, oy, dashed=False):
        """Draw a robot's path as a coloured line on the canvas."""
        if len(path) < 2:
            return
        pts = []
        for r, c in path:
            pts.extend([ox + c * cs + cs // 2, oy + r * cs + cs // 2])

        kwargs = dict(fill=color, smooth=True)
        if dashed:
            kwargs.update(width=1, dash=(5, 4))
        else:
            kwargs.update(
                width=max(2, cs // 14),
                arrow=tk.LAST,
                arrowshape=(max(6, cs // 6), max(8, cs // 5), max(3, cs // 12)),
            )
        self.canvas.create_line(*pts, **kwargs)

    def _draw_goal(self, robot: Robot, cs, ox, oy):
        """Draw a dashed goal-marker square at the robot's goal cell."""
        r, c = robot.goal
        pad  = max(3, cs // 8)
        x1   = ox + c * cs + pad
        y1   = oy + r * cs + pad
        x2   = ox + (c + 1) * cs - pad
        y2   = oy + (r + 1) * cs - pad

        # Tinted background
        self.canvas.create_rectangle(
            ox + c * cs, oy + r * cs,
            ox + (c + 1) * cs, oy + (r + 1) * cs,
            fill=C_GOAL_BG, outline="", width=0,
        )
        # Dashed border
        self.canvas.create_rectangle(
            x1, y1, x2, y2,
            fill="", outline=robot.color, width=2, dash=(4, 3),
        )
        # "G" label
        if cs >= 24:
            self.canvas.create_text(
                (x1 + x2) // 2, (y1 + y2) // 2,
                text="G", fill=robot.color,
                font=("Arial Black", max(7, cs // 5)),
            )

    def _draw_start_marker(self, robot: Robot, cs, ox, oy):
        """Draw a small 'S' at the starting cell."""
        if robot.current_pos == robot.start or not self.simulation:
            return
        r, c = robot.start
        cx   = ox + c * cs + cs // 2
        cy   = oy + r * cs + cs // 2
        if cs >= 28:
            self.canvas.create_text(
                cx, cy, text="S",
                fill=robot.color, font=("Arial Black", max(7, cs // 6)),
            )

    def _draw_robot(self, robot: Robot, cs, ox, oy):
        """Draw the robot as a coloured circle with its ID."""
        r, c = robot.current_pos
        cx   = ox + c * cs + cs // 2
        cy   = oy + r * cs + cs // 2
        rad  = max(8, cs // 2 - 4)

        # Outer glow ring
        self.canvas.create_oval(
            cx - rad - 3, cy - rad - 3,
            cx + rad + 3, cy + rad + 3,
            fill="", outline=robot.color, width=1,
        )
        # Body fill
        self.canvas.create_oval(
            cx - rad, cy - rad,
            cx + rad, cy + rad,
            fill=robot.color, outline="white", width=2,
        )
        # Robot ID text
        self.canvas.create_text(
            cx, cy, text=str(robot.robot_id),
            fill=C_BG, font=("Arial Black", max(8, cs // 3)),
        )
        # "Done" checkmark
        if robot.reached_goal and cs >= 22:
            self.canvas.create_text(
                cx, cy - rad - 6, text="✓",
                fill=C_GREEN, font=("Arial Black", max(7, cs // 4)),
            )

    def _draw_step_badge(self, w, h):
        """Small step counter overlay in top-right corner of canvas."""
        if not self.simulation:
            return
        txt = f"t = {self.simulation.current_step}"
        self.canvas.create_rectangle(
            w - 90, 8, w - 8, 32,
            fill=C_PANEL, outline=C_ACCENT, width=1,
        )
        self.canvas.create_text(
            w - 49, 20, text=txt,
            fill=C_ACCENT, font=("Arial Black", 10),
        )

    # ════════════════════════════════════════════════════════════════════════
    # ROBOT STATUS CARDS
    # ════════════════════════════════════════════════════════════════════════

    def _refresh_robot_cards(self):
        """Rebuild the robot status card list in the left panel."""
        for w in self.robot_status_frame.winfo_children():
            w.destroy()

        planned = self.simulation and self.simulation.planned

        for robot in self.robots:
            card = tk.Frame(self.robot_status_frame, bg=C_CARD, pady=5)
            card.pack(fill=tk.X, pady=3)

            # Colour dot
            dot_cv = tk.Canvas(card, width=14, height=14, bg=C_CARD,
                               highlightthickness=0)
            dot_cv.pack(side=tk.LEFT, padx=(8, 4))
            dot_cv.create_oval(1, 1, 13, 13, fill=robot.color, outline="")

            # ID + status
            right_f = tk.Frame(card, bg=C_CARD)
            right_f.pack(side=tk.LEFT, fill=tk.X, expand=True)

            header_txt = f"Robot {robot.robot_id}"
            if robot.reached_goal:
                header_txt += "  ✓"
            tk.Label(
                right_f, text=header_txt,
                bg=C_CARD, fg="white",
                font=("Trebuchet MS", 9, "bold"),
            ).pack(anchor="w")

            if planned:
                info = (
                    f"A*: {robot.path_cost} steps  |  "
                    f"BFS: {robot.bfs_cost} steps  |  "
                    f"wait: {robot.path_cost - robot.move_steps}"
                )
                tk.Label(
                    right_f, text=info,
                    bg=C_CARD, fg=C_TEXT,
                    font=("Calibri", 8),
                ).pack(anchor="w")
                tk.Label(
                    right_f, text=robot.status_text,
                    bg=C_CARD, fg=C_MUTED,
                    font=("Consolas", 8),
                ).pack(anchor="w")
            else:
                tk.Label(
                    right_f,
                    text=f"{robot.start} → {robot.goal}",
                    bg=C_CARD, fg=C_MUTED,
                    font=("Consolas", 8),
                ).pack(anchor="w")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    root.geometry("1280x760")
    root.minsize(900, 600)

    # Apply a basic ttk theme
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=C_PANEL)

    app = WarehouseGUI(root)   # noqa: F841
    root.mainloop()


if __name__ == "__main__":
    main()
