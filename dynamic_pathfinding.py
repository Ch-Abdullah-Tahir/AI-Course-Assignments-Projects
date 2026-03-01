"""
=============================================================
  DYNAMIC PATHFINDING AGENT  —  Question 6
  Algorithms : Greedy Best-First Search  |  A* Search
  Heuristics : Manhattan Distance        |  Euclidean Distance
  GUI        : Tkinter (stdlib – no extra installs needed)
=============================================================
HOW TO RUN
----------
  python dynamic_pathfinding.py

CONTROLS (in the GUI)
---------------------
  • Set Rows / Cols / Density → click "Generate Map"
  • Left-click  a cell  → toggle wall
  • Right-click a cell  → cycle: Start → Goal → Empty
  • Choose Algorithm & Heuristic from the dropdowns
  • "Run Search"        → visualise one search
  • "Dynamic Mode" checkbox → obstacles spawn while agent moves
  • "Reset Visited"     → clear colours, keep walls
  • "Clear All"         → wipe everything
=============================================================
"""

import tkinter as tk
from tkinter import ttk, messagebox
import heapq, math, random, time
from collections import defaultdict

# ─────────────────────────────────────────────
#  COLOUR PALETTE
# ─────────────────────────────────────────────
C = {
    "bg":       "#0f0f1a",
    "panel":    "#1a1a2e",
    "accent":   "#e94560",
    "accent2":  "#0f3460",
    "text":     "#eaeaea",
    "empty":    "#16213e",
    "wall":     "#2d2d44",
    "start":    "#00d4aa",
    "goal":     "#f5a623",
    "frontier": "#f0e14a",   # yellow  – in priority queue
    "visited":  "#3a6ea5",   # blue    – expanded
    "path":     "#00e676",   # green   – final path
    "agent":    "#e94560",   # red dot – current agent pos
    "grid":     "#1e1e35",
}

# ─────────────────────────────────────────────
#  HEURISTIC FUNCTIONS
# ─────────────────────────────────────────────
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

HEURISTICS = {"Manhattan": manhattan, "Euclidean": euclidean}

# ─────────────────────────────────────────────
#  SEARCH ALGORITHMS
# ─────────────────────────────────────────────
def get_neighbors(pos, rows, cols, walls):
    r, c = pos
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols and (nr,nc) not in walls:
            yield (nr, nc)

def gbfs(start, goal, rows, cols, walls, heuristic, step_cb=None):
    """Greedy Best-First Search  f(n) = h(n)"""
    h = heuristic
    heap = [(h(start, goal), start)]
    came_from = {start: None}
    visited_order = []
    frontier_set = {start}

    while heap:
        _, curr = heapq.heappop(heap)
        frontier_set.discard(curr)

        if curr == goal:
            return _reconstruct(came_from, goal), visited_order, set(frontier_set)

        visited_order.append(curr)
        if step_cb:
            step_cb(curr, "visited", set(frontier_set))

        for nb in get_neighbors(curr, rows, cols, walls):
            if nb not in came_from:
                came_from[nb] = curr
                heapq.heappush(heap, (h(nb, goal), nb))
                frontier_set.add(nb)
                if step_cb:
                    step_cb(nb, "frontier", set(frontier_set))

    return None, visited_order, set()

def astar(start, goal, rows, cols, walls, heuristic, step_cb=None):
    """A* Search  f(n) = g(n) + h(n)"""
    h = heuristic
    g = defaultdict(lambda: float('inf'))
    g[start] = 0
    heap = [(h(start, goal), 0, start)]
    came_from = {start: None}
    closed = set()
    visited_order = []
    frontier_set = {start}

    while heap:
        f, cost, curr = heapq.heappop(heap)
        if curr in closed:
            continue
        closed.add(curr)
        frontier_set.discard(curr)

        if curr == goal:
            return _reconstruct(came_from, goal), visited_order, set(frontier_set)

        visited_order.append(curr)
        if step_cb:
            step_cb(curr, "visited", set(frontier_set))

        for nb in get_neighbors(curr, rows, cols, walls):
            new_g = g[curr] + 1
            if new_g < g[nb]:
                g[nb] = new_g
                came_from[nb] = curr
                heapq.heappush(heap, (new_g + h(nb, goal), new_g, nb))
                frontier_set.add(nb)
                if step_cb:
                    step_cb(nb, "frontier", set(frontier_set))

    return None, visited_order, set()

def _reconstruct(came_from, goal):
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path

ALGORITHMS = {"A* Search": astar, "Greedy BFS": gbfs}

# ─────────────────────────────────────────────
#  MAIN APPLICATION
# ─────────────────────────────────────────────
class PathfindingApp(tk.Tk):
    CELL = 36          # px per cell (auto-scales)
    MIN_CELL = 14
    MAX_CELL = 60

    def __init__(self):
        super().__init__()
        self.title("Dynamic Pathfinding Agent")
        self.configure(bg=C["bg"])
        self.resizable(True, True)

        # ── state ──────────────────────────────
        self.rows = 15
        self.cols = 20
        self.walls = set()
        self.start = (0, 0)
        self.goal  = (self.rows-1, self.cols-1)
        self.path  = []
        self.visited_cells = set()
        self.frontier_cells = set()
        self.agent_pos = None
        self.dynamic_running = False
        self._anim_id = None
        self._right_click_mode = "start"   # cycles start→goal

        # ── metrics ────────────────────────────
        self.nodes_visited  = tk.IntVar(value=0)
        self.path_cost      = tk.IntVar(value=0)
        self.exec_time_ms   = tk.DoubleVar(value=0.0)
        self.replans        = tk.IntVar(value=0)

        self._build_ui()
        self._generate_map()

    # ══════════════════════════════════════════
    #  UI CONSTRUCTION
    # ══════════════════════════════════════════
    def _build_ui(self):
        # ── left panel ─────────────────────────
        panel = tk.Frame(self, bg=C["panel"], padx=12, pady=12)
        panel.pack(side=tk.LEFT, fill=tk.Y)

        def lbl(parent, text, **kw):
            tk.Label(parent, text=text, bg=C["panel"], fg=C["text"],
                     font=("Consolas", 10), **kw).pack(anchor="w", pady=(6,0))

        def entry_row(parent, label, var, width=5):
            f = tk.Frame(parent, bg=C["panel"])
            f.pack(fill=tk.X, pady=2)
            tk.Label(f, text=label, bg=C["panel"], fg=C["text"],
                     font=("Consolas",10), width=14, anchor="w").pack(side=tk.LEFT)
            e = tk.Entry(f, textvariable=var, width=width,
                         bg=C["accent2"], fg=C["text"],
                         insertbackground=C["text"], font=("Consolas",10))
            e.pack(side=tk.LEFT)
            return e

        # title
        tk.Label(panel, text="⬡ PATHFINDER", bg=C["panel"], fg=C["accent"],
                 font=("Consolas", 16, "bold")).pack(pady=(0,8))

        # separator helper
        def sep():
            tk.Frame(panel, bg=C["accent2"], height=1).pack(fill=tk.X, pady=6)

        # grid settings
        lbl(panel, "GRID SETTINGS")
        self._rows_var = tk.StringVar(value=str(self.rows))
        self._cols_var = tk.StringVar(value=str(self.cols))
        self._dens_var = tk.StringVar(value="30")
        entry_row(panel, "Rows:", self._rows_var)
        entry_row(panel, "Cols:", self._cols_var)
        entry_row(panel, "Density %:", self._dens_var)
        self._btn("Generate Map", self._generate_map, panel, C["accent2"])

        sep()

        # algorithm
        lbl(panel, "ALGORITHM")
        self._algo_var = tk.StringVar(value="A* Search")
        ttk.Combobox(panel, textvariable=self._algo_var,
                     values=list(ALGORITHMS.keys()), state="readonly",
                     font=("Consolas",10)).pack(fill=tk.X, pady=2)

        lbl(panel, "HEURISTIC")
        self._heur_var = tk.StringVar(value="Manhattan")
        ttk.Combobox(panel, textvariable=self._heur_var,
                     values=list(HEURISTICS.keys()), state="readonly",
                     font=("Consolas",10)).pack(fill=tk.X, pady=2)

        sep()

        # dynamic mode
        lbl(panel, "DYNAMIC MODE")
        self._dyn_var = tk.BooleanVar(value=False)
        tk.Checkbutton(panel, text=" Enable Dynamic Obstacles",
                       variable=self._dyn_var,
                       bg=C["panel"], fg=C["text"], selectcolor=C["accent2"],
                       activebackground=C["panel"], activeforeground=C["accent"],
                       font=("Consolas",10)).pack(anchor="w")
        self._spawn_var = tk.StringVar(value="8")
        entry_row(panel, "Spawn prob %:", self._spawn_var)

        sep()

        # action buttons
        self._btn("▶  Run Search",     self._run_search,    panel, C["accent"])
        self._btn("↺  Reset Visited",  self._reset_visited, panel, C["accent2"])
        self._btn("✕  Clear All",      self._clear_all,     panel, C["accent2"])
        self._btn("■  Stop Agent",     self._stop_agent,    panel, "#444")

        sep()

        # metrics dashboard
        lbl(panel, "METRICS")
        self._metric_row(panel, "Nodes Visited:", self.nodes_visited)
        self._metric_row(panel, "Path Cost:",     self.path_cost)
        self._metric_row(panel, "Time (ms):",     self.exec_time_ms, fmt="{:.2f}")
        self._metric_row(panel, "Re-plans:",      self.replans)

        sep()

        # legend
        lbl(panel, "LEGEND")
        for colour, label in [
            (C["start"],    "Start"),
            (C["goal"],     "Goal"),
            (C["wall"],     "Wall"),
            (C["frontier"], "Frontier"),
            (C["visited"],  "Visited"),
            (C["path"],     "Path"),
            (C["agent"],    "Agent"),
        ]:
            f = tk.Frame(panel, bg=C["panel"])
            f.pack(anchor="w", pady=1)
            tk.Frame(f, bg=colour, width=14, height=14).pack(side=tk.LEFT, padx=(0,6))
            tk.Label(f, text=label, bg=C["panel"], fg=C["text"],
                     font=("Consolas",9)).pack(side=tk.LEFT)

        # instructions
        sep()
        instr = ("Left-click: toggle wall\n"
                 "Right-click: set Start/Goal")
        tk.Label(panel, text=instr, bg=C["panel"], fg="#888",
                 font=("Consolas",8), justify=tk.LEFT).pack(anchor="w")

        # ── canvas frame ───────────────────────
        self._canvas_frame = tk.Frame(self, bg=C["bg"])
        self._canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)

        self._canvas = tk.Canvas(self._canvas_frame, bg=C["bg"],
                                 highlightthickness=0)
        self._canvas.pack(fill=tk.BOTH, expand=True)
        self._canvas.bind("<Button-1>",        self._on_left_click)
        self._canvas.bind("<B1-Motion>",       self._on_left_drag)
        self._canvas.bind("<Button-3>",        self._on_right_click)
        self._canvas.bind("<Configure>",       lambda e: self._draw_grid())

    def _btn(self, text, cmd, parent, bg):
        b = tk.Button(parent, text=text, command=cmd,
                      bg=bg, fg=C["text"], activebackground=C["accent"],
                      activeforeground="white", font=("Consolas",10,"bold"),
                      relief=tk.FLAT, padx=6, pady=4, cursor="hand2")
        b.pack(fill=tk.X, pady=2)
        return b

    def _metric_row(self, parent, label, var, fmt="{}"):
        f = tk.Frame(parent, bg=C["panel"])
        f.pack(fill=tk.X, pady=1)
        tk.Label(f, text=label, bg=C["panel"], fg="#aaa",
                 font=("Consolas",9), width=14, anchor="w").pack(side=tk.LEFT)
        val_lbl = tk.Label(f, bg=C["panel"], fg=C["accent"],
                           font=("Consolas",9,"bold"), anchor="w")
        val_lbl.pack(side=tk.LEFT)

        def _update(*_):
            v = var.get()
            val_lbl.config(text=fmt.format(v))
        var.trace_add("write", _update)
        _update()

    # ══════════════════════════════════════════
    #  GRID DRAWING
    # ══════════════════════════════════════════
    def _cell_size(self):
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        if cw < 10 or ch < 10:
            return self.CELL
        cs = min(cw // self.cols, ch // self.rows)
        return max(self.MIN_CELL, min(self.MAX_CELL, cs))

    def _cell_origin(self, r, c):
        cs = self._cell_size()
        total_w = cs * self.cols
        total_h = cs * self.rows
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        ox = (cw - total_w) // 2
        oy = (ch - total_h) // 2
        return ox + c*cs, oy + r*cs

    def _pos_from_xy(self, x, y):
        cs = self._cell_size()
        total_w = cs * self.cols
        total_h = cs * self.rows
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        ox = (cw - total_w) // 2
        oy = (ch - total_h) // 2
        c = (x - ox) // cs
        r = (y - oy) // cs
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (int(r), int(c))
        return None

    def _draw_grid(self):
        self._canvas.delete("all")
        cs = self._cell_size()
        for r in range(self.rows):
            for c in range(self.cols):
                self._draw_cell(r, c, cs)

    def _cell_colour(self, r, c):
        pos = (r, c)
        if pos == self.start:           return C["start"]
        if pos == self.goal:            return C["goal"]
        if pos in self.walls:           return C["wall"]
        if pos == self.agent_pos:       return C["agent"]
        if pos in set(self.path):       return C["path"]
        if pos in self.frontier_cells:  return C["frontier"]
        if pos in self.visited_cells:   return C["visited"]
        return C["empty"]

    def _draw_cell(self, r, c, cs=None):
        if cs is None:
            cs = self._cell_size()
        x0, y0 = self._cell_origin(r, c)
        x1, y1 = x0+cs-1, y0+cs-1
        fill = self._cell_colour(r, c)
        tag = f"cell_{r}_{c}"
        self._canvas.delete(tag)
        self._canvas.create_rectangle(x0, y0, x1, y1,
                                      fill=fill, outline=C["grid"],
                                      width=1, tags=tag)
        # draw agent indicator
        if (r,c) == self.agent_pos:
            pad = cs//4
            self._canvas.create_oval(x0+pad, y0+pad, x1-pad, y1-pad,
                                     fill=C["agent"], outline="", tags=tag)
        # labels for start/goal
        pos = (r,c)
        if pos == self.start:
            self._canvas.create_text(x0+cs//2, y0+cs//2, text="S",
                                     fill="black", font=("Consolas",max(8,cs//3),"bold"),
                                     tags=tag)
        elif pos == self.goal:
            self._canvas.create_text(x0+cs//2, y0+cs//2, text="G",
                                     fill="black", font=("Consolas",max(8,cs//3),"bold"),
                                     tags=tag)

    def _refresh_cell(self, r, c):
        self._draw_cell(r, c)

    # ══════════════════════════════════════════
    #  MAP GENERATION
    # ══════════════════════════════════════════
    def _generate_map(self):
        self._stop_agent()
        try:
            self.rows = max(5, int(self._rows_var.get()))
            self.cols = max(5, int(self._cols_var.get()))
            density = max(0, min(80, int(self._dens_var.get()))) / 100
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid integers.")
            return

        self.walls = set()
        self.start = (0, 0)
        self.goal  = (self.rows-1, self.cols-1)
        protected = {self.start, self.goal}

        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) not in protected and random.random() < density:
                    self.walls.add((r,c))

        self._reset_visited()

    # ══════════════════════════════════════════
    #  MOUSE EVENTS
    # ══════════════════════════════════════════
    def _on_left_click(self, event):
        pos = self._pos_from_xy(event.x, event.y)
        if pos and pos not in {self.start, self.goal}:
            if pos in self.walls:
                self.walls.discard(pos)
            else:
                self.walls.add(pos)
            self._draw_cell(*pos)

    def _on_left_drag(self, event):
        pos = self._pos_from_xy(event.x, event.y)
        if pos and pos not in {self.start, self.goal}:
            self.walls.add(pos)
            self._draw_cell(*pos)

    def _on_right_click(self, event):
        pos = self._pos_from_xy(event.x, event.y)
        if not pos or pos in self.walls:
            return
        # cycle: click sets start, next click sets goal, etc.
        if self._right_click_mode == "start":
            old = self.start
            self.start = pos
            self._right_click_mode = "goal"
            self._draw_cell(*old)
            self._draw_cell(*pos)
        else:
            old = self.goal
            self.goal = pos
            self._right_click_mode = "start"
            self._draw_cell(*old)
            self._draw_cell(*pos)

    # ══════════════════════════════════════════
    #  SEARCH  (with step-by-step visualisation)
    # ══════════════════════════════════════════
    def _run_search(self):
        self._stop_agent()
        self._reset_visited(redraw=False)
        algo = ALGORITHMS[self._algo_var.get()]
        heur = HEURISTICS[self._heur_var.get()]

        # collect step events during search
        events = []   # list of (pos, kind, frontier_snapshot)

        def step_cb(pos, kind, frontier):
            events.append((pos, kind, set(frontier)))

        t0 = time.perf_counter()
        path, visited_order, _ = algo(
            self.start, self.goal,
            self.rows, self.cols,
            self.walls, heur, step_cb
        )
        elapsed = (time.perf_counter() - t0) * 1000

        self.exec_time_ms.set(round(elapsed, 3))
        self.nodes_visited.set(len(visited_order))
        self.path_cost.set(len(path)-1 if path else 0)
        self.replans.set(0)

        if path is None:
            messagebox.showwarning("No Path", "No path found! Try removing some walls.")
            return

        # animate exploration then path
        self._animate_search(events, path)

    def _animate_search(self, events, path):
        DELAY = 18   # ms per step

        def play_events(idx=0):
            if idx < len(events):
                pos, kind, frontier = events[idx]
                if kind == "visited":
                    self.visited_cells.add(pos)
                else:
                    self.frontier_cells.add(pos)
                    self.frontier_cells -= self.visited_cells
                self._draw_cell(*pos)
                # also refresh frontier cells that changed
                for fp in frontier:
                    if fp not in self.visited_cells:
                        self._draw_cell(*fp)
                self._anim_id = self.after(DELAY, play_events, idx+1)
            else:
                # show final path
                self.path = path
                for p in path:
                    self._draw_cell(*p)
                # start dynamic agent if enabled
                if self._dyn_var.get():
                    self.agent_pos = path[0]
                    self._anim_id = self.after(150, self._move_agent, 1)

        play_events()

    # ══════════════════════════════════════════
    #  DYNAMIC AGENT MOVEMENT & RE-PLANNING
    # ══════════════════════════════════════════
    def _move_agent(self, step_idx):
        if not self.dynamic_running and self.agent_pos is not None:
            # first call initialises flag
            pass
        self.dynamic_running = True

        if step_idx >= len(self.path):
            # reached goal
            self.agent_pos = self.goal
            self._draw_cell(*self.goal)
            self.dynamic_running = False
            messagebox.showinfo("Done", "Agent reached the goal!")
            return

        # spawn obstacles with probability
        try:
            prob = max(0, min(50, int(self._spawn_var.get()))) / 100
        except ValueError:
            prob = 0.08

        spawned_on_path = False
        if prob > 0:
            for r in range(self.rows):
                for c in range(self.cols):
                    pos = (r,c)
                    if (pos not in {self.start, self.goal}
                            and pos != self.agent_pos
                            and pos not in self.walls
                            and random.random() < prob / (self.rows*self.cols) * 6):
                        self.walls.add(pos)
                        self._draw_cell(*pos)
                        if pos in self.path[step_idx:]:
                            spawned_on_path = True

        # re-plan if needed
        if spawned_on_path:
            algo = ALGORITHMS[self._algo_var.get()]
            heur = HEURISTICS[self._heur_var.get()]
            t0 = time.perf_counter()
            new_path, visited_order, _ = algo(
                self.agent_pos, self.goal,
                self.rows, self.cols,
                self.walls, heur
            )
            elapsed = (time.perf_counter() - t0)*1000
            self.exec_time_ms.set(round(elapsed, 3))
            self.replans.set(self.replans.get() + 1)
            self.nodes_visited.set(self.nodes_visited.get() + len(visited_order))

            if new_path is None:
                messagebox.showwarning("Blocked!", "Agent is blocked – no path exists!")
                self.dynamic_running = False
                return

            # clear old path visuals
            for p in self.path:
                if p not in self.walls:
                    self.visited_cells.discard(p)
                    self._draw_cell(*p)

            self.path = new_path
            self.path_cost.set(len(new_path)-1)
            for p in new_path:
                self._draw_cell(*p)
            step_idx = 1   # restart from next step in new path

        # move agent
        prev = self.agent_pos
        self.agent_pos = self.path[step_idx]

        # redraw previous cell
        if prev:
            self._draw_cell(*prev)
        self._draw_cell(*self.agent_pos)

        self._anim_id = self.after(160, self._move_agent, step_idx+1)

    # ══════════════════════════════════════════
    #  HELPERS
    # ══════════════════════════════════════════
    def _stop_agent(self):
        if self._anim_id:
            self.after_cancel(self._anim_id)
            self._anim_id = None
        self.dynamic_running = False
        self.agent_pos = None

    def _reset_visited(self, redraw=True):
        self._stop_agent()
        self.visited_cells.clear()
        self.frontier_cells.clear()
        self.path = []
        self.nodes_visited.set(0)
        self.path_cost.set(0)
        self.exec_time_ms.set(0.0)
        self.replans.set(0)
        if redraw:
            self._draw_grid()

    def _clear_all(self):
        self._stop_agent()
        self.walls.clear()
        self._reset_visited()


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
if __name__ == "__main__":
    app = PathfindingApp()
    # apply ttk styling
    style = ttk.Style(app)
    style.theme_use("clam")
    style.configure("TCombobox",
                    fieldbackground=C["accent2"],
                    background=C["accent2"],
                    foreground=C["text"],
                    arrowcolor=C["text"],
                    selectbackground=C["accent"],
                    selectforeground="white")
    app.mainloop()