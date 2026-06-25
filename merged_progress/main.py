"""Interactive task launcher for the merged G1 locomotion project.

Run:
    python main.py

Pick a task by pressing its key (single keypress, no Enter needed):
    1..9, 0, a, b, c  ->  run that task
    v                 ->  toggle the live MuJoCo viewer on/off
    r                 ->  toggle the robot model (G1 <-> Talos)
    ESC or q          ->  exit

Each task runs in an isolated subprocess, so a failure in one never crashes the
menu. Press Ctrl+C during a task to abort it and return to the menu.
"""
import os
import sys
import subprocess

ROOT = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

# key -> (title, [script+args], supports_viewer, supports_robot, "what you should see")
TASKS = [
    ("1", "Inspect model (DOF, torque actuators, frames)",
     ["scripts/00_inspect_model.py"], False, False,
     "nq=36 nv=35 nu=29, ALL actuators motor/torque, foot/pelvis frames, mass 33.3 kg (G1)."),
    ("2", "Gravity compensation — dynamics sanity (Stage 1)",
     ["scripts/01_gravity_comp.py"], True, False,
     "Robot stands perfectly still by feedforward torque (drift ~1.7 mm). RESULT: PASS."),
    ("3", "Standing balance: weight-shift + single-support (Stage 2)",
     ["scripts/02_stand_balance.py"], True, False,
     "Stands, sways left/right, then balances on ONE foot. QP 100% feasible. RESULT: PASS."),
    ("4", "Offline planner plots — footsteps + DCM (Stage 3)",
     ["scripts/03_plan_walk.py"], False, False,
     "Saves logs/stage3_plan.png (footsteps + CoM/DCM/ZMP, no robot motion). RESULT: PASS."),
    ("5", "Flat walking",
     ["scripts/run_walk.py", "--terrain", "flat"], True, True,
     "Robot walks forward ~0.9-1.0 m on flat ground. RESULT: PASS."),
    ("6", "Incline walking (12 deg uphill)",
     ["scripts/run_walk.py", "--terrain", "incline", "--angle", "12"], True, True,
     "Robot climbs a 12 deg slope, feet land flat. RESULT: PASS (G1 to 16 deg, Talos to ~8 deg)."),
    ("7", "Stairs climbing (6 x 2.5 cm)",
     ["scripts/run_walk.py", "--terrain", "stairs"], True, True,
     "G1 climbs a full staircase tread-by-tread (+0.15 m). (Talos stairs need own tuning.)"),
    ("8", "Omnidirectional — walk a curve (turn while walking)",
     ["scripts/run_omni.py", "--vx", "0.10", "--vyaw", "0.12"], True, True,
     "Robot walks a curved path turning ~42 deg. (Try --vy 0.08 for strafe.) RESULT: PASS."),
    ("9", "Push recovery while walking",
     ["scripts/05_push_recovery.py"], True, False,
     "External shoves hit the pelvis mid-walk; the robot steps to recover (G1). RESULT: PASS."),
    ("0", "Standing-on-incline slip-limit sweep (theory vs experiment)",
     ["scripts/06_walk_incline.py", "--sweep"], False, False,
     "Stands up to 26 deg, slips at 27 deg == arctan(mu). Matches theory (G1)."),
    ("a", "Generate lecture-style plots (path / footsteps / CoM height)",
     ["scripts/plot_walk.py", "--terrain", "stairs"], False, False,
     "Saves logs/plot_walk_stairs.png: footstep placement + CoM height climbing the treads."),
    ("b", "Full evaluation battery (all scenarios -> report)",
     ["scripts/evaluate.py"], False, False,
     "Runs everything headless, writes logs/eval_report.md. All scenarios PASS (G1)."),
]
TASK_BY_KEY = {t[0]: t[1:] for t in TASKS}


def getkey():
    """Read a single keypress (no Enter). Returns the character; ESC -> '\\x1b'."""
    try:
        import msvcrt
        ch = msvcrt.getwch()
        return ch
    except ImportError:
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch


def menu(viewer, robot):
    print("\n" + "=" * 72)
    print(f"  Unitree {robot.upper()} — Merged Locomotion  (torque WBC + DCM)")
    print("=" * 72)
    for (k, title, _cmd, vw, rb, _see) in TASKS:
        tag = ("  [viewer]" if vw else "") + ("  [robot]" if rb else "")
        print(f"  [{k}]  {title}{tag}")
    print("-" * 72)
    print(f"  [v]  toggle viewer   (now: {'ON' if viewer else 'OFF'})    "
          f"[r]  toggle robot   (now: {robot.upper()})")
    print("  [ESC / q]  exit")
    print("=" * 72)
    print("Press a key...")


def run_task(key, viewer, robot):
    title, cmd, supports_vw, supports_rb, see = TASK_BY_KEY[key]
    args = list(cmd)
    if viewer and supports_vw:
        args.append("--viewer")
    if supports_rb:
        args += ["--robot", robot]
    elif robot != "g1":
        print(f"\n(note: task '{title}' is G1-only; running on G1.)")
    print("\n" + "-" * 72)
    print(f"RUN [{robot.upper()}]: {title}")
    print(f"WHAT YOU SHOULD SEE: {see}")
    print(f"$ python {' '.join(args)}")
    print("-" * 72)
    try:
        subprocess.run([PY, *[os.path.join(ROOT, args[0])] + args[1:]], cwd=ROOT)
    except KeyboardInterrupt:
        print("\n[aborted -> back to menu]")
    print("\n[done -> press a key for the menu]")
    getkey()


def main():
    viewer = False
    robot = "g1"
    while True:
        menu(viewer, robot)
        ch = getkey()
        if ch in ("\x1b", "q", "Q"):
            print("bye.")
            return
        if ch in ("v", "V"):
            viewer = not viewer
            continue
        if ch in ("r", "R"):
            robot = "talos" if robot == "g1" else "g1"
            continue
        if ch in TASK_BY_KEY:
            run_task(ch, viewer, robot)
        # any other key: just redraw the menu


if __name__ == "__main__":
    main()
