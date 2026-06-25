# Δυναμική Βάδιση Unitree G1 — Merged (Marios + Kanellos)

> **Merged υλοποίηση.** Κρατά την **αρχιτεκτονική/λογική του Μάριου** (καθαρό
> offline → online → WBC pipeline, stage-by-stage, θεωρητική ανάλυση) και
> **υλοποιεί όλα τα tasks του Κανέλλου** (terrain-aware incline/stairs,
> omnidirectional, push recovery, evaluation harness).

**Πλατφόρμα:** MuJoCo 3.4 · Python 3.11 · torque-level Whole-Body Control (QP Inverse Dynamics).

---

## 1. Αρχιτεκτονική (pipeline)

```
OFFLINE  (ανά πλάνο)            ONLINE  (ανά control step)         WBC (ανά step)
─────────────────────          ──────────────────────────        ───────────────
footstep planner   ┐           DCM feedback (capture point)       QP Inverse Dynamics
 + terrain.height  ├─► CoM/ZMP ─► xi = x + v/ω, p_zmp_cmd  ──────► min Σwᵢ‖Jᵢv̇−bᵢ‖²
 + footstep_x      │   (DCM     reactive step adjustment           s.t. dynamics,
gait FSM (DS/SS)   ┘   ref)     CoM-height low-pass                 contacts (6D),
swing splines                  heading/yaw track                   friction cones
                                                                    (surface frame),
                                                                    torque limits  → τ
        ▲────────────── feedback: CoM, foot poses, contacts, base pose ─────────────┘
                                   ↓ τ
                              MuJoCo G1 (torque actuators)
```

- **Offline** (`src/planning/`): footsteps (flat/incline/stairs/omnidirectional),
  ZMP reference, **DCM** trajectory (backward recursion), swing splines, gait timeline.
- **Online** (`src/control/walking_controller.py`): DCM feedback CoM law, capture-point
  step adjustment, terrain-aware friction-cone orientation, CoM-height low-pass, heading.
- **WBC** (`src/control/wbc_qp.py`): QP inverse dynamics, hard 6D contact + surface-frame
  friction cones + torque limits, safe fallback.

---

## 2. Τι κάνει (όλα επαληθευμένα — `scripts/evaluate.py`)

| Δυνατότητα | Αποτέλεσμα | Script |
|---|---|---|
| Gravity-comp sanity (Στ.1) | drift 1.7 mm | `01_gravity_comp.py` |
| WBC standing + weight-shift + single-support | PASS | `02_stand_balance.py` |
| **Flat walking** | 0.90 m, tilt 3.6° | `run_walk.py --terrain flat` |
| **Incline walking** | **8/12/16° (rise +126/191/255 mm)** | `run_walk.py --terrain incline --angle 16` |
| **Stairs climb** | **6×2.5 cm, +149 mm, full** | `run_walk.py --terrain stairs` |
| **Omnidirectional** | fwd/back/strafe/**curve +42°** | `run_omni.py --vx 0.1 --vyaw 0.12` |
| **Push recovery (walk)** | lateral 50 N / sagittal 100 N | `05_push_recovery.py` |
| Standing on incline (slip-limited) | **stable έως 26° = arctan μ** | `06_walk_incline.py --sweep` |

**Σημαντικό:** το incline walking πήγε από **3° → 16°** στο merge, χάρη στο
**terrain-aware design** (footsteps ON the surface, friction cones σε surface frame,
swing feet aligned to normal) — η κύρια συνεισφορά που υιοθετήθηκε από τον Κανέλλο.

---

## 3. Ανάλυση ορίων κλίσης (απλά μαθηματικά)

- **Ολίσθηση:** `mg sinα ≤ μ mg cosα` ⟹ `α_slip = arctan(μ) = arctan(0.5) = 26.6°`.
- **Ανατροπή:** `α_tip = arctan(d/H)`· ο ενεργός WBC κρατά το CoM πάνω από τα πέλματα,
  οπότε η ανατροπή **δεν δεσμεύει** → standing **slip-limited**.
- **Πείραμα:** standing σταθερό **έως 26°, ολισθαίνει στις 27°** — ταιριάζει με τη θεωρία.
- Incline **walking** (16°) περιορίζεται δυναμικά (single-support), όχι από slip/tip.

---

## 4. Πώς τρέχει

### Διαδραστικό μενού (συνιστάται)
```bash
pip install -r requirements.txt
python main.py        # μενού: πάτα [1]..[b] για task, [v] viewer on/off, [ESC] έξοδος
```
Κάθε task τρέχει σε **απομονωμένο subprocess** (robust: σφάλμα σε ένα δεν ρίχνει το μενού)
και τυπώνει «τι θα δεις». Με `[v]` ανοίγεις τον ζωντανό MuJoCo viewer.

### Ταχύτητα βάδισης
- **~0.11 m/s** μέση (καθαρή απόσταση / συνολικό χρόνο)· **~0.15 m/s** σταθερό forward
  κατά τη συνεχή βάδιση (`step_length/T_step = 0.10/0.65`)· peak CoM ~0.38 m/s (με το sway).
- Ρυθμίζεται από `gait.step_length` / `t_ss` στο `config/params.yaml`, ή `vx` στο `run_omni.py`.

### Απευθείας scripts
```bash
# Stage-by-stage (η ραχοκοκαλιά της λογικής)
python scripts/00_inspect_model.py         # DOF, actuators (torque), frames
python scripts/01_gravity_comp.py          # dynamics sanity (gravity comp)
python scripts/02_stand_balance.py         # WBC standing / weight-shift / single-support
python scripts/03_plan_walk.py             # offline planner plots (no robot)

# Terrain-aware walking
python scripts/run_walk.py --terrain flat
python scripts/run_walk.py --terrain incline --angle 16
python scripts/run_walk.py --terrain stairs
python scripts/run_walk.py --terrain incline --angle 12 --viewer

# Omnidirectional
python scripts/run_omni.py --vx 0.12               # forward
python scripts/run_omni.py --vy 0.08               # strafe
python scripts/run_omni.py --vx 0.10 --vyaw 0.12   # curve

# Push recovery + incline analysis + full evaluation
python scripts/05_push_recovery.py
python scripts/06_walk_incline.py --sweep          # slip-limit vs theory
python scripts/evaluate.py                         # full battery -> logs/eval_report.md

# Lecture-style plots: path planning, footstep placement, CoM height, ZMP/DCM
python scripts/plot_walk.py --terrain flat
python scripts/plot_walk.py --terrain incline --angle 12
python scripts/plot_walk.py --terrain stairs       # CoM height climbs the treads
python scripts/plot_walk.py --omni curve           # curved path + rotated footsteps
```

**Plots (`logs/plot_walk_*.png`)** — 4 panels ανά scenario:
(A) path planning & **footstep placement** top-down (foot rectangles + CoM/DCM/ZMP),
(B) **CoM height** vs forward position με την επιφάνεια του terrain (φαίνεται να σκαρφαλώνει),
(C) CoM/DCM/ZMP vs χρόνο, (D) swing-foot height (clearance).

---

## 5. Τι κρατήθηκε από τον καθένα (merge map)

| Από Μάριο (αρχιτεκτονική/λογική) | Από Κανέλλο (tasks/features) |
|---|---|
| offline/online/WBC pipeline, stage-by-stage scripts | terrain abstraction (`terrain.py`: height/normal/footstep_x) |
| hard **6D contact constraint** WBC (full-rank, quadprog) | terrain-aware **incline walking** (3°→16°) |
| DCM feedback CoM law + capture-point | **stairs** (tread-centre, feet-together-per-tread) |
| gravity-comp sanity test, bent-knee crouch, tray arms | **omnidirectional** (vx, vy, vyaw) |
| **slip/tip physics analysis**, README changelog | mjSpec terrain loading, low-pass CoM-height |
| self-contained menagerie | **evaluation harness** idea |

**Δομικές διαφορές που συμφιλιώθηκαν:** ο Μάριος χρησιμοποιεί hard 6D contact +
quadprog· ο Κανέλλος soft contact + OSQP. Το merged κράτησε το hard-6D του Μάριου
(ακριβές) και πρόσθεσε το terrain-aware layer του Κανέλλου από πάνω.

---

## 6. Περιορισμοί & μελλοντική δουλειά
- **Incline walking ≤ ~16°** (standing 26°)· steeper walking → slope-projected DCM/MPC.
- **Lateral push ~50 N** (sagittal 100 N+)· step-timing adaptation θα το βελτίωνε.
- **Stairs**: δουλεύει για ήπια σκαλιά (2.5 cm)· πιο απότομα χρειάζονται μεγαλύτερο
  βήμα/clearance και per-tread gait tuning.
- **Δεν portαρίστηκαν ακόμη** (υπάρχουν στου Κανέλλου, συμβατά με το pipeline):
  DCM preview-MPC, go-to-goal navigation (potential field), δεύτερο ρομπότ (Talos).

## 7. Credits
- **Marios** — pipeline architecture, WBC (hard 6D contact), DCM feedback, gravity-comp,
  incline physics analysis, documentation.
- **Kanellos** — terrain-aware design (incline/stairs), omnidirectional planning,
  evaluation methodology, mjSpec terrain loading.

## 8. Αναφορές
Lecture 8 (CoM balancing), 9 (WBC via QP), 10 (ZMP/LIPM/DCM). Kuindersma 2016 (QP WBC).
Englsberger 2015 (DCM). Pratt 2006 (capture point). Del Prete (TSID). Caron (scaron.info).
