# Δυναμική Βάδιση Unitree G1 σε MuJoCo με Torque-level Whole-Body Control

> **Ζωντανή αναφορά (report backbone).** Ενημερώνεται με ΚΑΘΕ ουσιαστική αλλαγή (DESCRIPTION.md §9).
> Δομή: Τίτλος → Abstract → Πρόβλημα → Θεωρία → Αρχιτεκτονική → Υλοποίηση → Παράμετροι →
> Πειράματα/Αποτελέσματα → Decisions & Changelog → Πώς τρέχει → Περιορισμοί → Αναφορές.

**Ομάδα:** 2 άτομα · **Μάθημα:** Robotic Systems 1 · **Πλατφόρμα:** MuJoCo 3.4 + Python 3.11

---

## 2. Abstract

Κάνουμε το **Unitree G1** (humanoid, 29 actuated DOF + floating base) να **στέκεται** και να
**περπατά δυναμικά** σε MuJoCo, πρώτα σε επίπεδο έδαφος και μετά σε **κεκλιμένο** επίπεδο.
Η προσέγγιση είναι **dynamics-first / torque-level Whole-Body Control**: ανά control step λύνουμε
ένα **QP Inverse Dynamics** (Lecture 9) με μεταβλητές `[v̇; τ; f]`, και στέλνουμε **ροπές τ** στους
torque actuators (όχι IK+PD). Ο planner (footsteps + DCM/ZMP) είναι ανεξάρτητος από τον controller.
Όλη η balance-λογική εκφράζεται **ως προς τη βαρύτητα / την επιφάνεια** ώστε να γενικεύεται στην κλίση.

**Κατάσταση:** Στάδιο 0 (setup) ✅ · Στάδιο 1 (gravity comp) ⏳ — βλ. §9 Changelog.

---

## 3. Πρόβλημα & Στόχοι

- Δυναμική, σταθερή βάδιση ενός humanoid σε MuJoCo με έλεγχο σε επίπεδο ροπής.
- Robust σε flat, σε ήπιες διαταραχές (push), και σε **κεκλιμένο επίπεδο** (το challenge).
- Καθαρά interfaces: ο WBC controller αντικαταστάσιμος, ο planner ανεξάρτητος.
- **Done όρια ανά στάδιο** ορίζονται στο `CHECKLIST.md`.

---

## 4. Θεωρητικό υπόβαθρο

### 4.1 Floating-base δυναμική
```
M(q) v̇ + h(q,v) = Sᵀ τ + Σ_c J_cᵀ f_c
```
- `M(q)` (nv×nv): mass matrix — από `mj_fullM`.
- `h(q,v)` (nv): bias (Coriolis + φυγόκεντρες + βαρύτητα) — από `data.qfrc_bias`.
- `S` (nu×nv): selection matrix· τα **6 DOF της βάσης είναι μη ενεργοποιούμενα**.
- `J_c`, `f_c`: contact Jacobian & δύναμη επαφής ανά σημείο επαφής πέλματος.

### 4.2 Whole-Body Control ως QP (Lecture 9)
Μεταβλητές `x = [v̇ ; τ ; f]`.
```
min_x  ½ xᵀ Q x + qᵀ x      με  Σ_i w_i ‖W_i x − t_i‖²   →  Q = ΣwWᵀW,  q = −ΣwWᵀt
s.t.   [M  −Sᵀ  −Jᵀ] x = −h               (δυναμική, equality)
       J_c v̇ + J̇_c v = 0                  (stance πέλματα, equality)
       friction cones, τ_min ≤ τ ≤ τ_max  (inequalities)
```

### 4.3 Tasks (acceleration-level με PD)
```
a_des = a_ref + Kp (x_ref − x) + Kd (ẋ_ref − ẋ)
residual:  J_task v̇ + J̇_task v − a_des ≈ 0
```
Tasks: **CoM** (μεγάλο βάρος), **swing/stance-foot**, **torso orientation** (κάθετο ως προς
βαρύτητα), **posture** (regularization). Προαιρετικά angular momentum.

### 4.4 Friction cones (κλειδί για την κλίση)
Γραμμικοποιημένος κώνος (πυραμίδα) **στο frame της επιφάνειας** (n = surface normal):
```
f_n ≥ f_min ,   |f_t1| ≤ μ f_n ,   |f_t2| ≤ μ f_n
```
Σε κλίση, `n` = κάθετος της επιφάνειας (όχι world-z) → ο solver «ξέρει» τι είναι εφικτό.

### 4.5 Walking pattern (Lecture 10)
- **LIPM**: `p_ZMP = p_CoM − (z/g) p̈_CoM`.
- **DCM / capture point**: `ξ = p_CoM + ṗ_CoM/ω`, `ω = √(g/z)` — foot placement & push recovery.
- **Support polygon**: convex hull σημείων επαφής· ZMP εντός.

### 4.6 FSM βάδισης
`DS_init → SS_L → DS → SS_R → DS → …`, σταθερός χρονισμός (π.χ. SS≈0.7s, DS≈0.2s).

---

## 5. Αρχιτεκτονική συστήματος (pipeline)
```
Offline:  footsteps + ZMP reference
   → Online: DCM / capture-point re-plan (CoM traj + foot paths + contact schedule)
   → WBC QP Inverse Dynamics (tasks: CoM+feet+torso+posture ; constraints: dynamics,
                              contacts, friction cones, torque limits) → τ
   → MuJoCo G1 (torque actuators)
   → feedback: base pose/twist, foot/CoM poses, contact state
```
Δομή κώδικα: βλ. `DESCRIPTION.md §5`. Modules: `src/sim`, `src/dynamics`, `src/control`,
`src/planning`, `src/utils`.

---

## 6. Λεπτομέρειες υλοποίησης (ανά module)

### Στάδιο 0 — Setup & μοντέλο ✅
- **Μοντέλο:** `models/unitree_g1/g1.xml` = το G1 του `mujoco_menagerie` με **μετατροπή των
  actuators από `position` → `motor` (torque)**. `ctrlrange` κάθε motor = `actuatorfrcrange`
  της άρθρωσης (π.χ. hip_pitch ±88, knee ±139, ankle ±50, wrist ±5 N·m). `meshdir` δείχνει στα
  assets του menagerie (no copy). Προστέθηκαν 4 **corner sites** ανά πέλμα για contact Jacobians.
- **Scenes:** `scene_flat.xml` (επίπεδο), `scene_incline.xml` (κεκλιμένο plane, euler about-y).
- **Discovery (`scripts/00_inspect_model.py`):** `nq=36, nv=35, nu=29`· floating base = dof 0–5
  (μη ενεργοποιούμενα)· **ΟΛΟΙ οι actuators motor/torque** (gain=fixed, bias=none)· μάζα
  **33.34 kg** → βάρος **327 N**· base height 0.79 m. Frames: `pelvis`, `torso_link`,
  `left_foot`/`right_foot` (+ corners).
- **`src/sim/mujoco_env.py`:** load/reset/step/set_torques (+ ctrl clamp), selection matrix `S`.
- **`src/utils/config.py`:** φορτώνει το `config/params.yaml` (single source of truth).

### Στάδιο 1 — Dynamics plumbing & gravity compensation ✅
- **`src/dynamics/model_terms.py`:** εξάγει `M` (`mj_fullM`), `h` (`qfrc_bias` = Coriolis+βαρύτητα),
  selection matrix `S`, contact Jacobians `J_c` (translational `mj_jacSite` στα 8 corner points),
  και drift `J̇_c v` (spatial site accel με `q̈=0` μέσω `mj_rnePostConstraint` — αναλυτικό, =0 στο στατικό).
- **`src/control/gravity_comp.py`:** στατική Inverse Dynamics. Στο `v=0,v̇=0`: `Sᵀτ + Σ J_cᵀ f = h`.
  Οι 6 base (μη ενεργοποιούμενες) εξισώσεις λύνονται για contact forces `f` ως **equality-constrained
  least squares** γύρω από ομοιόμορφη κατακόρυφη κατανομή βάρους (`f₀ = W/8` ανά σημείο):
  `f = f₀ + Aᵀ(AAᵀ+εI)⁻¹(b − A f₀)`, `A = (J_cᵀ)|base`. Μετά `τ = (h − J_cᵀ f)|actuated`.
  Επαναϋπολογίζεται κάθε control step → quasi-static feedback, απορρίπτει numerical drift.
- **Αποτέλεσμα (`scripts/01_gravity_comp.py`, 5 s @ 500 Hz):** base drift **1.6 mm**, CoM drift
  **0.66 mm**, base |vel|→~0, dynamics residual **2.3e-4**, max |τ| 1.4 N·m. → **PASS** (όρθιο & ακίνητο
  μόνο με feedforward ροπές, όχι glued). Plot: `logs/stage1_gravity_comp.png`.

### Στάδιο 2 — WBC QP & standing balance ⏳ _(placeholder)_
### Στάδιο 3 — Planning layer ⏳ _(placeholder)_
### Στάδιο 4 — Δυναμική βάδιση σε flat ⏳ _(placeholder)_
### Στάδιο 5 — Robustness / push recovery ⏳ _(placeholder)_
### Στάδιο 6 — Κεκλιμένο επίπεδο ⏳ _(placeholder)_

---

## 7. Παράμετροι (από `config/params.yaml`)

| Παράμετρος | Τιμή | Νόημα |
|---|---|---|
| `model.base_body` | `pelvis` | floating base |
| `model.torso_body` | `torso_link` | torso orientation task |
| `wbc.friction_mu` | 0.6 | συντελεστής τριβής (= foot geom) |
| `wbc.f_min` | 10 N | ελάχιστη normal δύναμη ανά επαφή |
| `wbc.solver` | quadprog | QP backend (proxqp δεν έχει Windows wheel) |
| `env.gravity` | 9.81 | — |
| `env.incline_deg` | 3.0 | αρχική κλίση (Στάδιο 6) |
| `sim.control_rate_hz` | 1000 | ρυθμός WBC QP |

_(Συμπληρώνεται με task gains/βάρη καθώς προχωράμε.)_

---

## 8. Πειράματα & Αποτελέσματα

_TODO ανά στάδιο: flat walking (distance/duration/#steps), push recovery, max κλίση. Γραφήματα
από `logs/`._

---

## 9. Decisions & Changelog

- **2026-06-24 — Στάδιο 0 (setup):**
  - Στήθηκε δομή φακέλων (`src/`, `scripts/`, `config/`, `models/`, `logs/`, `report/`).
  - **Απόφαση:** μετατροπή των G1 actuators `position → motor (torque)` — απαιτείται για
    torque-level WBC (DESCRIPTION §6). `ctrlrange = actuatorfrcrange`.
  - **Απόφαση:** corner sites ανά πέλμα (4×2) για μελλοντικά contact Jacobians/friction cones.
  - **Απόφαση solver:** `proxsuite/proxqp` δεν χτίζεται σε Windows → χρήση `quadprog` (default,
    dense PD) με fallback `osqp`. Δεν αλλάζει η μέθοδος, μόνο το backend.
  - **Επαλήθευση:** `00_inspect_model.py` → όλοι οι actuators motor/torque· headless test:
    εντολή 20 N·m στο knee → `actuator_force=20`, clamp στα limits. ✅ Done όρος Σταδίου 0.

---

- **2026-06-24 — Στάδιο 1 (gravity compensation):**
  - Νέα modules `src/dynamics/model_terms.py` (M, h, S, J_c, J̇v) και
    `src/control/gravity_comp.py` (static ID).
  - **Απόφαση:** point contacts στα 8 foot corner sites (3D δύναμη/σημείο), friction στο world-z
    (flat)· θα γενικευτεί σε surface normal στο Στάδιο 6.
  - **Απόφαση:** `J̇v` μέσω `mj_objectAcceleration` με `q̈=0` (αναλυτικό) αντί finite-diff.
  - **Επαλήθευση:** base drift 1.6 mm, CoM 0.66 mm, residual 2.3e-4 → PASS (Done όρος Σταδίου 1).

## 10. Πώς τρέχει

```bash
pip install -r requirements.txt            # qpsolvers + quadprog/osqp/clarabel (Windows wheels)
python scripts/00_inspect_model.py         # Στάδιο 0: DOF, actuators, frames
python -m mujoco.viewer --mjcf=models/unitree_g1/scene_flat.xml   # οπτικός έλεγχος μοντέλου
python scripts/01_gravity_comp.py            # Στάδιο 1: gravity comp (headless metrics, PASS/FAIL)
python scripts/01_gravity_comp.py --viewer   # ίδιο με οπτικό παράθυρο
# (επόμενα scripts ανά στάδιο: 02_stand_balance.py, 03_walk_flat.py, ...)
```

---

## 11. Περιορισμοί & μελλοντική δουλειά
- QP backend `quadprog` αντί `proxqp` (περιορισμός Windows). 
- _TODO: συμπληρώνεται._

---

## 12. Αναφορές
- Lecture 8 (CoM/biped balancing), Lecture 9 (WBC via QP), Lecture 10 (ZMP/LIPM/DCM).
- Kuindersma et al. 2016 — QP whole-body control (Atlas). Englsberger et al. 2015 — DCM.
- Pratt et al. 2006 — capture point. Del Prete — TSID. Caron — scaron.info.
