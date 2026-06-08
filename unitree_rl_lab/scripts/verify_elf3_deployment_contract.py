"""End-to-end static verification of the elf3 deployment contract.

Cross-checks (without booting IsaacLab) that the elf3 training side
(unitree_rl_lab) is byte-compatible with the bxi `amp.py` `normal` mode
deployment. AST-parses the source of each side so the script keeps working
even when run with a vanilla Python (no isaacsim available).

Contract items checked:

  1. amp.py constants: action_scale (29), kps (29), kds (29),
     default_dof_pos (29), num_actions, num_obs, obs_history_len, single_obs_dim
  2. amp.py joint-order permutation tables (mujoco_to_isaac_idx,
     isaac_to_mujoco_idx) are 29-length inverse permutations
  3. bxi `bxi_example_demo.py` `joint_name` tuple (29 entries, HW order)
  4. velocity_env_cfg.py ActionsCfg.scale dict (29 keys) -- per-joint values
     equal amp.py.action_scale[mujoco_to_isaac_idx[i]] (training-order alignment)
  5. velocity_env_cfg.py ObservationsCfg.PolicyCfg term order is
     ang_vel | grav | cmd | joint_pos_rel | joint_vel_rel | last_action
  6. velocity_env_cfg.py history_length == amp.py.obs_history_len
  7. velocity_env_cfg.py sim.dt * decimation == 1 / 50 Hz, matches amp.py timer
  8. unitree.py ELF3_29DOF_CFG init_state.joint_pos values equal
     amp.py.default_dof_pos when ordered by HW (bxi joint_name) order
  9. unitree.py ELF3_29DOF_CFG actuator stiffness/damping values equal
     amp.py.kps / kds when ordered by HW (bxi joint_name) order
 10. export_elf3_onnx.py asserts ONNX I/O name=obs/actions, obs_dim=960

Run:
    python scripts/verify_elf3_deployment_contract.py
"""

from __future__ import annotations

import ast
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path("/home/hpf/wsm/unitree_rl/unitree_rl_lab")
BXI_ROOT = Path("/home/hpf/wsm/bxi_rl_controller_ros2_example")

AMP_PY = BXI_ROOT / "src/bxi_example_py_elf3/bxi_example_py_elf3/inference/amp.py"
DEMO_PY = BXI_ROOT / "src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py"
ENV_CFG = REPO_ROOT / "source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/elf3_29dof/velocity_env_cfg.py"
ROBOT_CFG = REPO_ROOT / "source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py"
EXPORT_SCRIPT = REPO_ROOT / "scripts/export_elf3_onnx.py"


# ----------------------------- reporter -----------------------------

class Report:
    def __init__(self) -> None:
        self.ok = 0
        self.fail = 0
        self.warn = 0

    def check(self, label: str, cond: bool, detail: str = "") -> None:
        tag = "[ OK ]" if cond else "[FAIL]"
        if cond:
            self.ok += 1
        else:
            self.fail += 1
        line = f"{tag}  {label}"
        if detail:
            line += f"  --  {detail}"
        print(line)

    def info(self, label: str, detail: str = "") -> None:
        print(f"[INFO]  {label}" + (f"  --  {detail}" if detail else ""))

    def warning(self, label: str, detail: str = "") -> None:
        self.warn += 1
        print(f"[WARN]  {label}" + (f"  --  {detail}" if detail else ""))

    def summary(self) -> int:
        print("-" * 80)
        print(f"SUMMARY: ok={self.ok}  fail={self.fail}  warn={self.warn}")
        return 0 if self.fail == 0 else 1


# ----------------------------- AST utilities -----------------------------

def _load_module(path: Path) -> ast.Module:
    return ast.parse(path.read_text(), filename=str(path))


def _find_class(module: ast.Module, name: str) -> ast.ClassDef | None:
    for node in ast.walk(module):
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    return None


def _find_assignments_in_class(cls: ast.ClassDef) -> dict[str, ast.AST]:
    """Return name -> RHS for `self.NAME = ...` lines inside the class' __init__."""
    out: dict[str, ast.AST] = {}
    for fn in cls.body:
        if not isinstance(fn, ast.FunctionDef) or fn.name != "__init__":
            continue
        for stmt in ast.walk(fn):
            if isinstance(stmt, ast.Assign):
                for tgt in stmt.targets:
                    if (
                        isinstance(tgt, ast.Attribute)
                        and isinstance(tgt.value, ast.Name)
                        and tgt.value.id == "self"
                    ):
                        out[tgt.attr] = stmt.value
    return out


def _literal(node: ast.AST):
    """ast.literal_eval that tolerates numpy.array(...) wrappers and float() casts."""
    if isinstance(node, ast.Call):
        # np.array([...], dtype=...)
        if isinstance(node.func, ast.Attribute) and node.func.attr == "array":
            return _literal(node.args[0])
        # np.zeros(...) etc.
        if isinstance(node.func, ast.Attribute) and node.func.attr in {"zeros", "ones"}:
            return None
        # float(...) / int(...)
        if isinstance(node.func, ast.Name) and node.func.id in {"float", "int"}:
            return _literal(node.args[0])
    return ast.literal_eval(node)


# ----------------------------- amp.py extraction -----------------------------

def extract_amp_constants() -> dict:
    mod = _load_module(AMP_PY)
    cls = _find_class(mod, "HumanoidGaitPolicyLite")
    assert cls is not None, "HumanoidGaitPolicyLite not found in amp.py"
    fields = _find_assignments_in_class(cls)

    out: dict = {}
    for key in [
        "action_scale", "kps", "kds", "default_dof_pos",
        "mujoco_to_isaac_idx", "isaac_to_mujoco_idx",
        "num_actions", "num_obs", "obs_history_len", "single_obs_dim",
    ]:
        if key in fields:
            try:
                out[key] = _literal(fields[key])
            except Exception:
                # not a literal -- compute from text (e.g. 3 + 3 + 3 + self.num_actions*3)
                out[key] = ast.unparse(fields[key])
    return out


# ----------------------------- amp.py mujoco_to_isaac_idx comments -----------------------------

JOINT_NAME_RE = re.compile(r"'([a-zA-Z_]+_joint)'")

def extract_amp_joint_order_from_comments(var_name: str) -> list[str]:
    """Extract the 29 joint names commented inline next to entries of `var_name`.

    For amp.py:
      `mujoco_to_isaac_idx`  comments give us the POLICY/ONNX order (position 0 = l_shoulder_y, ...)
      `isaac_to_mujoco_idx`  comments give us the HW/bxi order        (position 0 = waist_y, ...)
    """
    text = AMP_PY.read_text().splitlines()
    # find `self.<var_name> = [`
    start = None
    for i, line in enumerate(text):
        if f"self.{var_name}" in line and "= [" in line:
            start = i
            break
    if start is None:
        return []
    names: list[str] = []
    for line in text[start + 1 :]:
        if "]" in line and "[" not in line and not names:
            continue
        if line.strip().startswith("]"):
            break
        # value with inline comment: "    15,    # 'l_shoulder_y_joint', 0"
        m = JOINT_NAME_RE.search(line)
        if m is None:
            # try unquoted form: '#  "waist_y_joint",'
            m = re.search(r'"([a-zA-Z_]+_joint)"', line)
        if m is not None:
            names.append(m.group(1))
    return names


# ----------------------------- bxi_example_demo.py joint_name tuple -----------------------------

def extract_bxi_joint_name_tuple() -> list[str]:
    text = DEMO_PY.read_text()
    # find first `joint_name = (` at module scope (NOT inside a comment block)
    m = re.search(r"^joint_name\s*=\s*\(([\s\S]*?)\)\s*\n", text, re.MULTILINE)
    if not m:
        raise RuntimeError("Could not find `joint_name = (...)` in bxi_example_demo.py")
    body = m.group(1)
    names = re.findall(r'"([a-zA-Z_]+_joint)"', body)
    return names


# ----------------------------- velocity_env_cfg.py -----------------------------

def extract_env_cfg() -> dict:
    mod = _load_module(ENV_CFG)
    out: dict = {}

    # ActionsCfg.JointPositionAction.scale  -- nested classdef structure
    actions_cls = _find_class(mod, "ActionsCfg")
    assert actions_cls is not None
    for stmt in actions_cls.body:
        if isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Call):
            call = stmt.value
            for kw in call.keywords:
                if kw.arg == "scale":
                    out["action_scale_dict"] = _literal(kw.value)
                if kw.arg == "use_default_offset":
                    out["use_default_offset"] = ast.literal_eval(kw.value)

    # ObservationsCfg.PolicyCfg term order + history_length
    obs_cls = _find_class(mod, "ObservationsCfg")
    assert obs_cls is not None
    policy_cls = None
    for sub in obs_cls.body:
        if isinstance(sub, ast.ClassDef) and sub.name == "PolicyCfg":
            policy_cls = sub
            break
    assert policy_cls is not None
    obs_terms: list[str] = []
    for stmt in policy_cls.body:
        # PolicyCfg uses plain Assign (`base_ang_vel = ObsTerm(...)`), not AnnAssign
        if isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Call):
            for tgt in stmt.targets:
                if isinstance(tgt, ast.Name):
                    # Only count ObsTerm calls (skip other assignments in the class body)
                    callee = stmt.value.func
                    callee_name = (
                        callee.id if isinstance(callee, ast.Name)
                        else callee.attr if isinstance(callee, ast.Attribute)
                        else ""
                    )
                    if callee_name == "ObsTerm":
                        obs_terms.append(tgt.id)
    out["policy_obs_terms"] = obs_terms

    # history_length lives inside PolicyCfg.__post_init__
    history_length = None
    enable_corruption = None
    concatenate_terms = None
    for fn in policy_cls.body:
        if isinstance(fn, ast.FunctionDef) and fn.name == "__post_init__":
            for stmt in ast.walk(fn):
                if isinstance(stmt, ast.Assign):
                    for tgt in stmt.targets:
                        if isinstance(tgt, ast.Attribute) and isinstance(tgt.value, ast.Name) and tgt.value.id == "self":
                            try:
                                val = ast.literal_eval(stmt.value)
                            except Exception:
                                continue
                            if tgt.attr == "history_length":
                                history_length = val
                            elif tgt.attr == "enable_corruption":
                                enable_corruption = val
                            elif tgt.attr == "concatenate_terms":
                                concatenate_terms = val
    out["history_length"] = history_length
    out["enable_corruption"] = enable_corruption
    out["concatenate_terms"] = concatenate_terms

    # sim.dt and decimation inside RobotEnvCfg.__post_init__
    env_cls = _find_class(mod, "RobotEnvCfg")
    decimation = None
    sim_dt = None
    if env_cls is not None:
        for fn in env_cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name == "__post_init__":
                for stmt in ast.walk(fn):
                    if isinstance(stmt, ast.Assign):
                        for tgt in stmt.targets:
                            if isinstance(tgt, ast.Attribute):
                                # self.decimation = 4
                                if (
                                    isinstance(tgt.value, ast.Name)
                                    and tgt.value.id == "self"
                                    and tgt.attr == "decimation"
                                ):
                                    decimation = ast.literal_eval(stmt.value)
                                # self.sim.dt = 0.005
                                if (
                                    isinstance(tgt.value, ast.Attribute)
                                    and tgt.value.attr == "sim"
                                    and tgt.attr == "dt"
                                ):
                                    sim_dt = ast.literal_eval(stmt.value)
    out["decimation"] = decimation
    out["sim_dt"] = sim_dt
    return out


# ----------------------------- unitree.py ELF3_29DOF_CFG -----------------------------

def extract_robot_cfg() -> dict:
    mod = _load_module(ROBOT_CFG)
    out: dict = {}

    # Find module-level assignment `ELF3_29DOF_CFG = UnitreeArticulationCfg(...)`
    target_call = None
    for stmt in mod.body:
        if (
            isinstance(stmt, ast.Assign)
            and len(stmt.targets) == 1
            and isinstance(stmt.targets[0], ast.Name)
            and stmt.targets[0].id == "ELF3_29DOF_CFG"
            and isinstance(stmt.value, ast.Call)
        ):
            target_call = stmt.value
            break
    assert target_call is not None, "ELF3_29DOF_CFG not found in unitree.py"

    # init_state.joint_pos
    joint_pos: dict | None = None
    actuators: dict | None = None
    for kw in target_call.keywords:
        if kw.arg == "init_state" and isinstance(kw.value, ast.Call):
            for k in kw.value.keywords:
                if k.arg == "joint_pos":
                    joint_pos = _literal(k.value)
        if kw.arg == "actuators":
            actuators = _literal(kw.value) if False else None  # complex, parse manually
            # collect (group_name -> raw Call) so we can pluck stiffness/damping dicts later
            actuators_calls: dict[str, ast.Call] = {}
            if isinstance(kw.value, ast.Dict):
                for k_node, v_node in zip(kw.value.keys, kw.value.values):
                    if isinstance(k_node, ast.Constant) and isinstance(v_node, ast.Call):
                        actuators_calls[k_node.value] = v_node
            out["_actuators_calls"] = actuators_calls
    out["init_joint_pos"] = joint_pos

    # spawn -> UsdFileCfg(usd_path=...)
    usd_path = None
    for kw in target_call.keywords:
        if kw.arg == "spawn" and isinstance(kw.value, ast.Call):
            for k in kw.value.keywords:
                if k.arg == "usd_path":
                    # Could be f-string
                    if isinstance(k.value, ast.JoinedStr):
                        usd_path = "".join(
                            part.value if isinstance(part, ast.Constant) else "<expr>"
                            for part in k.value.values
                        )
                    elif isinstance(k.value, ast.Constant):
                        usd_path = k.value.value
    out["usd_path"] = usd_path

    return out


def collect_robot_stiffness_damping(actuators_calls: dict[str, ast.Call]) -> tuple[dict[str, float], dict[str, float]]:
    """Walk every ImplicitActuatorCfg call to extract per-joint stiffness/damping dicts."""
    stiffness_by_joint: dict[str, float] = {}
    damping_by_joint: dict[str, float] = {}
    for group_name, call in actuators_calls.items():
        for kw in call.keywords:
            if kw.arg in ("stiffness", "damping") and isinstance(kw.value, ast.Dict):
                d = _literal(kw.value)
                target = stiffness_by_joint if kw.arg == "stiffness" else damping_by_joint
                # keys may be joint_name regex; for our purposes the unitree.py uses concrete names
                # like ".*_hip_y_joint" -- expand against the known 29 joint names below.
                for pattern, value in d.items():
                    target[pattern] = value
    return stiffness_by_joint, damping_by_joint


def expand_joint_regex_map(joint_map: dict[str, float], all_joint_names: list[str]) -> dict[str, float]:
    out: dict[str, float] = {}
    for jn in all_joint_names:
        # try exact first
        if jn in joint_map:
            out[jn] = joint_map[jn]
            continue
        # try regex with `.*` -> match anywhere
        matched = []
        for pat, val in joint_map.items():
            if "." in pat or "*" in pat:
                try:
                    if re.fullmatch(pat, jn):
                        matched.append((pat, val))
                except re.error:
                    continue
        if len(matched) == 1:
            out[jn] = matched[0][1]
        elif len(matched) > 1:
            # last one wins (legs/feet may both match)
            out[jn] = matched[-1][1]
    return out


# ----------------------------- export_elf3_onnx.py -----------------------------

def extract_export_assertions() -> dict:
    text = EXPORT_SCRIPT.read_text()
    out: dict = {}
    out["asserts_input_obs"] = "in_info.name == \"obs\"" in text or "in_info.name == 'obs'" in text
    out["asserts_output_actions"] = "out_info.name == \"actions\"" in text or "out_info.name == 'actions'" in text
    m = re.search(r"expected_obs_dim[\s\S]*?default\s*=\s*(\d+)", text)
    out["expected_obs_dim_default"] = int(m.group(1)) if m else None
    out["opset"] = 11 if "opset_version=11" in text else None
    return out


# ----------------------------- main -----------------------------

def main() -> int:
    r = Report()

    print("=" * 80)
    print("Elf3 deployment contract -- STATIC verification")
    print("=" * 80)
    print()

    # ---- AMP CONSTANTS ----
    amp = extract_amp_constants()
    r.info("amp.py num_actions", str(amp.get("num_actions")))
    r.info("amp.py num_obs", str(amp.get("num_obs")))
    r.info("amp.py obs_history_len", str(amp.get("obs_history_len")))
    r.info("amp.py single_obs_dim", str(amp.get("single_obs_dim")))

    r.check("amp.action_scale length == 29", len(amp["action_scale"]) == 29, str(len(amp["action_scale"])))
    r.check("amp.kps length == 29", len(amp["kps"]) == 29, str(len(amp["kps"])))
    r.check("amp.kds length == 29", len(amp["kds"]) == 29, str(len(amp["kds"])))
    r.check("amp.default_dof_pos length == 29", len(amp["default_dof_pos"]) == 29, str(len(amp["default_dof_pos"])))
    r.check("amp.num_actions == 29", amp["num_actions"] == 29)
    r.check("amp.num_obs == 960", amp["num_obs"] == 960)
    r.check("amp.obs_history_len == 10", amp["obs_history_len"] == 10)

    mti = amp["mujoco_to_isaac_idx"]
    itm = amp["isaac_to_mujoco_idx"]
    r.check("amp.mujoco_to_isaac_idx length == 29", len(mti) == 29)
    r.check("amp.isaac_to_mujoco_idx length == 29", len(itm) == 29)
    r.check("permutation: mujoco_to_isaac_idx is a permutation of 0..28",
            sorted(mti) == list(range(29)))
    r.check("permutation: isaac_to_mujoco_idx is a permutation of 0..28",
            sorted(itm) == list(range(29)))
    # Inverse: mti and itm should be each other's inverse
    is_inverse = all(itm[mti[i]] == i for i in range(29)) and all(mti[itm[i]] == i for i in range(29))
    r.check("mujoco_to_isaac_idx and isaac_to_mujoco_idx are inverse permutations", is_inverse)

    # ---- AMP JOINT NAMES FROM COMMENTS ----
    policy_order = extract_amp_joint_order_from_comments("mujoco_to_isaac_idx")
    hw_order = extract_amp_joint_order_from_comments("isaac_to_mujoco_idx")
    r.check("amp.py mujoco_to_isaac_idx comments yield 29 joint names",
            len(policy_order) == 29, str(len(policy_order)))
    r.check("amp.py isaac_to_mujoco_idx comments yield 29 joint names",
            len(hw_order) == 29, str(len(hw_order)))

    print()
    print("  amp.py says POLICY/ONNX order  (mujoco_to_isaac_idx comments):")
    for i, n in enumerate(policy_order):
        print(f"    [{i:>2}] {n}")
    print()
    print("  amp.py says HW/bxi order  (isaac_to_mujoco_idx comments):")
    for i, n in enumerate(hw_order):
        print(f"    [{i:>2}] {n}")
    print()

    # ---- BXI joint_name TUPLE ----
    bxi_hw = extract_bxi_joint_name_tuple()
    r.check("bxi_example_demo.py joint_name tuple has 29 entries",
            len(bxi_hw) == 29, str(len(bxi_hw)))
    r.check("bxi joint_name tuple == amp.py isaac_to_mujoco_idx comment order",
            bxi_hw == hw_order, "(should be identical -- both are the HW order)")

    # ---- VELOCITY_ENV_CFG ----
    env = extract_env_cfg()
    r.info("velocity_env_cfg.py policy obs terms", " | ".join(env["policy_obs_terms"]))
    expected_obs = ["base_ang_vel", "projected_gravity", "velocity_commands",
                    "joint_pos_rel", "joint_vel_rel", "last_action"]
    r.check("PolicyCfg term order matches amp.py single_obs layout",
            env["policy_obs_terms"] == expected_obs,
            f"expected {expected_obs}")
    r.check("PolicyCfg history_length == 10", env["history_length"] == 10, str(env["history_length"]))
    r.check("PolicyCfg enable_corruption is True", env["enable_corruption"] is True)
    r.check("PolicyCfg concatenate_terms is True", env["concatenate_terms"] is True)

    r.check("ActionsCfg.scale has 29 entries", len(env["action_scale_dict"]) == 29, str(len(env["action_scale_dict"])))
    r.check("ActionsCfg.use_default_offset == True", env["use_default_offset"] is True)

    # Build a length-29 list from action_scale_dict in policy_order (mujoco_to_isaac comments),
    # and compare to amp.action_scale (which is in policy/ONNX order)
    if len(policy_order) == 29 and len(env["action_scale_dict"]) == 29:
        env_scale_ordered = [env["action_scale_dict"].get(jn) for jn in policy_order]
        amp_scale = list(amp["action_scale"])
        match = all(
            (a is not None) and abs(float(a) - float(b)) < 1e-9
            for a, b in zip(env_scale_ordered, amp_scale)
        )
        r.check("ActionsCfg.scale matches amp.action_scale per joint (POLICY order)", match)
        if not match:
            for i, jn in enumerate(policy_order):
                a, b = env_scale_ordered[i], amp_scale[i]
                if a is None or abs(float(a) - float(b)) > 1e-9:
                    print(f"    diff [{i:>2}] {jn}: env={a}  amp={b}")

    # decimation/sim.dt -> 50 Hz
    r.check("sim.dt == 0.005", env["sim_dt"] == 0.005, str(env["sim_dt"]))
    r.check("decimation == 4", env["decimation"] == 4, str(env["decimation"]))
    if env["sim_dt"] and env["decimation"]:
        ctrl_dt = env["sim_dt"] * env["decimation"]
        r.check("control dt == 0.02 (= amp.py timer 50 Hz)", abs(ctrl_dt - 0.02) < 1e-9, f"{ctrl_dt}")

    # ---- ROBOT_CFG ----
    rob = extract_robot_cfg()
    r.info("ELF3_29DOF_CFG.usd_path", str(rob["usd_path"]))
    r.check("ELF3_29DOF_CFG.init_state.joint_pos has 29 entries",
            len(rob["init_joint_pos"]) == 29, str(len(rob["init_joint_pos"])))

    # default_dof_pos comparison: amp.default_dof_pos is in HW order
    if len(hw_order) == 29:
        rob_default_hw = [rob["init_joint_pos"].get(jn) for jn in hw_order]
        amp_default = list(amp["default_dof_pos"])
        match = all(
            (a is not None) and abs(float(a) - float(b)) < 1e-6
            for a, b in zip(rob_default_hw, amp_default)
        )
        r.check("init_state.joint_pos matches amp.default_dof_pos per joint (HW order)", match)
        if not match:
            for i, jn in enumerate(hw_order):
                a, b = rob_default_hw[i], amp_default[i]
                if a is None or abs(float(a) - float(b)) > 1e-6:
                    print(f"    diff [{i:>2}] {jn}: robot={a}  amp={b}")

    # stiffness/damping per joint
    actuators_calls = rob.get("_actuators_calls", {})
    if actuators_calls:
        stiff, damp = collect_robot_stiffness_damping(actuators_calls)
        all_joints = hw_order if len(hw_order) == 29 else policy_order
        stiff_full = expand_joint_regex_map(stiff, all_joints)
        damp_full = expand_joint_regex_map(damp, all_joints)

        # build amp kp/kd lookup in HW order
        amp_kps_hw = list(amp["kps"])
        amp_kds_hw = list(amp["kds"])
        kp_match = all(
            (jn in stiff_full) and abs(float(stiff_full[jn]) - float(amp_kps_hw[i])) < 1e-3
            for i, jn in enumerate(hw_order)
        )
        kd_match = all(
            (jn in damp_full) and abs(float(damp_full[jn]) - float(amp_kds_hw[i])) < 1e-3
            for i, jn in enumerate(hw_order)
        )
        r.check("actuator stiffness matches amp.kps per joint (HW order)", kp_match)
        if not kp_match:
            for i, jn in enumerate(hw_order):
                rv, av = stiff_full.get(jn), amp_kps_hw[i]
                if rv is None or abs(float(rv) - float(av)) > 1e-3:
                    print(f"    diff kp [{i:>2}] {jn}: robot={rv}  amp={av}")
        r.check("actuator damping matches amp.kds per joint (HW order)", kd_match)
        if not kd_match:
            for i, jn in enumerate(hw_order):
                rv, av = damp_full.get(jn), amp_kds_hw[i]
                if rv is None or abs(float(rv) - float(av)) > 1e-3:
                    print(f"    diff kd [{i:>2}] {jn}: robot={rv}  amp={av}")

    # ---- BODY-SYMMETRY SANITY CHECK ----
    # amp.action_scale should be L/R symmetric when read in the correct order.
    # POLICY order = 0 asymmetric pairs (vendor design), HW order = many.
    if len(amp["action_scale"]) == 29 and len(policy_order) == 29 and len(hw_order) == 29:
        pairs = [
            "shoulder_y", "shoulder_x", "shoulder_z", "elbow_y",
            "hip_y", "hip_x", "hip_z",
            "wrist_x", "wrist_y", "wrist_z",
            "knee_y", "ankle_y", "ankle_x",
        ]
        d_pol = dict(zip([n.removesuffix("_joint") for n in policy_order], amp["action_scale"]))
        d_hw = dict(zip([n.removesuffix("_joint") for n in hw_order], amp["action_scale"]))
        asym_pol = sum(1 for p in pairs if abs(d_pol["l_" + p] - d_pol["r_" + p]) > 1e-9)
        asym_hw = sum(1 for p in pairs if abs(d_hw["l_" + p] - d_hw["r_" + p]) > 1e-9)
        r.info("amp.action_scale L/R-asymmetric pairs read as POLICY order", f"{asym_pol}/13")
        r.info("amp.action_scale L/R-asymmetric pairs read as HW order",     f"{asym_hw}/13")
        # The true ordering is the one where L/R pairs are symmetric.
        r.check("body-symmetry test: amp.action_scale must read as POLICY order, not HW",
                asym_pol == 0 and asym_hw > 0,
                f"asym_pol={asym_pol}, asym_hw={asym_hw} -- "
                "this confirms env.action_scale must be filled per joint using amp[POLICY_pos]")

    # ---- EXPORT SCRIPT ----
    exp = extract_export_assertions()
    r.check("export_elf3_onnx.py asserts ONNX input name 'obs'", exp["asserts_input_obs"])
    r.check("export_elf3_onnx.py asserts ONNX output name 'actions'", exp["asserts_output_actions"])
    r.check("export_elf3_onnx.py default expected_obs_dim == 960",
            exp["expected_obs_dim_default"] == 960, str(exp["expected_obs_dim_default"]))
    r.check("export_elf3_onnx.py uses opset 11", exp["opset"] == 11)

    print()
    return r.summary()


if __name__ == "__main__":
    sys.exit(main())
