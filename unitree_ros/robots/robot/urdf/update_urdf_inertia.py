#!/usr/bin/env python3
"""
update_urdf_inertia.py
将 SolidWorks 导出的质量属性 txt 文件中的惯量数据写入 URDF 文件。

用法：
    python update_urdf_inertia.py <inertia_txt> <input_urdf> <output_urdf>

匹配规则：
    坐标系名称与 link 名称完全一致（大小写不敏感）。
    无匹配的 link 跳过。

单位：
    从 txt 文件中自动检测，支持 千克/克、米/厘米/毫米。
    使用"由重心决定，并且对齐输出的坐标系"的惯量张量（Lxx/Lyy/Lzz…）。
    非对角线惯量元素（Lxy/Lxz/Lyz）取负后写入 URDF。

关节轴修正：
    仅将 (0 0 -1) 自动修正为 (0 0 1)。
    其他轴方向保持不变。
    fixed 关节跳过。

错误处理：
    txt 中任何字段解析失败，或 URDF 中匹配 link 的 <inertial>/<origin>/
    <mass>/<inertia> 元素缺失，均报错退出。
"""

import re
import sys
from xml.etree import ElementTree as ET

_LENGTH_UNITS = {"毫米": 1e-3, "厘米": 1e-2, "米": 1.0}
_MASS_UNITS   = {"千克": 1.0,  "克":   1e-3}


# ═══════════════════════════════════════════════════════════════════════════════
# 解析 txt
# ═══════════════════════════════════════════════════════════════════════════════

def parse_mass_props(txt_path: str) -> dict:
    with open(txt_path, encoding="utf-8") as f:
        text = f.read()

    segments = re.split(r"(?=坐标系：\s*\S+)", text)
    result = {}

    for seg in segments:
        m_cs = re.search(r"坐标系：\s*(\S+)", seg)
        if not m_cs:
            continue
        cs_name = m_cs.group(1)

        m_mass = re.search(r"质量\s*=\s*([\d.]+)\s*(千克|克)", seg)
        if not m_mass:
            raise ValueError(f"坐标系 [{cs_name}] 无法解析质量")
        mass_factor = _MASS_UNITS[m_mass.group(2)]
        mass = float(m_mass.group(1)) * mass_factor

        m_com = re.search(
            r"(?:质心|重心)\s*:\s*\(\s*(毫米|厘米|米)\s*\).*?"
            r"X\s*=\s*([-\d.]+).*?Y\s*=\s*([-\d.]+).*?Z\s*=\s*([-\d.]+)",
            seg,
            re.DOTALL,
        )
        if not m_com:
            raise ValueError(f"坐标系 [{cs_name}] 无法解析质心/重心数据")
        length_factor = _LENGTH_UNITS[m_com.group(1)]
        com = (
            float(m_com.group(2)) * length_factor,
            float(m_com.group(3)) * length_factor,
            float(m_com.group(4)) * length_factor,
        )

        m_iu = re.search(
            r"惯性张量:\s*\(\s*(千克|克)\s*\*\s*平方(米|毫米|厘米)\s*\)",
            seg
        )
        if not m_iu:
            raise ValueError(f"坐标系 [{cs_name}] 无法解析惯量单位")
        inertia_factor = _MASS_UNITS[m_iu.group(1)] * _LENGTH_UNITS[m_iu.group(2)] ** 2

        m_inertia = re.search(
            r"由重心决定，并且对齐输出的坐标系。\s*（使用(正|负)张量记数法。）.*?"
            r"Lxx\s*=\s*([-\d.]+)\s*Lxy\s*=\s*([-\d.]+)\s*Lxz\s*=\s*([-\d.]+)\s*"
            r"Lyx\s*=\s*([-\d.]+)\s*Lyy\s*=\s*([-\d.]+)\s*Lyz\s*=\s*([-\d.]+)\s*"
            r"Lzx\s*=\s*([-\d.]+)\s*Lzy\s*=\s*([-\d.]+)\s*Lzz\s*=\s*([-\d.]+)",
            seg, re.DOTALL
        )
        if not m_inertia:
            raise ValueError(f"坐标系 [{cs_name}] 无法解析惯量张量（或记数法非正/负张量）")

        off_diag_sign = -1 if m_inertia.group(1) == "正" else 1
        v = [float(m_inertia.group(i)) * inertia_factor for i in range(2, 11)]
        result[cs_name] = {
            "mass": mass,
            "com":  com,
            "inertia": {
                "ixx":  v[0], "ixy": off_diag_sign * v[1], "ixz": off_diag_sign * v[2],
                "iyy":  v[4], "iyz": off_diag_sign * v[5],
                "izz":  v[8],
            }
        }

    return result


# ═══════════════════════════════════════════════════════════════════════════════
# 名称匹配
# ═══════════════════════════════════════════════════════════════════════════════

def find_cs_for_link(link_name: str, props: dict):
    ln = link_name.lower()
    candidates = [ln]

    if ln == "pelvis_link":
        candidates.append("global_link")

    if ln.endswith("_ankle_pitch_link"):
        candidates.append(ln.replace("_ankle_pitch_link", "_ankle_link"))
    if ln.endswith("_ankle_roll_link"):
        candidates.append(ln.replace("_ankle_roll_link", "_ankle_link"))

    normalized_candidates = set(candidates)
    normalized_candidates.update(f"s_{name}" for name in candidates)

    for cs in props:
        if cs.lower() in normalized_candidates:
            return cs
    return None


# ═══════════════════════════════════════════════════════════════════════════════
# 关节轴修正：-1 → 1
# ═══════════════════════════════════════════════════════════════════════════════

def fix_negative_axes(root) -> list:
    """
    遍历所有关节，仅处理轴为 (0 0 -1) 的情况，将其改为 (0 0 1)。
    其他轴方向保持不变。
    返回被修改的关节名称列表。
    """
    fixed = []
    for joint_elem in root.iter("joint"):
        if joint_elem.get("type") == "fixed":
            continue
        axis_elem = joint_elem.find("axis")
        if axis_elem is None:
            continue
        joint_name = joint_elem.get("name")
        if joint_name is None:
            raise ValueError("URDF 中存在未命名的关节，无法处理")
        xyz_str = axis_elem.get("xyz", "")
        try:
            components = [float(v) for v in xyz_str.split()]
        except ValueError:
            raise ValueError(f"关节 [{joint_name}] 轴格式无法解析：'{xyz_str}'")
        if components == [0.0, 0.0, -1.0]:
            axis_elem.set("xyz", "0 0 1")
            fixed.append(joint_name)
    return fixed


# ═══════════════════════════════════════════════════════════════════════════════
# 更新 URDF
# ═══════════════════════════════════════════════════════════════════════════════

def update_urdf(input_urdf: str, output_urdf: str, props: dict):
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    # ── 1. 修正负轴 ───────────────────────────────────────────────────────────
    fixed_joints = fix_negative_axes(root)

    # ── 2. 替换惯量 ───────────────────────────────────────────────────────────
    updated = []
    skipped = []

    for link_elem in root.iter("link"):
        link_name = link_elem.get("name")
        cs_name   = find_cs_for_link(link_name, props)

        if cs_name is None:
            skipped.append(link_name)
            continue

        data = props[cs_name]

        inertial = link_elem.find("inertial")
        if inertial is None:
            raise ValueError(f"link [{link_name}] 缺少 <inertial> 元素")

        origin = inertial.find("origin")
        if origin is None:
            raise ValueError(f"link [{link_name}] 的 <inertial> 缺少 <origin>")
        cx, cy, cz = data["com"]
        origin.set("xyz", f"{cx:.10f} {cy:.10f} {cz:.10f}")
        origin.set("rpy", "0 0 0")

        mass_elem = inertial.find("mass")
        if mass_elem is None:
            raise ValueError(f"link [{link_name}] 的 <inertial> 缺少 <mass>")
        mass_elem.set("value", f"{data['mass']:.10f}")

        inertia_elem = inertial.find("inertia")
        if inertia_elem is None:
            raise ValueError(f"link [{link_name}] 的 <inertial> 缺少 <inertia>")
        for key, val in data["inertia"].items():
            inertia_elem.set(key, f"{val:.10f}")

        updated.append((link_name, cs_name))

    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass

    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)

    # ── 报告 ──────────────────────────────────────────────────────────────────
    print("=" * 60)
    if fixed_joints:
        print(f"[FIXED] 已修正负轴关节（{len(fixed_joints)} 个）：")
        for jn in fixed_joints:
            print(f"  {jn}")
        print()

    print(f"[OK] 已更新惯量（{len(updated)} 个 link）：")
    for ln, cs in updated:
        hint = f"  ← 匹配自坐标系 [{cs}]" if ln != cs else ""
        print(f"  {ln}{hint}")

    used_cs = {cs for _, cs in updated}
    unused_cs = [cs for cs in props if cs not in used_cs]
    if unused_cs:
        print(f"\n[UNUSED] txt 中未匹配到任何 link 的坐标系（{len(unused_cs)} 个）：")
        for cs in unused_cs:
            print(f"  {cs}")

    if skipped:
        print(f"\n[SKIPPED] 未找到匹配数据，跳过（{len(skipped)} 个 link）：")
        for ln in skipped:
            print(f"  {ln}")

    print(f"\n[OUTPUT] 输出文件：{output_urdf}")
    print("=" * 60)


# ═══════════════════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)

    txt_path, input_urdf, output_urdf = sys.argv[1], sys.argv[2], sys.argv[3]

    print(f"[INFO] 读取质量属性文件：{txt_path}")
    props = parse_mass_props(txt_path)
    print(f"    解析到 {len(props)} 个坐标系：{list(props.keys())}")

    print(f"[INFO] 读取原始 URDF：{input_urdf}")
    update_urdf(input_urdf, output_urdf, props)


if __name__ == "__main__":
    main()
