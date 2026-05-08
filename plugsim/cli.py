"""PlugSim CLI entry point."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent
PLUGIN_BASE = WORKSPACE / "plugin"
SCENARIO_BASE = WORKSPACE / "scenarios"


def _load(parse_metadata, scan_plugins):
    """Discover and parse all plugins. Returns (metadata_list, dir_list)."""
    dirs = scan_plugins(PLUGIN_BASE)
    if not dirs:
        print("[INFO] No plugins found in", PLUGIN_BASE)
        return [], []
    plugins, valid_dirs = [], []
    for d in dirs:
        try:
            plugins.append(parse_metadata(d))
            valid_dirs.append(d)
        except Exception as exc:
            print(f"[WARN] Skipping {d.name}: {exc}")
    return plugins, valid_dirs


def _load_scenario(path_str: str):
    from plugsim.scenario import parse_scenario
    p = Path(path_str)
    if not p.is_absolute():
        # Try relative to scenarios/ first, then cwd
        candidate = SCENARIO_BASE / p
        p = candidate if candidate.exists() else Path.cwd() / p
    return parse_scenario(p)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_scan(_args):
    from plugsim.scanner import scan_plugins
    from plugsim.parser import parse_metadata
    plugins, dirs = _load(parse_metadata, scan_plugins)
    if not plugins:
        return
    print(f"\nDiscovered {len(plugins)} plugin(s):\n")
    for m, d in zip(plugins, dirs):
        ros = m.ros_distro or "—"
        isaac_lab = m.isaac_lab or "—"
        isaac_ep = []
        if m.isaac_entry.app: isaac_ep.append(f"app:{m.isaac_entry.app}")
        if m.isaac_entry.usd: isaac_ep.append(f"usd:{m.isaac_entry.usd}")
        ros2_ep = []
        if m.ros2_entry.launch: ros2_ep.append(f"launch:{m.ros2_entry.launch}")
        if m.ros2_entry.workspace: ros2_ep.append(f"ws:{m.ros2_entry.workspace}")
        print(
            f"  [{m.plugin_type:11}]  {m.name:<35} v{m.version:<8} "
            f"ros:{ros}  isaac-lab:{isaac_lab}"
        )
        if isaac_ep:
            print(f"               isaac: {' | '.join(isaac_ep)}")
        if ros2_ep:
            print(f"               ros2:  {' | '.join(ros2_ep)}")
    print()


def cmd_validate(args):
    from plugsim.scanner import scan_plugins
    from plugsim.parser import parse_metadata, validate_compatibility
    plugins, _ = _load(parse_metadata, scan_plugins)

    if args.scenario:
        from plugsim.scenario import validate_scenario
        scenario = _load_scenario(args.scenario)
        print(f"\nValidating scenario '{scenario.name}' ...")
        errors = validate_scenario(scenario, plugins)
        # Also run plugin-level compatibility check on referenced plugins
        referenced = {p.name for p in plugins
                      if p.name == scenario.world
                      or any(r.plugin == p.name for r in scenario.robots)}
        compat_errors = validate_compatibility([p for p in plugins if p.name in referenced])
        errors += compat_errors
    else:
        print(f"\nValidating all {len(plugins)} plugin(s) ...")
        errors = validate_compatibility(plugins)

    if errors:
        print(f"\n[FAIL] {len(errors)} issue(s):\n")
        for e in errors:
            print(f"  x  {e}")
        sys.exit(1)
    print(f"\n[OK]  All checks passed.\n")


def cmd_up(args):
    from plugsim.scanner import scan_plugins
    from plugsim.parser import parse_metadata, validate_compatibility
    from plugsim.launcher import up

    plugins, _ = _load(parse_metadata, scan_plugins)

    if args.scenario:
        from plugsim.scenario import parse_scenario, validate_scenario
        scenario = _load_scenario(args.scenario)
        errors = validate_scenario(scenario, plugins)
        if errors:
            print("[FAIL] Scenario validation failed:")
            for e in errors:
                print(f"  x  {e}")
            sys.exit(1)
        ros2_distro = args.ros_distro or _infer_ros2_distro(scenario, plugins)
        print(f"\nStarting scenario '{scenario.name}' (ROS2: {ros2_distro}) ...")
        _print_scenario_plan(scenario, plugins)
        up(WORKSPACE, PLUGIN_BASE, scenario=scenario, target=args.target, ros2_distro=ros2_distro)

    elif args.world or args.robot:
        # Quick shorthand: --world X --robot Y [--robot Z ...]
        from plugsim.schema import Scenario, RobotInstance
        robots = [RobotInstance(plugin=r, instance=r) for r in (args.robot or [])]
        scenario = Scenario(name="adhoc", world=args.world or "", robots=robots)
        errors = validate_scenario(scenario, plugins)
        if errors:
            print("[FAIL] Validation failed:")
            for e in errors:
                print(f"  x  {e}")
            sys.exit(1)
        ros2_distro = args.ros_distro or _infer_ros2_distro(scenario, plugins)
        _print_scenario_plan(scenario, plugins)
        up(WORKSPACE, PLUGIN_BASE, scenario=scenario, target=args.target, ros2_distro=ros2_distro)

    else:
        print("[ERROR] Specify a scenario: --scenario <file>")
        print("        Or use shorthand:  --world <name> [--robot <name> ...]")
        sys.exit(1)

    print("\n[OK]  Containers are up.\n")


def _print_scenario_plan(scenario, plugins):
    by_name = {p.name: p for p in plugins}
    print()
    if scenario.world:
        w = by_name.get(scenario.world)
        print(f"  World  : {scenario.world}"
              + (f"  (app: {w.isaac_entry.app})" if w and w.isaac_entry.app else ""))
    for ri in scenario.robots:
        r = by_name.get(ri.plugin)
        launch = r.ros2_entry.launch if r else "—"
        print(f"  Robot  : {ri.plugin} → instance={ri.instance}  ns={ri.namespace}"
              + (f"\n           launch: {launch}" if launch else ""))
    print()


def cmd_down(args):
    from plugsim.launcher import down
    target = args.target or "both"
    print(f"\nStopping {target} container(s)...")
    down(target)
    print(f"\n[OK]  {target} container(s) stopped.\n")


def cmd_shell(args):
    from plugsim.launcher import shell
    shell(target=args.target or "isaac")


def cmd_info(args):
    from plugsim.scanner import scan_plugins
    from plugsim.parser import parse_metadata
    plugins, dirs = _load(parse_metadata, scan_plugins)
    for m, d in zip(plugins, dirs):
        if m.name == args.plugin:
            print(f"\n{'='*52}")
            print(f"  {m.name}  (v{m.version})")
            print(f"{'='*52}")
            print(f"  Type        : {m.plugin_type}")
            print(f"  Description : {m.description}")
            print(f"  Author      : {m.author or '—'}")
            print(f"  License     : {m.license or '—'}")
            print(f"  Repository  : {m.repository or '—'}")
            print(f"\n  Compatibility")
            print(f"    Isaac Lab  : {m.isaac_lab or '—'}")
            print(f"    ROS distro : {m.ros_distro or '—'}")
            print(f"\n  Isaac Entry")
            print(f"    USD    : {m.isaac_entry.usd or '—'}")
            print(f"    App    : {m.isaac_entry.app or '—'}")
            print(f"\n  ROS2 Entry")
            print(f"    Workspace : {m.ros2_entry.workspace or '—'}")
            print(f"    Launch    : {m.ros2_entry.launch or '—'}")
            print(f"    Config    : {m.ros2_entry.config or '—'}")
            if m.ros2_entry.launch_args:
                for k, v in m.ros2_entry.launch_args.items():
                    print(f"    Arg {k:<10}: {v}")
            if m.ros2_interface.namespace or m.ros2_interface.publishes or m.ros2_interface.subscribes:
                print(f"\n  ROS2 Interface  (namespace: {m.ros2_interface.namespace or '—'})")
                for t in m.ros2_interface.publishes:
                    print(f"    pub  {t.topic}  [{t.msg_type}]")
                for t in m.ros2_interface.subscribes:
                    print(f"    sub  {t.topic}  [{t.msg_type}]")
                for a in m.ros2_interface.action_servers:
                    print(f"    act  {a.name}  [{a.action_type}]")
            if m.dep_plugins:
                print(f"\n  Depends on  : {', '.join(m.dep_plugins)}")
            print(f"\n  Directory   : {d}\n")
            return
    print(f"[ERROR] Plugin '{args.plugin}' not found.")
    sys.exit(1)


def _infer_ros2_distro(scenario, plugins) -> str:
    """Pick ros_distro from robot plugin metadata. Falls back to 'jazzy'."""
    plugin_map = {p.name: p for p in plugins}
    distros = {
        plugin_map[ri.plugin].ros_distro
        for ri in scenario.robots
        if ri.plugin in plugin_map and plugin_map[ri.plugin].ros_distro
    }
    if len(distros) == 1:
        return distros.pop()
    if len(distros) > 1:
        print(f"[WARN] Robot plugins declare mixed ros_distros {distros}, defaulting to jazzy")
    return "jazzy"


def cmd_setup(args):
    from plugsim.launcher import setup
    setup(WORKSPACE, ros2_distro=args.ros_distro)


def cmd_init(_args):
    """Interactive scaffold for a new plugin."""
    print("\n=== New Plugin Scaffold ===\n")
    plugin_type = input("Plugin type [environment/robot/asset]: ").strip()
    if plugin_type not in {"environment", "robot", "asset"}:
        print("[ERROR] Invalid plugin type. Choose: environment, robot, asset")
        sys.exit(1)
    name = input("Plugin name: ").strip()
    if not name:
        print("[ERROR] Name cannot be empty.")
        sys.exit(1)
    version = input("Version [1.0.0]: ").strip() or "1.0.0"
    description = input("Description: ").strip()

    target = PLUGIN_BASE / name
    if target.exists():
        print(f"[ERROR] {target} already exists.")
        sys.exit(1)

    (target / "assets").mkdir(parents=True)

    if plugin_type == "environment":
        entry_block = (
            "isaac_entry:\n"
            "  usd: assets/scene.usd\n"
            "  app: app.py\n"
        )
    elif plugin_type == "robot":
        entry_block = (
            "isaac_entry:\n"
            "  usd: assets/robot.usd\n"
            "  app: null\n"
            "\n"
            "ros2_entry:\n"
            "  workspace: null\n"
            "  launch: launch/robot_bringup.launch.py\n"
            "  launch_args: {}\n"
            "\n"
            "ros2_interface:\n"
            "  namespace: /robot\n"
            "  publishes:\n"
            "    - topic: joint_states\n"
            "      type: sensor_msgs/JointState\n"
            "  subscribes: []\n"
            "  action_servers: []\n"
        )
    else:  # asset
        entry_block = (
            "isaac_entry:\n"
            "  usd: assets/object.usd\n"
        )

    (target / "METADATA.yaml").write_text(
        f'schema_version: "2.0"\n'
        f"plugin_type: {plugin_type}\n"
        f"name: {name}\n"
        f"version: {version}\n"
        f'description: "{description}"\n'
        f"\n"
        f"compatibility:\n"
        f"  isaac_lab: \">=2.0.0\"\n"
        f"  ros_distro: jazzy\n"
        f"\n"
        f"{entry_block}"
        f"\n"
        f"dep_plugins: []\n"
        f"\n"
        f'author: ""\n'
        f"license: MIT\n"
        f'repository: ""\n',
        encoding="utf-8",
    )
    (target / "README.md").write_text(f"# {name}\n\n{description}\n", encoding="utf-8")

    print(f"\n[OK]  Plugin scaffold created at {target}")
    print("      Edit METADATA.yaml and add your USD / launch files.\n")


# ---------------------------------------------------------------------------
# CLI wiring
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="PlugSim — plugin-based Isaac Sim + ROS2 environment manager",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_setup = sub.add_parser("setup", help="Build Docker images and initialise storage dirs")
    p_setup.add_argument(
        "--ros-distro", choices=["jazzy", "humble"], default="jazzy",
        help="ROS2 distro for the control container (default: jazzy)",
    )
    sub.add_parser("scan",     help="List all discovered plugins")
    p_down = sub.add_parser("down", help="Stop simulation container(s)")
    p_down.add_argument(
        "target", nargs="?", choices=["isaac", "ros2", "both"], default="both",
        help="Which container to stop (default: both)",
    )
    sub.add_parser("init",     help="Scaffold a new plugin interactively")

    p_shell = sub.add_parser("shell", help="Open a bash shell inside a running container")
    p_shell.add_argument(
        "target", nargs="?", choices=["isaac", "ros2"], default="isaac",
        help="Which container to connect to (default: isaac)",
    )

    p_val = sub.add_parser("validate", help="Check plugin and scenario compatibility")
    p_val.add_argument("--scenario", metavar="FILE",
                       help="Scenario file to validate (default: validate all plugins)")

    p_up = sub.add_parser("up", help="Start simulation container(s) with a scenario")
    p_up.add_argument(
        "target", nargs="?", choices=["isaac", "ros2", "both"], default="both",
        help="Which container to start (default: both)",
    )
    p_up.add_argument("--scenario", metavar="FILE",
                      help="Scenario file (from scenarios/ or absolute path)")
    p_up.add_argument(
        "--ros-distro", choices=["jazzy", "humble"], default=None,
        help="ROS2 distro override (default: auto-detected from robot plugins)",
    )
    p_up.add_argument("--world", metavar="PLUGIN",
                      help="Environment plugin name (shorthand)")
    p_up.add_argument("--robot", metavar="PLUGIN", action="append",
                      help="Robot plugin name (shorthand, repeatable)")

    p_info = sub.add_parser("info", help="Show details for a plugin")
    p_info.add_argument("plugin", help="Plugin name")

    args = parser.parse_args()
    {
        "setup":    cmd_setup,
        "scan":     cmd_scan,
        "validate": cmd_validate,
        "up":       cmd_up,
        "down":     cmd_down,
        "shell":    cmd_shell,
        "info":     cmd_info,
        "init":     cmd_init,
    }[args.command](args)


if __name__ == "__main__":
    main()
