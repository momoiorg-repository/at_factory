"""Container lifecycle — two-container orchestration (Isaac + ROS2)."""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import List, Optional

from .schema import PluginMetadata, Scenario

# ---------------------------------------------------------------------------
# Container / image names
# ---------------------------------------------------------------------------

ISAAC_CONTAINER = "plugsim-isaac"
ROS2_CONTAINER  = "plugsim-ros2"
ISAAC_IMAGE     = "plugsim:isaac"
PLUGIN_MOUNT    = "/plugin"


_ROS2_UBUNTU = {"jazzy": "24.04", "humble": "22.04"}


def _ros2_image(distro: str) -> str:
    return f"plugsim:ros2-{distro}"

_ISAAC_SIM_CACHE_DIRS = [
    "cache/kit",
    "cache/ov",
    "cache/pip",
    "cache/glcache",
    "cache/computecache",
    "logs",
    "data",
    "documents",
    "config",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _run(cmd: List[str], check: bool = True) -> subprocess.CompletedProcess:
    print(f"  [RUN]  {' '.join(cmd)}")
    return subprocess.run(cmd, check=check)


def _ensure_isaac_dirs(isaac_sim: Path) -> None:
    """Pre-create Isaac Sim cache dirs so Docker doesn't make them root-owned."""
    for sub in _ISAAC_SIM_CACHE_DIRS:
        (isaac_sim / sub).mkdir(parents=True, exist_ok=True)


def _container_running(name: str) -> bool:
    result = subprocess.run(
        ["docker", "ps", "--format", "{{.Names}}"],
        capture_output=True, text=True,
    )
    return name in result.stdout.splitlines()


def _image_exists(tag: str) -> bool:
    result = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        capture_output=True, text=True,
    )
    return tag in result.stdout.splitlines()


def is_running(target: str = "isaac") -> bool:
    """Return True if the specified container (isaac | ros2 | both) is running."""
    if target == "both":
        return _container_running(ISAAC_CONTAINER) and _container_running(ROS2_CONTAINER)
    name = ISAAC_CONTAINER if target == "isaac" else ROS2_CONTAINER
    return _container_running(name)


# ---------------------------------------------------------------------------
# Up
# ---------------------------------------------------------------------------

def up(workspace: Path, plugin_base: Path, scenario: Optional[Scenario] = None,
       target: str = "both", ros2_distro: str = "jazzy") -> None:
    """Start containers.  target: isaac | ros2 | both"""
    if target in ("isaac", "both"):
        _up_isaac(workspace, plugin_base, scenario)
    if target in ("ros2", "both"):
        _up_ros2(workspace, plugin_base, scenario, ros2_distro)
    if scenario and target == "both":
        _print_next_steps(scenario, plugin_base)


def _resolve_robot_usd(plugin_base: Path, plugin_name: str) -> Optional[str]:
    """Return the container-side USD path for a robot plugin, or None if not declared."""
    meta_file = plugin_base / plugin_name / "METADATA.yaml"
    if not meta_file.exists():
        return None
    try:
        import yaml
        raw = yaml.safe_load(meta_file.read_text(encoding="utf-8"))
        usd_rel = (raw.get("isaac_entry") or {}).get("usd")
        if not usd_rel:
            return None
        # Resolve relative to the plugin dir, then express as a container path
        resolved = (plugin_base / plugin_name / usd_rel).resolve()
        rel = resolved.relative_to(plugin_base.resolve())
        return f"/plugin/{rel}"
    except Exception:
        return None


def _scenario_env(scenario: Scenario, plugin_base: Path) -> List[str]:
    robots_json = json.dumps([
        {
            "instance":  ri.instance,
            "plugin":    ri.plugin,
            "namespace": ri.namespace,
            "usd":       _resolve_robot_usd(plugin_base, ri.plugin),
            "prim_path": f"/World/{ri.instance}",
            "spawn": {
                "x":     ri.spawn.x,
                "y":     ri.spawn.y,
                "z":     ri.spawn.z,
                "roll":  ri.spawn.roll,
                "pitch": ri.spawn.pitch,
                "yaw":   ri.spawn.yaw,
            },
        }
        for ri in scenario.robots
    ])
    return [
        "-e", f"PLUGSIM_SCENARIO={scenario.name}",
        "-e", f"PLUGSIM_WORLD={scenario.world}",
        "-e", f"PLUGSIM_ROBOTS={','.join(r.instance for r in scenario.robots)}",
        "-e", f"PLUGSIM_ROBOTS_JSON={robots_json}",
    ]


def _up_isaac(workspace: Path, plugin_base: Path, scenario: Optional[Scenario]) -> None:
    if _container_running(ISAAC_CONTAINER):
        print(f"[INFO] Isaac container '{ISAAC_CONTAINER}' is already running.")
        return

    isaac_sim = workspace / "isaac-sim"
    ros_ws    = workspace / "IsaacSim-ros_workspaces"
    _ensure_isaac_dirs(isaac_sim)

    cmd = [
        "docker", "run", "--name", ISAAC_CONTAINER, "-d",
        "--runtime=nvidia", "--gpus", "all",
        "--network", "host",
        "--ipc=host",
        "--pid=host",
        "-e", "ACCEPT_EULA=Y",
        "-e", "PRIVACY_CONSENT=Y",
        "-e", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
        "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "-e", "QT_X11_NO_MITSHM=1",
        "-e", "QT_GRAPHICSSYSTEM=native",
        "-e", "QT_QPA_PLATFORM=xcb",
        "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "-e", "ROS_DOMAIN_ID=31",
        # Isaac Sim ROS2 bridge ships its own internal Jazzy libs — must be on LD_LIBRARY_PATH
        "-e", "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib",
        *(_scenario_env(scenario, plugin_base) if scenario else []),
        "-v", f"{plugin_base.resolve()}:{PLUGIN_MOUNT}:rw",
        "-v", f"{ros_ws.resolve()}:/IsaacSim-ros_workspaces:rw",
        "-v", f"{(isaac_sim / 'cache/kit').resolve()}:/isaac-sim/kit/cache:rw",
        "-v", f"{(isaac_sim / 'cache/ov').resolve()}:/root/.cache/ov:rw",
        "-v", f"{(isaac_sim / 'cache/pip').resolve()}:/root/.cache/pip:rw",
        "-v", f"{(isaac_sim / 'cache/glcache').resolve()}:/root/.cache/nvidia/GLCache:rw",
        "-v", f"{(isaac_sim / 'cache/computecache').resolve()}:/root/.nv/ComputeCache:rw",
        "-v", f"{(isaac_sim / 'logs').resolve()}:/root/.nvidia-omniverse/logs:rw",
        "-v", f"{(isaac_sim / 'data').resolve()}:/root/.local/share/ov/data:rw",
        "-v", f"{(isaac_sim / 'documents').resolve()}:/root/isaac-sim/Documents:rw",
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
        "-v", f"{Path.home() / '.Xauthority'}:/root/.Xauthority:ro",
        "--entrypoint", "/bin/bash",
        ISAAC_IMAGE,
        "-c", "tail -f /dev/null",
    ]
    _run(cmd)


def _up_ros2(workspace: Path, plugin_base: Path, scenario: Optional[Scenario],
             ros2_distro: str = "jazzy") -> None:
    if _container_running(ROS2_CONTAINER):
        print(f"[INFO] ROS2 container '{ROS2_CONTAINER}' is already running.")
        return

    image = _ros2_image(ros2_distro)
    cmd = [
        "docker", "run", "--name", ROS2_CONTAINER, "-d",
        "--network", "host",
        "--ipc=host",
        "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "-e", "ROS_DOMAIN_ID=31",
        "-e", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
        "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "-e", "QT_X11_NO_MITSHM=1",
        *(_scenario_env(scenario, plugin_base) if scenario else []),
        "-v", f"{plugin_base.resolve()}:{PLUGIN_MOUNT}:rw",
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
        "-v", f"{Path.home() / '.Xauthority'}:/root/.Xauthority:ro",
        "--entrypoint", "/bin/bash",
        image,
        "-c", "tail -f /dev/null",
    ]
    _run(cmd)


# ---------------------------------------------------------------------------
# Down
# ---------------------------------------------------------------------------

def down(target: str = "both") -> None:
    """Stop and remove containers.  target: isaac | ros2 | both"""
    names = {
        "isaac": [ISAAC_CONTAINER],
        "ros2":  [ROS2_CONTAINER],
        "both":  [ISAAC_CONTAINER, ROS2_CONTAINER],
    }.get(target, [ISAAC_CONTAINER, ROS2_CONTAINER])
    for name in names:
        _run(["docker", "stop", name], check=False)
        _run(["docker", "rm",   name], check=False)


# ---------------------------------------------------------------------------
# Shell
# ---------------------------------------------------------------------------

def shell(target: str = "isaac") -> None:
    """Open an interactive bash shell inside isaac or ros2 container."""
    name = ISAAC_CONTAINER if target == "isaac" else ROS2_CONTAINER
    if not _container_running(name):
        hint = "plugsim up" if not _container_running(ISAAC_CONTAINER) else f"plugsim up (ros2 not started)"
        print(f"[ERROR] Container '{name}' is not running. Run '{hint}' first.")
        return
    _run(["docker", "exec", "-it", name, "/bin/bash"], check=False)


# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

def setup(workspace: Path, ros2_distro: str = "jazzy") -> None:
    """Bootstrap: create cache dirs, build both Docker images."""
    _ensure_isaac_dirs(workspace / "isaac-sim")
    print("[INFO] Storage directories ready.")

    _build_image(workspace, ISAAC_IMAGE, "Dockerfile", "Isaac Sim")
    ubuntu_ver = _ROS2_UBUNTU.get(ros2_distro, "24.04")
    _build_image(workspace, _ros2_image(ros2_distro), "Dockerfile.ros2",
                 f"ROS2 {ros2_distro}", build_args={"ROS_DISTRO": ros2_distro, "UBUNTU_VERSION": ubuntu_ver})

    print(
        "\n=========================================="
        "\nSetup complete. Next steps:"
        "\n=========================================="
        "\n"
        "\n1. Start a scenario:"
        "\n   plugsim up --scenario scenarios/factory_melon.yaml"
        "\n"
        "\n2. Open a shell in Isaac container:"
        "\n   plugsim shell"
        "\n"
        "\n3. Open a shell in ROS2 container:"
        "\n   plugsim shell ros2"
        "\n"
        "\n4. List plugins:"
        "\n   plugsim scan"
        "\n=========================================="
    )


def _build_image(workspace: Path, tag: str, dockerfile: str, label: str,
                 build_args: Optional[dict] = None) -> None:
    df_path = workspace / dockerfile
    if not df_path.exists():
        print(f"[ERROR] {dockerfile} not found in {workspace}")
        return

    if _image_exists(tag):
        ans = input(f"[INFO] Image '{tag}' ({label}) already exists. Rebuild? (y/N): ").strip()
        if ans.lower() != "y":
            print(f"[INFO] Skipping {label} image build.")
            return

    print(f"[INFO] Building '{tag}' ({label}) ...")
    cmd = ["docker", "build", "-t", tag, "-f", str(df_path)]
    for k, v in (build_args or {}).items():
        cmd += ["--build-arg", f"{k}={v}"]
    cmd.append(str(workspace))
    _run(cmd)


# ---------------------------------------------------------------------------
# Next-step instructions
# ---------------------------------------------------------------------------

def _print_next_steps(scenario: Scenario, plugin_base: Path) -> None:
    """Print what to run inside each container for this scenario."""
    print("\n  ┌─ Isaac container  (plugsim shell)")
    if scenario.world:
        print(f"  │  cd /plugin/{scenario.world}")
        print(f"  │  python app.py")
    else:
        print(f"  │  (no world — start Isaac Sim manually)")

    print(f"  │")
    print(f"  └─ ROS2 container  (plugsim shell ros2)")
    for ri in scenario.robots:
        print(f"     # {ri.instance}  ns={ri.namespace}")
        print(f"     ros2 launch /plugin/{ri.plugin}/<launch_file> \\")
        print(f"       namespace:={ri.namespace.lstrip('/')}")
    print()
