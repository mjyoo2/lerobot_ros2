#!/usr/bin/env python3
"""
SO-ARM URDF Download Script

Downloads actual URDF and mesh files from TheRobotStudio/SO-ARM100.

Usage:
    python scripts/download_so_arm_urdf.py --model so101 --output urdf/
    python scripts/download_so_arm_urdf.py --model so100 --output urdf/
"""

import argparse
import urllib.request
import json
from pathlib import Path


def download_file(url, output_path):
    """Download file"""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        print(f"  Downloading: {url}")
        urllib.request.urlretrieve(url, output_path)
        print(f"  ✓ Saved to: {output_path}")
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False


def get_github_tree(repo_owner, repo_name, path=""):
    """Get GitHub directory tree"""
    api_url = f"https://api.github.com/repos/{repo_owner}/{repo_name}/contents/{path}"

    try:
        with urllib.request.urlopen(api_url) as response:
            return json.loads(response.read())
    except Exception as e:
        print(f"  ✗ GitHub API error: {e}")
        return []


def download_so_arm_urdf(model="so101", output_dir="urdf"):
    """Download SO-ARM URDF"""

    output_dir = Path(output_dir)

    # GitHub repository info
    REPO_OWNER = "TheRobotStudio"
    REPO_NAME = "SO-ARM100"
    BRANCH = "main"

    print("="*80)
    print(f"📥 Downloading SO-ARM {model.upper()} URDF")
    print("="*80)
    print(f"Repository: https://github.com/{REPO_OWNER}/{REPO_NAME}")
    print(f"Output directory: {output_dir}")
    print()

    # 1. Download URDF file
    print("1️⃣ Downloading URDF file...")

    if model.lower() == "so101":
        # SO101 is under Simulation/SO101/
        urdf_file = "so101_new_calib.urdf"  # New calibration version
        urdf_subdir = "SO101"
    elif model.lower() == "so100":
        # SO100 is also under Simulation/SO100/
        urdf_file = "so100_new_calib.urdf"
        urdf_subdir = "SO100"
    else:
        print(f"❌ Unknown model: {model}")
        print("   Supported: so100, so101")
        return False

    urdf_url = f"https://raw.githubusercontent.com/{REPO_OWNER}/{REPO_NAME}/{BRANCH}/Simulation/{urdf_subdir}/{urdf_file}"
    urdf_path = output_dir / urdf_file

    if not download_file(urdf_url, urdf_path):
        return False

    print()

    # 2. Download mesh files (optional)
    print("2️⃣ Downloading mesh files (optional)...")
    print("   Note: Mesh files are used for visualization in RViz")

    response = input("   Download mesh files? (y/N): ")

    if response.lower() == 'y':
        # Check meshes directory with GitHub API (exists in assets directory)
        meshes_dir = f"Simulation/{urdf_subdir}/assets"
        files = get_github_tree(REPO_OWNER, REPO_NAME, meshes_dir)

        if files:
            mesh_output_dir = output_dir / "meshes"

            for file_info in files:
                if file_info['type'] == 'file' and (
                    file_info['name'].endswith('.stl') or
                    file_info['name'].endswith('.dae') or
                    file_info['name'].endswith('.obj')
                ):
                    mesh_url = file_info['download_url']
                    mesh_path = mesh_output_dir / file_info['name']
                    download_file(mesh_url, mesh_path)
        else:
            print("   ⚠️  No mesh files found or GitHub API limit reached")
            print("   You can download manually from:")
            print(f"   https://github.com/{REPO_OWNER}/{REPO_NAME}/tree/{BRANCH}/Simulation/meshes")
    else:
        print("   Skipped mesh download")

    print()
    print("="*80)
    print("✅ Download Complete!")
    print("="*80)
    print()
    print(f"URDF file: {urdf_path}")
    print()
    print("Next steps:")
    print()
    print("1. Visualize in RViz:")
    print(f"   ros2 launch urdf_tutorial display.launch.py model:={urdf_path}")
    print()
    print("2. Use with ROS2 Bridge:")
    print(f"   python scripts/run_ros2_bridge.py --config configs/robot/so101_scanned.yaml")
    print()
    print("3. Check joint names:")
    print(f"   grep 'joint name' {urdf_path}")
    print()

    # Check URDF contents
    print("📋 URDF Contents:")
    try:
        content = urdf_path.read_text()

        # Extract joint names
        import re
        joints = re.findall(r'<joint name="([^"]+)"', content)
        links = re.findall(r'<link name="([^"]+)"', content)

        print(f"   Joints ({len(joints)}):")
        for joint in joints:
            print(f"      - {joint}")

        print(f"   Links ({len(links)}):")
        for link in links[:5]:  # First 5 only
            print(f"      - {link}")
        if len(links) > 5:
            print(f"      ... and {len(links) - 5} more")

    except Exception as e:
        print(f"   ⚠️  Could not parse URDF: {e}")

    print()
    print("💡 Tip: If joint names don't match your robot config,")
    print("   you may need to update robot_bridge.py or rename joints in URDF")
    print()

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Download SO-ARM URDF",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Download SO-101 URDF
  python scripts/download_so_arm_urdf.py --model so101 --output urdf/

  # Download SO-100 URDF
  python scripts/download_so_arm_urdf.py --model so100 --output urdf/

  # Visualize after download
  ros2 launch urdf_tutorial display.launch.py model:=urdf/so_arm101.urdf
        """
    )
    parser.add_argument(
        "--model",
        type=str,
        default="so101",
        choices=["so100", "so101"],
        help="Robot model (Default: so101)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="urdf",
        help="Output directory (Default: urdf/)",
    )

    args = parser.parse_args()

    try:
        success = download_so_arm_urdf(args.model, args.output)
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n\n❌ Cancelled")
        return 1
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        raise


if __name__ == "__main__":
    import sys
    sys.exit(main())
