#!/bin/bash
# Apply (or revert) this project's fixes to Hello Robot's stretch_mujoco and stretch_ros2,
# and to the course starter package parcs_stretch_nav2 (if it is checked out).
# See README.md "Simulator patches" for what they change.
#
#   sim/patches/apply_patches.sh                     # apply
#   sim/patches/apply_patches.sh --revert            # undo
#   sim/patches/apply_patches.sh [--revert] MUJOCO_DIR ROS2_DIR [STARTER_DIR]
#
# Defaults: ~/repos/stretch_mujoco, and stretch_ros2 and parcs_stretch_nav2_WVH checked out
# next to this repo (workspace/src/).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REVERT=""
if [ "$1" = "--revert" ]; then REVERT="-R"; shift; fi
MUJOCO_DIR="${1:-$HOME/repos/stretch_mujoco}"
ROS2_DIR="${2:-$HERE/../../../stretch_ros2}"
STARTER_DIR="${3:-$HERE/../../../parcs_stretch_nav2_WVH}"

patch_repo() {  # <repo dir> <patch file>
  local repo="$1" patch="$HERE/$2"
  if [ ! -d "$repo/.git" ]; then
    echo "error: $repo is not a git checkout (pass its path as an argument)" >&2
    exit 1
  fi
  repo="$(cd "$repo" && pwd)"
  if [ -z "$REVERT" ] && git -C "$repo" apply -R --check "$patch" 2>/dev/null; then
    echo "already applied: $2 -> $repo"
    return
  fi
  if [ -n "$REVERT" ] && ! git -C "$repo" apply -R --check "$patch" 2>/dev/null; then
    echo "not applied, nothing to revert: $2 -> $repo"
    return
  fi
  if ! git -C "$repo" apply $REVERT --check "$patch"; then
    echo "error: $2 does not apply cleanly to $repo." >&2
    echo "       It was made against the commit listed in README.md; try: git -C $repo apply --3way $patch" >&2
    exit 1
  fi
  git -C "$repo" apply $REVERT "$patch"
  if [ -n "$REVERT" ]; then echo "reverted: $2 -> $repo"; else echo "applied: $2 -> $repo"; fi
}

patch_repo "$MUJOCO_DIR" stretch_mujoco.patch
patch_repo "$ROS2_DIR" stretch_ros2.patch
if [ -d "$STARTER_DIR/.git" ]; then
  patch_repo "$STARTER_DIR" parcs_stretch_nav2.patch
else
  echo "skipped: parcs_stretch_nav2.patch ($STARTER_DIR is not checked out; see README setup)"
fi

WS="$(cd "$ROS2_DIR/../.." && pwd)"
echo
echo "Now rebuild (stretch_mujoco is installed with 'pip install -e', so it needs nothing):"
echo "  cd $WS && source /opt/ros/humble/setup.bash && colcon build --packages-select stretch_simulation parcs_stretch_nav2"
