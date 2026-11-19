#!/bin/bash
# Copies your lab code from the ecn_manip package into this repository folder.
# Run it from inside the manip-lab folder:  bash collect.sh
set -e
PKG=~/ros2/src/ecn_manip
[ -d "$PKG" ] || { echo "ecn_manip not found in $PKG"; exit 1; }
mkdir -p src
for f in control.cpp robot_turret.cpp robot_kr16.cpp robot_ur10.cpp; do
    found=$(find "$PKG" -maxdepth 2 -name "$f" -not -path "*/build/*" | head -1)
    if [ -n "$found" ]; then cp "$found" src/ && echo "copied  $found"; else echo "MISSING $f"; fi
done
echo "Done. Check src/ before committing."
