#!/usr/bin/env bash
# Strip CRLF line endings from every text file in the workspace that
# Linux tools care about. Re-run after any Windows-side edit that
# introduces \r\n.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

count=0
while IFS= read -r -d '' f; do
    if file "$f" | grep -q CRLF; then
        sed -i 's/\r$//' "$f"
        echo "fixed: ${f#$ROOT/}"
        count=$((count + 1))
    fi
done < <(find "$ROOT" \
    -type f \
    \( -name '*.sh' \
       -o -name '*.py' \
       -o -name '*.yaml' \
       -o -name '*.yml' \
       -o -name '*.xml' \
       -o -name '*.launch' \
       -o -name '*.xacro' \
       -o -name '*.srdf' \
       -o -name '*.urdf' \
    \) \
    -not -path '*/build/*' \
    -not -path '*/install/*' \
    -not -path '*/log/*' \
    -not -path '*/.git/*' \
    -print0)

echo
echo "Done. $count file(s) converted."
