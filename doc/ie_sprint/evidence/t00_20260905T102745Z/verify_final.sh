#!/usr/bin/env bash
set -u

repo=/home/dev/ws_uwb_ie/src/uwb-imu-fusion
cd "$repo" || exit 1

echo "UTC=$(date -u +%FT%TZ)"
echo "PWD=$PWD"
echo "BRANCH=$(git branch --show-current)"
echo "HEAD=$(git rev-parse HEAD)"

echo FROZEN_HASHES
sha256sum doc/v2/paper_structure.tex doc/v2/v2_roadmap.md

echo SOURCE_CONFIG_DIFF_CHECK
git diff --quiet HEAD -- CMakeLists.txt package.xml src include tools simulator config launch test msg
source_diff_exit=$?
echo "git_diff_quiet_exit=$source_diff_exit"

echo TRACKED_DIFF_WHITESPACE_CHECK
git diff --check
tracked_check_exit=$?
echo "git_diff_check_exit=$tracked_check_exit"

echo UNTRACKED_DOC_WHITESPACE_CHECK
for doc in AGENTS.md doc/ie_sprint/STATUS.md doc/ie_sprint/REPO_AUDIT.md; do
  output_file="doc/ie_sprint/evidence/t00_20260905T102745Z/$(basename "$doc").whitespace.log"
  git diff --no-index --check /dev/null "$doc" > "$output_file" 2>&1
  no_index_exit=$?
  output_bytes=$(wc -c < "$output_file")
  echo "$doc no_index_exit=$no_index_exit whitespace_output_bytes=$output_bytes"
done

echo MARKDOWN_LOCAL_LINK_CHECK
python3 - "$repo" doc/ie_sprint/STATUS.md doc/ie_sprint/REPO_AUDIT.md <<'PY'
import pathlib
import re
import sys

repo = pathlib.Path(sys.argv[1])
missing = []
checked = 0
for item in sys.argv[2:]:
    doc = repo / item
    text = doc.read_text(encoding="utf-8")
    for raw in re.findall(r"\]\(([^)]+)\)", text):
        target = raw.strip().split()[0].strip("<>")
        if "://" in target or target.startswith(("mailto:", "#")):
            continue
        target = target.split("#", 1)[0]
        if not target:
            continue
        checked += 1
        resolved = (doc.parent / target).resolve()
        if not resolved.exists():
            missing.append((str(doc.relative_to(repo)), raw, str(resolved)))
print(f"checked={checked} missing={len(missing)}")
for row in missing:
    print("MISSING", *row, sep=" | ")
sys.exit(1 if missing else 0)
PY
link_exit=$?
echo "link_check_exit=$link_exit"

echo DOC_HASHES_AND_LINES
sha256sum AGENTS.md doc/ie_sprint/STATUS.md doc/ie_sprint/REPO_AUDIT.md
wc -l AGENTS.md doc/ie_sprint/STATUS.md doc/ie_sprint/REPO_AUDIT.md

echo FULL_GIT_STATUS
git status --short --branch --untracked-files=all

if [[ $source_diff_exit -ne 0 || $tracked_check_exit -ne 0 || $link_exit -ne 0 ]]; then
  exit 1
fi
for whitespace_log in doc/ie_sprint/evidence/t00_20260905T102745Z/*.whitespace.log; do
  [[ ! -s "$whitespace_log" ]] || exit 1
done
exit 0
