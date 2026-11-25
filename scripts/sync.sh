#!/bin/bash
# Save your work to GitHub and get everyone else's, in one go.
#
#   ./scripts/sync.sh "what you changed"
#
# 1. Makes every .cpp and its .v5cpp match (scripts/v5cpp_sync.py).
# 2. Commits everything you changed with your message.
# 3. Pulls the team's latest changes from GitHub on top.
# 4. Pushes the result.
# Run it before you stop working on a computer, and again when you start on
# another one (with no message it just pulls).
#
# If it stops because two people changed the same lines, fix the .cpp file
# it names, then run:   ./scripts/sync.sh --continue

set -e
cd "$(dirname "$0")/.."

git_dir=$(git rev-parse --git-dir)
in_rebase() { [ -d "$git_dir/rebase-merge" ] || [ -d "$git_dir/rebase-apply" ]; }

if [ "$1" = "--continue" ]; then
  if ! in_rebase; then
    echo "Nothing to continue: there is no clash being fixed. Run ./scripts/sync.sh"
    exit 1
  fi
  # Only the .cpp files are fixed by hand; each .v5cpp is rebuilt from its .cpp.
  for v5 in $(git diff --name-only --diff-filter=U -- '*.v5cpp'); do
    name=${v5%.v5cpp}
    if [ -f "$name.cpp" ]; then
      python3 scripts/v5cpp_sync.py --from-cpp "$name" > /dev/null
    fi
  done
  if grep -l -E '^(<<<<<<<|>>>>>>>)' $(git diff --name-only --diff-filter=U) 2>/dev/null; then
    echo "Those files above still have clash markers (<<<<<<< ======= >>>>>>>)."
    echo "Keep the lines you want, delete the markers, save, and run this again."
    exit 1
  fi
  git add -A
  GIT_EDITOR=true git rebase --continue || {
    echo "Another clash further on: fix the files it names, then run ./scripts/sync.sh --continue"
    exit 1
  }
  shift
fi

if in_rebase; then
  echo "A clash from last time is still being fixed. Fix the .cpp files marked"
  echo "as conflicted, then run:   ./scripts/sync.sh --continue"
  exit 1
fi

branch=$(git rev-parse --abbrev-ref HEAD)
echo "== Repo: $(pwd)"
echo "== Branch: $branch"

echo "== Matching .cpp and .v5cpp files"
if ! python3 scripts/v5cpp_sync.py; then
  echo
  echo "Stopped: fix the conflict above first, then run this again."
  exit 1
fi

if [ -n "$(git status --porcelain)" ]; then
  message="${1:-Update code $(date '+%Y-%m-%d %H:%M')}"
  echo "== Saving your changes: $message"
  git add -A
  git commit -q -m "$message"
else
  echo "== Nothing new to save on this computer"
fi

echo "== Getting the team's latest changes"
if ! git pull --rebase -q origin "$branch"; then
  echo
  echo "Stopped: someone else changed the same lines. Your work is saved and"
  echo "nothing was pushed. To fix it:"
  echo "  1. Open the .cpp file(s) listed above with CONFLICT in VS Code."
  echo "  2. At each clash pick Accept Current / Accept Incoming / Accept Both"
  echo "     (or edit by hand so no <<<<<<< ======= >>>>>>> lines are left). Save."
  echo "     Ignore the .v5cpp files: they are rebuilt from the .cpp."
  echo "  3. Run:   ./scripts/sync.sh --continue"
  echo "Or ask Claude to sort it out."
  exit 1
fi

# The pull may have brought new code in; check the pairs still match.
python3 scripts/v5cpp_sync.py --check > /dev/null || {
  echo "Warning: some .cpp / .v5cpp pairs differ after pulling."
  echo "Run: python3 scripts/v5cpp_sync.py --check   to see which."
}

if [ -n "$(git log "origin/$branch..HEAD" --oneline 2>/dev/null)" ]; then
  echo "== Sending your changes to GitHub"
  git push -q origin "$branch"
fi

echo "== Done. This computer and GitHub now match:"
git log --oneline -3
