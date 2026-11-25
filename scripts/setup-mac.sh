#!/bin/bash
# Set up a Mac to work on the 93601V code. Safe to run again: it skips
# anything that is already done and updates the rest.
#
# On a new Mac, open Terminal and paste:
#   curl -fsSL https://raw.githubusercontent.com/LincFPV/93601V-repo/main/scripts/setup-mac.sh | bash
# or, if you already have the repo:
#   ./scripts/setup-mac.sh
#
# It installs or updates: git, the GitHub CLI (gh), VS Code and the VEX
# Robotics extension, signs you in to GitHub, and puts the repo in
# ~/Robotics/93601V-repo.

set -e
REPO_URL="https://github.com/LincFPV/93601V-repo.git"
DEST="$HOME/Robotics/93601V-repo"

if ! command -v brew >/dev/null 2>&1; then
  for b in /opt/homebrew/bin/brew /usr/local/bin/brew; do
    [ -x "$b" ] && eval "$("$b" shellenv)"
  done
fi
if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew is needed first. Install it from https://brew.sh (it asks for"
  echo "your Mac password), close and reopen Terminal, then run this again."
  exit 1
fi
export HOMEBREW_NO_ENV_HINTS=1

echo "== git and GitHub CLI"
brew install git gh >/dev/null 2>&1 || brew upgrade git gh

echo "== VS Code"
if [ -d "/Applications/Visual Studio Code.app" ] && ! brew list --cask visual-studio-code >/dev/null 2>&1; then
  echo "   already installed (it updates itself)"
else
  brew install --cask visual-studio-code >/dev/null 2>&1 || brew upgrade --cask visual-studio-code || true
fi
CODE=$(command -v code || echo "/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code")

echo "== VS Code extensions (VEX Robotics, C/C++)"
"$CODE" --install-extension vexrobotics.vexcode >/dev/null 2>&1 || true
"$CODE" --install-extension ms-vscode.cpptools >/dev/null 2>&1 || true
"$CODE" --update-extensions >/dev/null 2>&1 || true

echo "== GitHub sign-in"
if gh auth status >/dev/null 2>&1; then
  echo "   already signed in"
else
  echo "   A code will appear: press Enter, paste it in the browser, click Authorize."
  gh auth login --hostname github.com --git-protocol https --web
fi
gh auth setup-git

echo "== The repo"
if [ -d "$DEST/.git" ]; then
  git -C "$DEST" pull --rebase -q || echo "   (could not pull: run ./scripts/sync.sh in the repo to see why)"
  echo "   updated $DEST"
else
  mkdir -p "$(dirname "$DEST")"
  git clone -q "$REPO_URL" "$DEST"
  echo "   downloaded to $DEST"
fi
chmod +x "$DEST"/scripts/*.sh 2>/dev/null || true

echo
echo "All set. Open the code with:"
echo "  code \"$DEST\""
echo "Read HANDOFF.md first. When you finish working, run ./scripts/sync.sh"
