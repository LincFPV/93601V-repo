#!/usr/bin/env python3
"""Keep each .cpp file and its matching .v5cpp (VEXcode) file the same.

The repo keeps every program twice: Name.cpp (plain C++, easy to edit in
VS Code and to read on GitHub) and Name.v5cpp (the VEXcode V5 project, which
holds the same code inside it). Edit either one, then run this script and it
copies your change into the other.

How it decides which way to copy: it compares both files with the last
commit. If only the .cpp changed, the .cpp is copied into the .v5cpp. If only
the .v5cpp changed (edited in VEXcode), its code is copied out to the .cpp.
If both changed differently it stops and tells you, so nothing is lost.

Usage (from the repo folder):
  python3 scripts/v5cpp_sync.py            sync every pair
  python3 scripts/v5cpp_sync.py --check    only report, change nothing
  python3 scripts/v5cpp_sync.py --from-cpp TankComp     force .cpp -> .v5cpp
  python3 scripts/v5cpp_sync.py --from-v5cpp TankComp   force .v5cpp -> .cpp
"""

import json
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent


def committed(path):
    """Text of a file at the last commit, or None if it isn't committed."""
    try:
        return subprocess.run(
            ["git", "show", "HEAD:" + path.name],
            cwd=REPO, capture_output=True, text=True, check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def code_in_v5cpp(text):
    return json.loads(text)["textContent"]


def read_project(path):
    """The .v5cpp as JSON. If the file is broken (for example half-way through
    fixing a clash), fall back to the last committed copy for its settings."""
    try:
        return json.loads(path.read_text())
    except ValueError:
        old = committed(path)
        if old is None:
            raise SystemExit(f"{path.name} is not a valid VEXcode file and has no committed copy to rebuild from.")
        return json.loads(old)


def write_v5cpp(path, code):
    project = read_project(path)
    project["textContent"] = code
    path.write_text(json.dumps(project, separators=(",", ":")))


def main(args):
    check_only = "--check" in args
    force = {}
    for flag in ("--from-cpp", "--from-v5cpp"):
        if flag in args:
            i = args.index(flag)
            if i + 1 >= len(args):
                print(f"{flag} needs a program name, e.g. {flag} TankComp")
                return 2
            force[args[i + 1].removesuffix(".cpp").removesuffix(".v5cpp")] = flag

    problems = 0
    pairs = sorted(p for p in REPO.glob("*.cpp") if p.with_suffix(".v5cpp").exists())
    if force:
        pairs = [p for p in pairs if p.stem in force]   # only touch the named ones
        missing = set(force) - {p.stem for p in pairs}
        if missing:
            print("No .cpp + .v5cpp pair called: " + ", ".join(sorted(missing)))
            return 2
    for cpp in pairs:
        v5 = cpp.with_suffix(".v5cpp")
        name = cpp.stem
        cpp_now = cpp.read_text()
        if name in force and force[name] == "--from-cpp":
            v5_now = None                      # not needed, and may be broken
        else:
            try:
                v5_now = code_in_v5cpp(v5.read_text())
            except ValueError:
                problems += 1
                print(f"  {name}: {v5.name} is broken (not valid VEXcode JSON).")
                print(f"         Rebuild it from the .cpp with:")
                print(f"           python3 scripts/v5cpp_sync.py --from-cpp {name}")
                continue

        if name in force:
            if check_only:
                print(f"  {name}: would force {force[name]} (--check, nothing changed)")
            elif force[name] == "--from-cpp":
                write_v5cpp(v5, cpp_now)
                print(f"  {name}: copied {cpp.name} -> {v5.name} (forced)")
            else:
                cpp.write_text(v5_now)
                print(f"  {name}: copied {v5.name} -> {cpp.name} (forced)")
            continue

        if cpp_now == v5_now:
            print(f"  {name}: in sync")
            continue

        cpp_then = committed(cpp)
        v5_then_text = committed(v5)
        v5_then = code_in_v5cpp(v5_then_text) if v5_then_text else None
        cpp_changed = cpp_now != cpp_then
        v5_changed = v5_now != v5_then

        if cpp_changed and not v5_changed:
            if check_only:
                print(f"  {name}: OUT OF SYNC - {cpp.name} was edited, {v5.name} needs updating")
                problems += 1
            else:
                write_v5cpp(v5, cpp_now)
                print(f"  {name}: copied your {cpp.name} edits into {v5.name}")
        elif v5_changed and not cpp_changed:
            if check_only:
                print(f"  {name}: OUT OF SYNC - {v5.name} was edited, {cpp.name} needs updating")
                problems += 1
            else:
                cpp.write_text(v5_now)
                print(f"  {name}: copied your {v5.name} edits into {cpp.name}")
        else:
            problems += 1
            print(f"  {name}: CONFLICT - {cpp.name} and {v5.name} hold different code and")
            print(f"         the script can't tell which is right. Pick one and run:")
            print(f"           python3 scripts/v5cpp_sync.py --from-cpp {name}")
            print(f"           python3 scripts/v5cpp_sync.py --from-v5cpp {name}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
