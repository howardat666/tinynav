#!/usr/bin/env python3
"""Flag `self.NAME` reads that nothing in the class ever assigns.

Written after a comment rewrite silently deleted POI_ARRIVAL_RADIUS_XY_M: still
valid Python, still clean under ruff, and the node ran for 40 s before the first
POI arrived and killed it with AttributeError. Syntax and lint cannot see a
missing attribute; this can, without needing ROS or a board.

Only screams about SCREAMING_CASE names -- constants are declared in one place, so
a read with no assignment is unambiguous. Lower-case attributes are set from too
many places (setattr, loops, base classes) to judge without false positives.
"""

import ast
import sys


def check(path: str) -> list[str]:
    tree = ast.parse(open(path).read())
    out = []
    for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
        assigned, read = set(), {}
        for node in ast.walk(cls):
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if isinstance(t, ast.Name):
                        assigned.add(t.id)
                    elif isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name) and t.value.id == "self":
                        assigned.add(t.attr)
            elif isinstance(node, ast.AnnAssign):
                if isinstance(node.target, ast.Name):
                    assigned.add(node.target.id)
                elif isinstance(node.target, ast.Attribute):
                    assigned.add(node.target.attr)
            elif (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                  and node.value.id == "self" and node.attr.isupper()):
                read.setdefault(node.attr, node.lineno)
        for name, line in sorted(read.items(), key=lambda kv: kv[1]):
            if name not in assigned:
                out.append(f"{path}:{line}: self.{name} is read but {cls.name} never assigns it")
    return out


def main(argv: list[str]) -> int:
    bad = [msg for p in argv[1:] for msg in check(p)]
    for msg in bad:
        print(msg)
    print(f"checked {len(argv) - 1} file(s), {len(bad)} problem(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
