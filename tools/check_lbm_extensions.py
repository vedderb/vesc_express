#!/usr/bin/env python3
"""
Fail the build if the built-in lispBM extensions exceed EXTENSION_STORAGE_SIZE.

Preprocesses the main component sources with their commands from
compile_commands.json and counts unique names passed to lbm_add_extension,
as string literals or via { "name", fn } tables. Runtime-conditional
registrations are counted too, so the result is an upper bound.
"""

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

# Files that register extensions with runtime names (package ext- placeholders,
# native libs). These use the USER_EXTENSION_STORAGE_SIZE reserve.
DYNAMIC_CALL_FILES = {"eval_cps.c", "lispif_c_lib.c", "extensions.c"}

CALL_RE = re.compile(r"\blbm_add_extension\s*\(")
STRING_LIT_RE = re.compile(r'\s*((?:"(?:[^"\\]|\\.)*"\s*)+)')
TABLE_ARG_RE = re.compile(r"\s*(\w+)\s*\[[^\]]*\]\s*\.\s*\w+")
IDENT_ARG_RE = re.compile(r"\s*(\w+)\s*[,)]")
DECL_ARG_RE = re.compile(r"\s*(?:const\s+)?char\b")

# Files worth preprocessing: those that register extensions or include other
# sources (e.g. #include HW_SOURCE in hwconf/hw.c).
PREFILTER_RE = re.compile(r'lbm_add_extension|^\s*#\s*include\s+(?:[A-Za-z_]\w*\s*$|"[^"]+\.c")', re.M)


def split_windows(cmd):
    """Split a command line using the Windows C runtime rules."""
    args = []
    arg = ""
    in_arg = in_quotes = False
    i = 0
    while i < len(cmd):
        c = cmd[i]
        if c == "\\":
            n = 0
            while i < len(cmd) and cmd[i] == "\\":
                n += 1
                i += 1
            if i < len(cmd) and cmd[i] == '"':
                arg += "\\" * (n // 2)
                if n % 2:
                    arg += '"'
                    i += 1
            else:
                arg += "\\" * n
            in_arg = True
            continue
        if c == '"':
            if in_quotes and i + 1 < len(cmd) and cmd[i + 1] == '"':
                arg += '"'
                i += 1
            else:
                in_quotes = not in_quotes
            in_arg = True
        elif c in " \t" and not in_quotes:
            if in_arg:
                args.append(arg)
                arg = ""
                in_arg = False
        else:
            arg += c
            in_arg = True
        i += 1
    if in_arg:
        args.append(arg)
    return args


def split_command(entry):
    if "arguments" in entry:
        return list(entry["arguments"])
    if os.name == "nt":
        return split_windows(entry["command"])
    return shlex.split(entry["command"])


def preprocess_args(args, extra):
    """Turn a compile command into one that preprocesses to stdout."""
    out = []
    skip_next = False
    for a in args:
        if skip_next:
            skip_next = False
        elif a in ("-o", "-MF", "-MT", "-MQ"):
            skip_next = True
        elif a not in ("-c", "-MD", "-MMD", "-MP") and not a.startswith("-o"):
            out.append(a)
    return out[:1] + ["-E", "-P"] + extra + out[1:]


def run_preprocessor(entry, extra=()):
    args = preprocess_args(split_command(entry), list(extra))
    res = subprocess.run(args, cwd=entry["directory"], capture_output=True,
                         encoding="utf-8", errors="replace")
    if res.returncode != 0:
        raise RuntimeError("preprocessing %s failed:\n%s" % (entry["file"], res.stderr))
    return res.stdout


def decode_c_strings(lit):
    return "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', lit))


def find_table_type(text, name):
    """Element type of a table, from 'T *name' or 'T name[]'."""
    m = re.search(r"\b(\w+)\s*\*\s*(?:const\s+)?" + re.escape(name) + r"\b", text)
    if m and m.group(1) not in ("const", "static", "char", "void"):
        return m.group(1)
    m = re.search(r"\b(\w+)\s+(?:const\s+)?" + re.escape(name) + r"\s*\[", text)
    if m and m.group(1) not in ("const", "static"):
        return m.group(1)
    return None


def table_entry_names(text, type_name):
    """Names from all '{ "name", ... }' entries of arrays of type_name."""
    names = []
    for m in re.finditer(r"\b" + re.escape(type_name) + r"\s+\w+\s*\[[^\]]*\]\s*=\s*\{", text):
        i = start = m.end()
        depth = 1
        while i < len(text) and depth > 0:
            c = text[i]
            if c == '"':
                i += 1
                while i < len(text) and text[i] != '"':
                    i += 2 if text[i] == "\\" else 1
            elif c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
            i += 1
        for e in re.finditer(r'\{\s*((?:"(?:[^"\\]|\\.)*"\s*)+)\s*,', text[start:i - 1]):
            names.append(decode_c_strings(e.group(1)))
    return names


def scan_file(entry):
    text = run_preprocessor(entry)
    base = os.path.basename(entry["file"])
    names = []
    errors = []
    for m in CALL_RE.finditer(text):
        rest = text[m.end():m.end() + 300]
        lit = STRING_LIT_RE.match(rest)
        table = TABLE_ARG_RE.match(rest)
        if lit:
            names.append(decode_c_strings(lit.group(1)))
        elif DECL_ARG_RE.match(rest):
            pass  # Prototype or definition
        elif table:
            t = find_table_type(text, table.group(1))
            entries = table_entry_names(text, t) if t else []
            if not entries:
                errors.append("%s: table registration via '%s' not understood" % (base, table.group(1)))
            names.extend(entries)
        elif not (IDENT_ARG_RE.match(rest) and base in DYNAMIC_CALL_FILES):
            errors.append("%s: unrecognised call: lbm_add_extension(%s" % (base, rest.split(";")[0].strip()))
    return [names, errors]


def read_limits(entry):
    defs = run_preprocessor(entry, ["-dM"])
    vals = {}
    for key in ("EXTENSION_STORAGE_SIZE", "USER_EXTENSION_STORAGE_SIZE"):
        m = re.search(r"^#define\s+" + key + r"\s+(.+)$", defs, re.M)
        if not m:
            raise RuntimeError("%s not found in %s" % (key, entry["file"]))
        vals[key] = int(m.group(1).strip().strip("()"), 0)
    return vals


def object_mtime(entry):
    out = entry.get("output")
    if not out:
        return None
    try:
        return os.stat(os.path.join(entry["directory"], out)).st_mtime_ns
    except OSError:
        return None


def load_cache(path, key):
    try:
        with open(path, encoding="utf-8") as f:
            c = json.load(f)
        return c["files"] if c.get("key") == key else {}
    except (OSError, ValueError, KeyError, TypeError):
        return {}


def save_cache(path, key, files):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"key": key, "files": files}, f)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--compile-commands", required=True)
    ap.add_argument("--source-dir", required=True, help="main component directory")
    ap.add_argument("--stamp", help="written when the check passes")
    ap.add_argument("--cache", help="per-file result cache")
    ap.add_argument("--verbose", action="store_true", help="list counted extensions")
    a = ap.parse_args()

    src_dir = os.path.normcase(os.path.realpath(a.source_dir))
    with open(a.compile_commands, encoding="utf-8") as f:
        db = json.load(f)

    entries = []
    lispif = None
    for e in db:
        path = os.path.normcase(os.path.realpath(os.path.join(e["directory"], e["file"])))
        if not path.startswith(src_dir + os.sep) or not path.endswith(".c"):
            continue
        if os.path.basename(path) == "lispif.c":
            lispif = e
        try:
            with open(path, encoding="utf-8", errors="replace") as f:
                if not PREFILTER_RE.search(f.read()):
                    continue
        except OSError:
            pass
        entries.append(e)

    if lispif is None:
        print("check_lbm_extensions: lispif.c not found in compile_commands.json", file=sys.stderr)
        return 1

    # Reuse results while the object file is unchanged; it is rebuilt on any
    # source or header change.
    key = "%d:%d" % (os.stat(os.path.abspath(__file__)).st_mtime_ns, os.stat(a.compile_commands).st_mtime_ns)
    cache = load_cache(a.cache, key) if a.cache else {}
    new_cache = {}

    def cached(kind, fn, entry):
        mt = object_mtime(entry)
        ck = kind + ":" + entry["file"]
        hit = cache.get(ck)
        res = hit["result"] if mt is not None and hit and hit["mtime"] == mt else fn(entry)
        if mt is not None:
            new_cache[ck] = {"mtime": mt, "result": res}
        return res

    with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        limits_f = ex.submit(cached, "limits", read_limits, lispif)
        results = list(ex.map(lambda e: (e, cached("scan", scan_file, e)), entries))
        limits = limits_f.result()

    if a.cache:
        save_cache(a.cache, key, new_cache)

    origin = {}
    errors = []
    for e, (names, errs) in results:
        errors.extend(errs)
        for n in names:
            origin.setdefault(n, os.path.basename(e["file"]))

    if errors:
        print("check_lbm_extensions: cannot count extensions:", file=sys.stderr)
        for err in errors:
            print("  " + err, file=sys.stderr)
        print("  Update tools/check_lbm_extensions.py for the new registration style.", file=sys.stderr)
        return 1

    if a.verbose:
        for n in sorted(origin):
            print("  %-40s %s" % (n, origin[n]))

    count = len(origin)
    cap = limits["EXTENSION_STORAGE_SIZE"]
    reserve = limits["USER_EXTENSION_STORAGE_SIZE"]

    if count > cap:
        print("check_lbm_extensions: ERROR: %d built-in lispBM extensions, but EXTENSION_STORAGE_SIZE is %d."
              % (count, cap), file=sys.stderr)
        print("  Raise EXTENSION_STORAGE_SIZE in main/lispif.c to at least %d." % count, file=sys.stderr)
        return 1

    print("check_lbm_extensions: %d built-in extensions, capacity %d (%d free), +%d reserved for packages/native libs"
          % (count, cap, cap - count, reserve))

    if a.stamp:
        with open(a.stamp, "w", encoding="utf-8") as f:
            f.write("%d/%d\n" % (count, cap))
    return 0


if __name__ == "__main__":
    sys.exit(main())
