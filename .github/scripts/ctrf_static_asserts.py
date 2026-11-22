"""Turns the compiler output of the compile-time tests into one CTRF report: one test per static_assert of tests/*.hpp.

A static_assert fails when an error, or a "required from here" line of a template error, points inside it; errors
that point at no static_assert are reported as a failed "compilation" test of their file, so none is lost.
"""
import argparse
import glob
import json
import os
import re
from collections import Counter

# "tests/test_fxp.hpp:12:5: error: ..." (paths may go through "tests/../impl/")
DIAGNOSTIC = re.compile(r"^(?P<path>[^:\s]+):(?P<line>\d+):\d+:\s+(?:(?P<error>error|fatal error):\s+(?P<message>.*)|required from here)")


def blank_comments(text):
    """The text with comments and string contents replaced by spaces, so parentheses can be counted; keeps line breaks."""
    out, i, n = [], 0, len(text)
    while i < n:
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end < 0 else end
            out.append(" " * (end - i)); i = end
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            out.append(re.sub(r"[^\n]", " ", text[i:end])); i = end
        elif text[i] in "\"'":
            quote, j = text[i], i + 1
            while j < n and text[j] != quote:
                j += 2 if text[j] == "\\" else 1
            out.append(quote + re.sub(r"[^\n]", " ", text[i + 1:j]) + quote); i = j + 1
        else:
            out.append(text[i]); i += 1
    return "".join(out)


def find_static_asserts(path):
    """(first line, last line, message) of each static_assert in a file; the message is its string argument, if any."""
    text = open(path, encoding="utf-8").read()
    code = blank_comments(text)
    found = []
    for match in re.finditer(r"\bstatic_assert\s*\(", code):
        depth, i = 1, match.end()
        while i < len(code) and depth:
            depth += {"(": 1, ")": -1}.get(code[i], 0)
            i += 1
        first = code.count("\n", 0, match.start()) + 1
        last = code.count("\n", 0, i) + 1
        message = re.search(r',\s*"((?:[^"\\]|\\.)*)"\s*\)$', text[match.start():i], re.S)
        condition = " ".join(text[match.end():i - 1].split())
        found.append((first, last, message.group(1) if message else condition[:120]))
    return found


def build_report(test_dir, log_lines, compiled):
    """One CTRF report: each static_assert is a test of its file's suite."""
    asserts = {os.path.normpath(path): find_static_asserts(path) for path in sorted(glob.glob(os.path.join(test_dir, "*.hpp")))}
    failures = {}        # (path, first line) -> error messages
    unattributed = {}    # path -> error lines no static_assert claims
    pending = []         # "required from here" locations waiting for the error they lead to

    for raw in log_lines:
        diagnostic = DIAGNOSTIC.match(raw.strip())
        if not diagnostic:
            continue
        location = (os.path.normpath(diagnostic.group("path")), int(diagnostic.group("line")))
        if not diagnostic.group("error"):
            pending.append(location)
            continue
        owner = None
        for path, line in [location] + pending:
            owner = next(((path, first) for first, last, _ in asserts.get(path, []) if first <= line <= last), None)
            if owner:
                break
        pending = []
        if owner:
            failures.setdefault(owner, []).append(raw.strip())
        else:
            unattributed.setdefault(location[0], []).append(raw.strip())

    tests = []
    for path, found in asserts.items():
        suite = os.path.splitext(os.path.basename(path))[0]
        for first, _, message in found:
            test = {"name": f"{suite}:{first} {message}", "status": "passed", "rawStatus": "passed", "duration": 0,
                    "retries": 0, "suite": suite, "filePath": path, "line": first}
            if (path, first) in failures:
                errors = failures[(path, first)]
                test.update(status="failed", rawStatus="failed", message=errors[0], trace="\n".join(errors))
            tests.append(test)

    # Errors outside any static_assert (a broken include, a typo in a test): one failed test per file
    for path, errors in unattributed.items():
        suite = os.path.splitext(os.path.basename(path))[0]
        tests.append({"name": f"{suite} compilation", "status": "failed", "rawStatus": "failed", "duration": 0,
                      "retries": 0, "suite": suite, "filePath": path, "message": errors[0], "trace": "\n".join(errors)})
    # The compiler failed without a readable error (crash, missing compiler)
    if not compiled and not failures and not unattributed:
        tests.append({"name": "compilation", "status": "failed", "rawStatus": "failed", "duration": 0, "retries": 0,
                      "suite": "compilation", "message": "The compiler failed", "trace": "".join(log_lines)[-4000:]})

    failed = sum(1 for test in tests if test["status"] == "failed")
    return {
        "reportFormat": "CTRF",
        "specVersion": "0.0.0",
        "results": {
            "summary": {
                "name": "SaturnMath++ compile-time tests",
                "tests": len(tests),
                "failed": failed,
                "passed": len(tests) - failed,
                "skipped": 0,
                "suites": dict(Counter(test["suite"] for test in tests))
            },
            "tool": {"name": "static_assert (sh-elf-g++)"},
            "tests": tests
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Convert the compile-time tests' compiler output to a CTRF JSON report.")
    parser.add_argument("compile_log", help="Compiler output (stderr) of the tests' compilation")
    parser.add_argument("exit_code", type=int, help="Exit code of the compiler")
    parser.add_argument("output_file", help="Path to the output JSON file")
    parser.add_argument("--tests", default="tests", help="Folder of the test headers (default: tests)")
    args = parser.parse_args()

    with open(args.compile_log, encoding="utf-8", errors="replace") as log:
        report = build_report(args.tests, log.readlines(), args.exit_code == 0)
    with open(args.output_file, "w") as json_file:
        json.dump(report, json_file, indent=4)

    summary = report["results"]["summary"]
    print(f"Conversion complete: {summary['tests']} tests, {summary['failed']} failed, saved to {args.output_file}")
    return 1 if summary["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
