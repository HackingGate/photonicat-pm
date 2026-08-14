#!/bin/sh
# Render man pages and fail on any groff warning.
#
# groff exits 0 even when -ww reports problems, so treat any diagnostic output
# as a failure. -z drops the formatted output and keeps the warnings.
set -eu

if ! command -v groff >/dev/null 2>&1; then
    echo "lint-manpages: groff not found; install groff to run this check" >&2
    exit 1
fi

status=0

for page in "$@"; do
    if ! output=$(groff -man -Tutf8 -ww -z -- "$page" 2>&1); then
        status=1
    fi
    if [ -n "$output" ]; then
        printf '%s\n' "$output" >&2
        status=1
    fi

    # groff accepts \\- but renders a literal backslash, which is never wanted
    # here and reads as a correct escape in the source.
    if doubled=$(grep -n '\\\\' -- "$page"); then
        printf '%s\n' "$doubled" | while IFS= read -r hit; do
            echo "$page:$hit: doubled backslash renders literally; use a single escape" >&2
        done
        status=1
    fi
done

exit "$status"
