#!/usr/bin/env python3
"""Fail when a Markdown file links to an in-document #anchor that no heading defines.

Slugs follow the GitHub rules: drop inline code and link markup, lowercase,
strip everything except word characters, spaces and hyphens, then turn spaces
into hyphens. Headings inside fenced code blocks are ignored, so shell comments
in examples are not mistaken for headings.
"""

import re
import sys

FENCE = re.compile(r"^\s*(```|~~~)")
HEADING = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
LOCAL_LINK = re.compile(r"\]\(#([^)]+)\)")


def slug(heading: str) -> str:
    text = re.sub(r"`([^`]*)`", r"\1", heading)
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"[*_~]", "", text)
    text = text.strip().lower()
    text = re.sub(r"[^\w\- ]", "", text)
    return text.replace(" ", "-")


def scan(path: str) -> list[str]:
    """Return one message per link that points at a missing anchor."""
    anchors: set[str] = set()
    links: list[tuple[int, str]] = []
    fence = None

    with open(path, encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            marker = FENCE.match(line)
            if marker:
                if fence is None:
                    fence = marker.group(1)
                elif line.strip().startswith(fence):
                    fence = None
                continue
            if fence is not None:
                continue

            heading = HEADING.match(line)
            if heading:
                base = slug(heading.group(2))
                candidate = base
                suffix = 0
                while candidate in anchors:
                    suffix += 1
                    candidate = f"{base}-{suffix}"
                anchors.add(candidate)

            links.extend((number, target) for target in LOCAL_LINK.findall(line))

    return [
        f"{path}:{number}: link to #{target} matches no heading"
        for number, target in links
        if target not in anchors
    ]


def main(paths: list[str]) -> int:
    problems = [message for path in paths for message in scan(path)]
    for message in problems:
        print(message, file=sys.stderr)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
