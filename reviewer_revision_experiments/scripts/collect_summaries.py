#!/usr/bin/env python3
"""Collect revision experiment summaries into one Markdown index."""

from __future__ import annotations

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SUMMARY_DIR = ROOT / "summary"


def read_first_rows(csv_path: Path, limit: int = 8) -> list[list[str]]:
    rows: list[list[str]] = []
    try:
        with csv_path.open(newline="") as handle:
            reader = csv.reader(handle)
            for _, row in zip(range(limit), reader):
                rows.append(row)
    except OSError:
        return []
    return rows


def markdown_table(rows: list[list[str]]) -> str:
    if not rows:
        return ""
    width = max(len(row) for row in rows)
    padded = [row + [""] * (width - len(row)) for row in rows]
    header = padded[0]
    body = padded[1:]
    lines = [
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(["---"] * width) + " |",
    ]
    lines.extend("| " + " | ".join(row) + " |" for row in body)
    return "\n".join(lines)


def collect() -> str:
    experiment_dirs = [
        path
        for path in sorted(ROOT.iterdir())
        if path.is_dir() and path.name[:2].isdigit()
    ]

    lines: list[str] = [
        "# Reviewer Revision Experiment Index",
        "",
        f"Root: `{ROOT}`",
        "",
    ]

    for exp_dir in experiment_dirs:
        lines.append(f"## {exp_dir.name}")
        summary_md = exp_dir / "summary.md"
        if summary_md.exists():
            lines.append(summary_md.read_text(encoding="utf-8").strip())
        else:
            lines.append("_No summary.md yet._")

        summary_csvs = sorted(exp_dir.glob("*summary*.csv"))
        summary_csvs.extend(sorted((exp_dir / "paper_scale").glob("*summary*.csv")))
        if summary_csvs:
            lines.append("")
            lines.append("CSV previews:")
            for csv_path in summary_csvs:
                rows = read_first_rows(csv_path)
                lines.append("")
                lines.append(f"`{csv_path.relative_to(ROOT)}`")
                table = markdown_table(rows)
                lines.append(table if table else "_Unable to read CSV._")

        log_files = sorted(exp_dir.glob("*.log"))
        if log_files:
            lines.append("")
            lines.append("Logs:")
            for log_path in log_files:
                lines.append(f"- `{log_path.relative_to(ROOT)}`")
        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def main() -> None:
    SUMMARY_DIR.mkdir(parents=True, exist_ok=True)
    output = SUMMARY_DIR / "experiment_index.md"
    output.write_text(collect(), encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
