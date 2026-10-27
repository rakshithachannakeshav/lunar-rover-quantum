#!/usr/bin/env python3
"""Tiny markdown-table helper shared by the benchmark scripts."""

from __future__ import annotations

import math
from typing import Callable, List, Sequence, Tuple, Union

import pandas as pd

Fmt = Union[str, Callable[[object], str]]


def _cell(value: object, fmt: Fmt) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "-"
    if isinstance(value, float) and math.isinf(value):
        return "inf"
    if callable(fmt):
        return fmt(value)
    return fmt.format(value)


def md_table(df: pd.DataFrame, columns: Sequence[Tuple[str, str, Fmt]]) -> str:
    """columns = [(df_column, header, format), ...]; format is a str.format
    template such as '{:.3f}' or a callable."""
    header = "| " + " | ".join(h for _, h, _ in columns) + " |"
    rule = "|" + "|".join("---" for _ in columns) + "|"
    lines: List[str] = [header, rule]
    for _, row in df.iterrows():
        lines.append("| " + " | ".join(_cell(row[c], f) for c, _, f in columns) + " |")
    return "\n".join(lines)


def sci(x: object) -> str:
    """Compact number: 12 -> '12', 12345 -> '12,345', 1.2e9 -> '1.2e+09'."""
    v = float(x)
    if v == 0:
        return "0"
    if abs(v) >= 1e7 or abs(v) < 1e-3:
        return f"{v:.2e}"
    if abs(v) >= 100:
        return f"{v:,.0f}"
    return f"{v:.3g}"
