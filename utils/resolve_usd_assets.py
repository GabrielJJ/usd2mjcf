from __future__ import annotations

import hashlib
import re
import shutil
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


ASSET_REF_RE = re.compile(r"@([^@]+)@")


@dataclass
class ResolveSummary:
    downloaded: int = 0
    reused: int = 0
    rewritten: int = 0
    unresolved: int = 0
    patched_file: Optional[Path] = None


def resolve_usd_input(
    input_path: Path,
    cache_dir: Path,
    *,
    enabled: bool = True,
    strict: bool = False,
    search_roots: Optional[Iterable[Path]] = None,
) -> tuple[Path, ResolveSummary]:
    """Resolve external USD dependencies for plain Python runtimes.

    The returned path is either the original input or a patched temporary USD.
    """
    summary = ResolveSummary()
    if not enabled:
        return input_path, summary

    input_path = input_path.resolve()
    cache_dir = cache_dir.resolve()
    cache_dir.mkdir(parents=True, exist_ok=True)
    search_roots = [p.resolve() for p in (search_roots or []) if p.exists()]

    text = _read_text_if_possible(input_path)
    if text is None:
        if strict:
            raise RuntimeError(f"Input USD is not a text USD file: {input_path}")
        return input_path, summary

    cache_index = _build_cache_basename_index(cache_dir)

    def mirror_remote(url: str) -> Optional[Path]:
        return _mirror_remote_asset(url, cache_dir, summary, cache_index, strict)

    replacements: dict[str, str] = {}
    unresolved_refs: list[str] = []
    refs = ASSET_REF_RE.findall(text)
    for ref in refs:
        if ref in replacements:
            continue
        resolved = _resolve_asset_ref(
            ref,
            cache_dir=cache_dir,
            cache_index=cache_index,
            search_roots=search_roots,
            mirror_remote=mirror_remote,
        )
        if resolved is None:
            unresolved_refs.append(ref)
            continue
        replacements[ref] = resolved.as_posix()

    patched_text = text
    for src, dst in replacements.items():
        patched_text = patched_text.replace(f"@{src}@", f"@{dst}@")
        summary.rewritten += 1

    summary.unresolved = len(unresolved_refs)
    if unresolved_refs and strict:
        refs_str = "\n".join(f"  - {r}" for r in unresolved_refs)
        raise RuntimeError(f"Unresolved USD references:\n{refs_str}")

    if not replacements:
        return input_path, summary

    patched_dir = cache_dir / "_patched_inputs"
    patched_dir.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha1(str(input_path).encode("utf-8")).hexdigest()[:12]
    patched_path = patched_dir / f"{input_path.stem}.{digest}.resolved{input_path.suffix}"
    patched_path.write_text(patched_text, encoding="utf-8")
    summary.patched_file = patched_path
    return patched_path, summary


def _build_cache_basename_index(cache_dir: Path) -> dict[str, Path]:
    index: dict[str, Path] = {}
    for file_path in cache_dir.rglob("*"):
        if not file_path.is_file():
            continue
        index[file_path.name] = file_path.resolve()
    return index


def _read_text_if_possible(path: Path) -> Optional[str]:
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    if raw.startswith(b"#usda"):
        return raw.decode("utf-8", errors="replace")
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return None


def _resolve_asset_ref(
    ref: str,
    *,
    cache_dir: Path,
    cache_index: dict[str, Path],
    search_roots: list[Path],
    mirror_remote,
) -> Optional[Path]:
    parsed = urllib.parse.urlparse(ref)
    scheme = parsed.scheme.lower()

    if scheme in {"http", "https"}:
        return mirror_remote(ref)

    if scheme == "file":
        file_path = Path(parsed.path)
        if file_path.exists():
            return file_path.resolve()
        by_name = cache_index.get(file_path.name)
        if by_name:
            return by_name
        for root in search_roots:
            candidate = root / file_path.name
            if candidate.exists():
                return candidate.resolve()
        return None

    if scheme:
        return None

    ref_path = Path(ref)
    if ref_path.is_absolute() and ref_path.exists():
        return ref_path.resolve()

    return None


def _mirror_remote_asset(
    url: str,
    cache_dir: Path,
    summary: ResolveSummary,
    cache_index: dict[str, Path],
    strict: bool,
) -> Optional[Path]:
    parsed = urllib.parse.urlparse(url)
    relative = Path(parsed.netloc) / parsed.path.lstrip("/")
    target = (cache_dir / "_remote" / relative).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)

    if target.exists():
        summary.reused += 1
    else:
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                with target.open("wb") as f:
                    shutil.copyfileobj(response, f)
            summary.downloaded += 1
        except Exception:
            if strict:
                raise
            return None

    cache_index[target.name] = target

    # Recursively mirror relative refs for text USD files.
    text = _read_text_if_possible(target)
    if text is None:
        return target

    for ref in ASSET_REF_RE.findall(text):
        sub = urllib.parse.urlparse(ref)
        if sub.scheme in {"http", "https"}:
            _mirror_remote_asset(ref, cache_dir, summary, cache_index, strict)
        elif sub.scheme or Path(ref).is_absolute():
            continue
        else:
            child_url = urllib.parse.urljoin(url, ref)
            _mirror_remote_asset(child_url, cache_dir, summary, cache_index, strict)
    return target
