#!/usr/bin/env python3
"""Fail if current inputs or generated P0 assets no longer match the manifest."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/i10_rh56_moveit_config'


def check(root=ROOT, package=PACKAGE):
    provenance = json.loads((package / 'config/provenance.json').read_text())
    generated = json.loads((package / 'config/generated_hashes.json').read_text())
    errors = []
    for base, records in [(root, provenance['sources']), (package, generated)]:
        for name, expected in records.items():
            path = base / name
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                errors.append(str(path))
    if errors:
        raise ValueError('Stale P0 model; regenerate before loading:\n' + '\n'.join(errors))
    return len(provenance['sources']), len(generated)


if __name__ == '__main__':
    inputs, outputs = check()
    print(f'Model provenance OK: {inputs} inputs, {outputs} generated files')
