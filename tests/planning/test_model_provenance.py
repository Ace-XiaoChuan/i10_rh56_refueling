"""拒绝失效的规划资产；保留的源几何不得被改动。"""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('model_sources', ROOT / 'tools/check_moveit_model_sources.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ProvenanceTests(unittest.TestCase):
    def test_current_model_matches_inputs_and_outputs(self):
        sources, outputs = module.check()
        self.assertGreater(sources, 0)
        self.assertGreater(outputs, 0)

    def test_changed_or_missing_source_and_generated_file_are_rejected(self):
        for target, remove in [('source.txt', False), ('package/robot.urdf', False), ('source.txt', True)]:
            with self.subTest(target=target, remove=remove), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                package = root / 'package'
                (package / 'config').mkdir(parents=True)
                source = root / 'source.txt'
                generated = package / 'robot.urdf'
                source.write_text('source')
                generated.write_text('generated')
                digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
                (package / 'config/provenance.json').write_text(json.dumps({'sources': {'source.txt': digest(source)}}))
                (package / 'config/generated_hashes.json').write_text(json.dumps({'robot.urdf': digest(generated)}))
                module.check(root, package)
                if remove:
                    (root / target).unlink()
                else:
                    (root / target).write_text('changed')
                with self.assertRaisesRegex(ValueError, 'Stale P0 model'):
                    module.check(root, package)


if __name__ == '__main__':
    unittest.main()
