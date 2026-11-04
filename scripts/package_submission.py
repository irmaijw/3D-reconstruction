"""Build the handout submission layout from an explicit source allowlist."""
from pathlib import Path
import hashlib
import zipfile
ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'112006220_hw1.zip'

def main():
    files={}
    def include(path,archive_name=None):
        p=ROOT/path
        if not p.is_file():raise FileNotFoundError(p)
        files[archive_name or p.relative_to(ROOT).as_posix()]=p
    for name in ['load.py','reconstruct.py','utils.py','api.py','plot.py','README.md','pixi.toml','pixi.lock','ontology/hw1.ttl']:
        include(name)
    include('112006220_hw1_report.pdf','report.pdf')
    for directory in ['configs','docs','packages/simulator','tests']:
        for p in sorted((ROOT/directory).rglob('*')):
            if p.is_file() and not any(x.startswith('.') or x=='__pycache__' for x in p.relative_to(ROOT).parts) and p.suffix not in {'.pyc','.pyo'}:
                include(p.relative_to(ROOT))
    for name in ['compare_common_frames.py','summarize_experiment.py','view_reconstruction.py','fuse_map.py','package_submission.py']:
        include('scripts/'+name)
    tmp=OUTPUT.with_suffix('.building.zip')
    with zipfile.ZipFile(tmp,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for archive_name,p in sorted(files.items()):z.write(p,archive_name)
    with zipfile.ZipFile(tmp) as z:
        assert z.testzip() is None
        assert {'load.py','reconstruct.py','utils.py','api.py','ontology/hw1.ttl','report.pdf','README.md'}<=set(z.namelist())
        assert z.read('report.pdf')==(ROOT/'112006220_hw1_report.pdf').read_bytes()
        assert not any(n.startswith(('eval/','experiments/','replica_v1/','.pixi/','report_assets/')) for n in z.namelist())
    tmp.replace(OUTPUT)
    print(f'{OUTPUT.name}: {len(files)} files, {OUTPUT.stat().st_size:,} bytes')
    print('SHA256',hashlib.sha256(OUTPUT.read_bytes()).hexdigest())
if __name__=='__main__':main()
