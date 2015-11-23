import os.path, subprocess, shutil

from build.project import Project

def run_quilt(toolchain, cwd, patches_path, *args):
    env = dict(toolchain.env)
    env['QUILT_PATCHES'] = patches_path
    subprocess.check_call(['/usr/bin/quilt'] + list(args), cwd=cwd, env=env)

def push_all(toolchain, src_path, patches_path):
    run_quilt(toolchain, src_path, patches_path, 'push', '-a')
