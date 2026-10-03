# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec file for MBD Pre-Processor

This is a starting point. Packaging pythonocc-core is non-trivial.
Expect the final folder to be very large (500MB+).

Usage:
    pyinstaller MBD-PreProcessor.spec

Recommended: Build inside the same conda environment where the app works:
    conda activate mbd_preproc
    pyinstaller MBD-PreProcessor.spec
"""

from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs
import os

block_cipher = None

# ------------------------------------------------------------------
# Collect OCC resources (very important for pythonocc-core)
# ------------------------------------------------------------------
occ_datas = []
occ_binaries = []

try:
    import OCC
    occ_path = os.path.dirname(OCC.__file__)
    # Include share/resources if present (step files, etc.)
    share_path = os.path.join(occ_path, '..', '..', 'share')
    if os.path.exists(share_path):
        occ_datas.append((share_path, 'share'))
except Exception:
    pass

# ------------------------------------------------------------------
# Main analysis
# ------------------------------------------------------------------
a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=occ_binaries,
    datas=occ_datas + [
        # Add any extra data files here if needed
    ],
    hiddenimports=[
        # PySide6
        'PySide6.QtCore',
        'PySide6.QtGui',
        'PySide6.QtWidgets',
        'PySide6.QtOpenGL',
        # OCC core modules that are often not auto-detected
        'OCC.Core',
        'OCC.Core.TopoDS',
        'OCC.Core.TopExp',
        'OCC.Core.TopAbs',
        'OCC.Core.STEPControl',
        'OCC.Core.IFSelect',
        'OCC.Core.BRepPrimAPI',
        'OCC.Core.gp',
        'OCC.Core.AIS',
        'OCC.Core.V3d',
        'OCC.Core.Aspect',
        'OCC.Display',
        'OCC.Display.qtDisplay',
        'OCC.Display.backend',
        # Scientific
        'scipy.spatial',
        'scipy.sparse',
        'scipy.sparse.linalg',
        'trimesh',
        'numpy',
        'jax',
        'jaxlib',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='MBD-PreProcessor',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,          # Set to False for a windowed app (no console)
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,             # Add path to .ico if you have one
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='MBD-PreProcessor',
)
