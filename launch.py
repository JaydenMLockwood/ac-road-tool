#!/usr/bin/env python3
"""
AC Road Tool launcher — auto-opens browser.
"""
import subprocess
import sys
import time
import webbrowser
import os

PORT = 8743

def check_deps():
    missing = []
    for pkg in ['numpy', 'scipy', 'pyproj']:
        try:
            __import__(pkg)
        except ImportError:
            missing.append(pkg)
    if missing:
        print(f"Installing missing packages: {', '.join(missing)}")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install'] + missing)

if __name__ == '__main__':
    check_deps()

    server_path = os.path.join(os.path.dirname(__file__), 'server.py')
    proc = subprocess.Popen([sys.executable, server_path])

    time.sleep(1.0)
    print(f"Opening http://localhost:{PORT} ...")
    webbrowser.open(f"http://localhost:{PORT}")

    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        print("Stopped.")
