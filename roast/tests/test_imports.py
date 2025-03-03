import importlib
import os
import pathlib
import pkgutil
import subprocess
import sys

exit_code = 0

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.resolve()))

packages = ["roast"]
skip_packages = ["roast.io", "roast.perf"]


def test_imports():
    # Setup the environment variables
    os.environ["DISPLAY"] = ":0"
    os.environ["MPLBACKEND"] = "Agg"

    while len(packages) > 0:
        package_name = packages.pop()
        if package_name in skip_packages:
            continue

        import_context = f'python3 -c "import {package_name}"'
        print(f"Importing {import_context}")
        pipe = subprocess.Popen(import_context, shell=True, stderr=subprocess.PIPE)
        return_code = pipe.wait()
        if return_code != 0:
            print()
            raise ImportError(pipe.communicate()[1].decode())

        print(f'Test import {package_name}: {"DONE" if return_code == 0 else "FAIL"}')
        package = importlib.import_module(package_name)
        for _, modname, ispkg in pkgutil.iter_modules(package.__path__):
            if ispkg:
                packages.append(f"{package_name}.{modname}")
