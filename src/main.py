import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from viam.module.module import Module  # noqa: E402

import touchoff.controls  # noqa: E402,F401  (importing registers the models with the registry)
import touchoff.service  # noqa: E402,F401

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
