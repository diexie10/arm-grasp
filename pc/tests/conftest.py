# -*- coding: utf-8 -*-
"""conftest.py — insert pc/ into sys.path so imports like `import config` work."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
