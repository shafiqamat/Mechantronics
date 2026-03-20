# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os, sys
import time as _time
sys.path.insert(0, os.path.abspath('../src'))  # ← adjust this path to wherever your .py files are


def _ticks_us():
    """Provide a MicroPython-like fallback for desktop doc builds."""
    return int(_time.time() * 1_000_000)


def _ticks_diff(new, old):
    """Provide a MicroPython-like fallback for desktop doc builds."""
    return new - old


if not hasattr(_time, 'ticks_us'):
    _time.ticks_us = _ticks_us

if not hasattr(_time, 'ticks_diff'):
    _time.ticks_diff = _ticks_diff

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Mechatronics final'
copyright = '2026, Max Soury and Shafiq Amat'
author = 'Max Soury and Shafiq Amat'
release = '1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',    # ← reads your docstrings automatically
    'sphinx.ext.napoleon',   # ← lets you write cleaner docstrings
    'sphinx.ext.viewcode',   # ← adds "view source" links
    'sphinxcontrib.video',
]

# Mock MicroPython / pyb so modules can be imported when building on desktop
autodoc_mock_imports = ['pyb', 'utime', 'micropython', 'ulab', 'ulab.numpy']

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'   
html_static_path = ['_static']
