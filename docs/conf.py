# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os, sys
sys.path.insert(0, os.path.abspath('../src'))  # ← adjust this path to wherever your .py files are

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
]

# Mock MicroPython / pyb so modules can be imported when building on desktop
autodoc_mock_imports = ['pyb', 'utime', 'micropython']

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'   
html_static_path = ['_static']
