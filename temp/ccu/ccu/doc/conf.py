# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import sys
from datetime import datetime
from os import walk
from os.path import abspath

[sys.path.append(dir) for dir, n, f in walk(abspath('.')) if '.git/' not in dir]


project = 'BAUTIRO (TOP100)'
copyright = f'2020-{datetime.now().year}, Robert Bosch GmbH'
author = 'sg82fe'
version = f'2023.10.{datetime.now().isoformat()}'
release = '2'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

language = "en"
master_doc = 'index'           # index.rst  as entry point

source_suffix = {
    '.rst': 'restructuredtext',
    '.md':  'markdown',         # tell sphinx to handle these as Markdown
}

extensions = [
    'myst_parser',             # This enables Markdown mixed with rst
    'sphinx.ext.viewcode',
    'sphinxcontrib.plantuml',  # Add support for PlantUML
    'sphinx.ext.todo',
    'sphinx.ext.autodoc'
]

myst_enable_extensions = [
    "dollarmath",
]
myst_dmath_double_inline = True

pygments_style = 'github-dark'  # The name of the Pygments (syntax highlighting) style to use.

# configure Plant-UML
plantuml = 'java -jar /opt/plantuml/plantuml.jar'
plantuml_output_format = 'svg'

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ['custom.css']
html_logo = '_static/bautiro_logo.png'
html_theme_options = {
    'prev_next_buttons_location': 'both',  # bottom, top, both, None
    'style_external_links': True,
    'vcs_pageview_mode': 'view',  # On GitHub or GitLab this can be: blob (default), edit, or raw. On Bitbucket, this can be either: view (default) or edit.
    'style_nav_header_background': '#0c3861',
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 6,
    'includehidden': True,
    'titles_only': False
}


# files to exclude
exclude_patterns = [
    '**/template_decision.md',
    '**/template_lvl1.md',
]
