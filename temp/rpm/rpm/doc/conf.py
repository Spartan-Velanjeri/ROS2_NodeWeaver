# Copyright 2020 Robert Bosch GmbH and its subsidiaries.
#
# This program and the accompanying materials are made available under the
# terms of the Bosch Internal Open Source License v4 which accompanies this
# distribution, and is available at http://bios.intranet.bosch.com/bioslv4.txt
# flake8: noqa
import datetime

#  BEGIN: copy other repos into this for docubuild
#  sys.path.append(os.path.dirname(os.path.abspath(__file__)))
#  from  docubuild_copy_operation import yaml_based_repo_copy
#  yaml_based_repo_copy('docubuild-repo-structure.yml')
#  END

project = 'BAUTIRO (TOP100)'
copyright = '2020-{year}, Robert Bosch GmbH'.format(year=datetime.datetime.now().year)
author = 'sg82fe'
version = '1.0'
release = '0.1.0'
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',        # tell sphinx to handle these as Markdown
}
extensions = [
    'myst_parser',             # This enables Markdown mixed with rst
    'sphinx.ext.viewcode',
]
templates_path = ['_templates']
master_doc = 'index'
language = "en"

exclude_patterns = [
    'readme.md',
    '**/readme.md',
    '**/README.MD',
    '**/README.md',
]

pygments_style = 'sphinx'  # The name of the Pygments (syntax highlighting) style to use.

html_theme = 'sphinx_rtd_theme'
html_static_path = ['img']
html_css_files = ['custom.css']
# html_logo = 'img/bautiro_logo.png'

html_theme_options = {
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'vcs_pageview_mode': 'edit',
    'style_nav_header_background': '#04975',
    # Toc options
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 6,
    'includehidden': True,
    'titles_only': False
}
