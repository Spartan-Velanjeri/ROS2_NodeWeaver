import datetime
import os
import sys

#
#  BEGIN: copy other repos into this for docubuild
#
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# from docubuild_copy_operation import yaml_based_repo_copy

# yaml_based_repo_copy("docubuild-repo-structure.yml")
#
#  END
#

project = "BAUTIRO (TOP100)"
copyright = "2020-{year}, Robert Bosch GmbH".format(
    year=datetime.datetime.now().year
)
author = "sg82fe"
version = "1.0"
release = "0.1.0"
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",  # tell sphinx to handle these as Markdown
}
extensions = [
    "myst_parser",  # This enables Markdown mixed with rst
    "sphinx.ext.viewcode",
    "sphinxcontrib.plantuml",  # Add support for PlantUML
]

# configure Plant-UML
plantuml = "java -jar /opt/plantuml/plantuml.jar"
plantuml_output_format = "svg"

master_doc = "index"
language = "en"

exclude_patterns = [
    "**/template_decision.md",
    "**/template_lvl1.md",
    "**/template_lvl2.md",
    "**/Overview.md",
    "**/readme2.md",
    "**/readme.md",
    "**/README.md",
    "**/CHANGELOG.rst",
    "*hcu_ccu_host_prototyp*",
    "**/ursim_installation.md",
    "**/interface_definition.md",
    "**/interface_definition_v2.md",
    "**/ISSUE_TEMPLATE/*",
    "**/LICENSING/*",
    "**/CONTRIBUTING.md",
    "**/lu_driver_ouster/ros2_ouster/design/design_doc.md",
]

pygments_style = "github-dark"  # The name of the Pygments (syntax highlighting) style to use.

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_logo = "_static/bautiro_logo.png"

html_theme_options = {
    "logo_only": False,
    "display_version": True,
    "prev_next_buttons_location": "bottom",
    "style_external_links": True,
    "vcs_pageview_mode": "edit",
    "style_nav_header_background": "#0c3861",
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 6,
    "includehidden": True,
    "titles_only": False,
}
