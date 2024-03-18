#!/usr/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
# flake8: noqa


from os import makedirs, walk, chdir, scandir
from os.path import abspath, dirname, exists, join, relpath, basename
from shutil import copy2
import yaml

YAML_FILENAME = 'docubuild-repo-structure.yml'


def yaml_based_repo_copy(yaml_filename: str):
    doc_path = dirname(abspath(__file__))
    copy_root_stuff_into_doc(doc_path)
    copy_yaml_stuff_into_doc(doc_path, yaml_filename)


def copy_root_stuff_into_doc(doc_path):
    integration_repo_path = dirname(doc_path)
    for doc_sibling in scandir(integration_repo_path):
        if doc_sibling.name != 'doc' and not doc_sibling.name.startswith('.'):
            s = join(doc_sibling.path)
            t = join(doc_path, doc_sibling.name)
            if doc_sibling.is_file() and doc_sibling.name not in \
                    ['dependencies.repos', 'jenkinsfile_bautiro', 'readme.md']:
                if not exists(dirname(t)):
                    makedirs(dirname(t))
                copy2(s, t)
            else:
                copy_files(s, t)


def copy_yaml_stuff_into_doc(doc_path, yaml_filename):
    integration_repo_path = dirname(doc_path)
    integration_repo_name = basename(integration_repo_path)
    chdir(doc_path)
    with open(yaml_filename, "r") as yaml_file:
        yaml_dict: dict = yaml.safe_load(yaml_file)
        for function_repo_name in yaml_dict[integration_repo_name]:
            workspace_path: str = dirname(integration_repo_path)
            s = join(workspace_path, function_repo_name)
            t = join(doc_path, function_repo_name)
            # print(s+'::::::'+t)
            copy_files(s, t)


def copy_files(src, dst):
    for root, dirs, files in walk(src):
        for file in files:
            if condition_fulfilled(root, file):
                src_path = join(root, file)
                dst_path = join(dst, relpath(src_path, src))
                if not exists(dirname(dst_path)):
                    makedirs(dirname(dst_path))
                # print('src:'+src_path + '\n' + 'dst:' + dst_path)  #
                copy2(src_path, dst_path)


def condition_fulfilled(root, file):
    return ('.git' not in root)
    # intention: only copy ..../doc/....  folders
    # return (keyword in abspath(join(root, file)))


if __name__ == '__main__':
    yaml_based_repo_copy(YAML_FILENAME)
