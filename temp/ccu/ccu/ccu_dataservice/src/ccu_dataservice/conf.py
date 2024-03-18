# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import shutil
from os import environ
from os.path import join
from pathlib import Path


class Conf():
    """DATA-SERVICE configuration: 'data'-folder: '/home/<user>/data'."""

    # core.py - CoreDs - get_next_drill_jobs
    NUMBER_OF_NEXT_DEFAULT = 23
    NUMBER_OF_NEXT_MAX = 101
    FQN_SEPERATOR = '::'

    def __init__(self, data_folder_name: str = None) -> None:
        # data-folder    '/..../<data>'
        self.data_folder_name = data_folder_name or 'bautiro_data_tp21'
        self.data_folder_path = self.__set_data_folder(self.data_folder_name)

    def __set_data_folder(self, name: str) -> str:
        if 'CCU_DATA_DIR' in environ:
            d = join(environ['CCU_DATA_DIR'], name)
        elif ('HOME' in environ):
            d = join(environ['HOME'], name)
        else:
            d = f'/tmp/{name}'
        Path(d).mkdir(parents=True, exist_ok=True)  # guaranty exists
        return d

    def _clean(self) -> None:
        print(f"Permanently delete data_folder: '{self.data_folder_path}'", end=' ')
        shutil.rmtree(self.data_folder_path)
        print('whoops')
        return self
