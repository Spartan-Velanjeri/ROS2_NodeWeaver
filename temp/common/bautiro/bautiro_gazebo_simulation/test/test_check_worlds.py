# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os
import unittest
import subprocess
from ament_index_python.packages import get_package_share_directory


class TestCheckSDFFiles(unittest.TestCase):
    def setUp(self) -> None:
        self._world_files = []
        package_path = get_package_share_directory(
            'bautiro_gazebo_simulation')
        worlds_dir = os.path.join(
            package_path, 'worlds')

        for item in os.listdir(worlds_dir):
            if not item.endswith(('.sdf', '.world')):
                continue
            self._world_files.append(
                os.path.join(worlds_dir, item))

    def test_check_sdf(self):
        self.assertGreater(
            len(self._world_files), 0,
            'No world files were found: ' + str(len(self._world_files)))
        for world_file in self._world_files:
            output = subprocess.check_output(
                f'ign sdf -k {world_file}', shell=True).decode('utf-8')
            self.assertIn('Valid.', output)
