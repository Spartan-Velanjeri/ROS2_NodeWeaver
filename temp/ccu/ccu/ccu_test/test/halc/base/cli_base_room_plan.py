#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from sys import argv
from typing import List
from os import makedirs
from os.path import join
from shutil import rmtree

from grpc import insecure_channel

from ccu_grpc.base_layer_service_pb2 import MissionIdRequest, FileStreamingMessage
from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub as Stub


# SERVER = '192.168.0.2'  # on Spectra
TEST_DIR = '/tmp/test_transfer_get_obj_file'


def handle(response_stream: List[FileStreamingMessage]):

    rmtree(TEST_DIR, ignore_errors=True)
    makedirs(TEST_DIR, exist_ok=True)

    first: FileStreamingMessage
    for first in response_stream:
        if 'file_meta_data' != first.WhichOneof('data'):
            print("Error: First Message must be of type 'file_meta_data'")
            break
        else:
            print(f' Start file transfer of : {first.file_meta_data.file_name}\n' +
                  f'             size(bytes): {first.file_meta_data.file_size_bytes}')
            with open(join(TEST_DIR, first.file_meta_data.file_name), 'wb') as f:
                c = 1
                msg: FileStreamingMessage
                for msg in response_stream:
                    if 'file_chunk_data' == msg.WhichOneof('data'):
                        print(f'               receiving chunk: {c:03}' +
                              f'         [payload-size (bytes): {len(msg.file_chunk_data)}]')
                        f.write(msg.file_chunk_data)
                        c += 1
                    elif 'file_completed' == msg.WhichOneof('data'):
                        print(f'   Completed file transfer of : {msg.file_completed.file_name}]')
                    else:
                        print("Error: Subsequent messages at his point must be of type " +
                              "'file_chunk_data' or 'file_completed'")


SERVER = 'localhost'
PORT = 50052


def start_client(mission_index: int):
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel(f'{SERVER}:{PORT}') as channel:
            print('GetRoomPlanObjFile:')

            handle(Stub(channel).GetRoomPlanObjFile(MissionIdRequest(mission_id=mission_index)))

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    print(argv)
    if len(argv) > 1:
        start_client(int(argv[1]))
    else:
        start_client(1)
