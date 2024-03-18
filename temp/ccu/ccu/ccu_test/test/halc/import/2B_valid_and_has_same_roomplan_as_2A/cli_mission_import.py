#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from os.path import abspath, dirname, join
from os import scandir

from grpc import insecure_channel

from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import (DirectoryStreamingMessage as DStream,
                                             FileStreamingMessage as FStream,
                                             FileTransferCompletedMessage as FileCompleted,
                                             DirectoryTransferCompletedMessage as DirCompleted,
                                             FileMetaData, DirectoryMetaData,
                                             DirectoryStreamingResponse)

from ccu_util.util import calc_md5, size


def directory_streaming_message_gen() -> DStream:

    print(f'Folder of this file: {abspath(dirname(__file__))}')
    data_dir = join(abspath(dirname(__file__)), 'data_dir')
    file_count = sum(1 for e in scandir(data_dir) if e.is_file())
    yield DStream(dir_meta_data=DirectoryMetaData(directory_name=data_dir,
                                                  directory_file_count=file_count))

    for file in scandir(data_dir):
        if file.is_dir():
            continue
        yield DStream(file_data=FStream(file_meta_data=FileMetaData(file_name=file.name,
                                                                    file_size_bytes=size(file),
                                                                    md5_hash=calc_md5(file))))
        with open(file.path, 'rb') as f:
            while True:
                chunk = f.read(1024 * 10)
                if not chunk:
                    break
                yield DStream(file_data=FStream(file_chunk_data=chunk))
        yield DStream(file_data=FStream(file_completed=FileCompleted(file_name=file.name)))
    yield DStream(dir_completed=DirCompleted())


def start_client():
    try:
        with insecure_channel('localhost:50052') as channel:
            stub = BaseLayerServiceStub(channel)
            r: DirectoryStreamingResponse = stub.ImportMission(directory_streaming_message_gen())
            print(f'Response Success    :  {r.success}\n')

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
