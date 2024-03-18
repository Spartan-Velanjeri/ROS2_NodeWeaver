# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ccu_grpc.base_layer_service_pb2 import DeveloperFunctionList
from bautiro_developer_functions.core.developer_functions import DeveloperFunctions


class DeveloperFunctionHandlerTest(DeveloperFunctions):

    @property
    def config(self):
        repo = '/home/sg82fe/btr-devcontainer/ws/src/ccu/ccu_data_services'
        return repo + '/ccu_hcu_abstraction/config/'


if __name__ == '__main__':
    candidate = DeveloperFunctionHandlerTest(None, None, None)
    print(candidate.config)

    dev_func_list: DeveloperFunctionList = candidate.get_developer_functions(None)

    for source in dev_func_list.sources:

        print(f'{source.name} =============================================')
        for function in source.functions:
            print(f'-- {function.name} : {function.description} -------------------------------')
            for param in function.parameters:
                if param.HasField('param_number'):
                    print('        - float  ' + param.param_number.param_name)
                else:
                    print('        - bool   ' + param.param_bool.param_name)
