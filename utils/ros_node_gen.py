# This Utils file is to generate ROS Node scripts directly from JSON File
# Should support both C++ and Python Generation

# Ideally should go through JSON, through each Node, Detect node attribute,
# and produced relevant code. All Node attributes under a node should have its own
# script/src with the file name equal to the node name


# TODO: C++ stuff too 

import json
import os
import sys
import importlib

# from functionality import *

from functionality.publisher import generate_stuff
# function_folder = './functionality'

def function_finder(function_folder): # Iterate through all functions inside
    available_function = []
    function_folder = os.path.join(os.getcwd(),function_folder)
    for filename in os.listdir(function_folder):
        if os.path.isfile(os.path.join(function_folder,filename)):
            available_function.append(filename.split('.')[0])
    return available_function

def json_dataloader(json_file_path): # Loads JSON file
    with open(json_file_path,"r") as json_file:
        return json.load(json_file)

def json_iterator(data,available_functions): # Reads through JSON tree
    for node, node_data in data.items():
        # print(f"Node: {node}")
        with open(f"scripts/{node}.py","w+") as file: # assuming all nodes have unique labels
            file.write('#!/usr/bin/env python3')
            node_content_list = [] # Stores all the node attributes of a node
            for node_attribute, node_attribute_data in node_data.items():
                # print(f"\t{node_attribute}: {node_attribute_data}")
                #node_attribute_data : The widget inside the particular node-attribute (functionality)
                node_content_list.append(node_function_gen(node_attribute,available_functions,node_attribute_data))
            header_list = [item[0] for item in node_content_list if item[0]]
            class_list = [item[1] for item in node_content_list if item[1]]
            main_list = [item[2] for item in node_content_list if item[2]]
            
            # header_list, class_list, main_list = zip(*node_content_list)
            for i in [header_list,class_list]:
                for j in i:
                        # print("Stuff",j)
                        file.write(j)
                        file.write("\n")
            if len(main_list):        
                file.write(main_gen())
            for i in main_list:
                    file.write(i)
            if len(main_list):        
                file.write(main_fininsher())



def main_gen():
    return """
def main(args=None):
    rclpy.init(args=args)
    """

def main_fininsher():
    return """
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    """


def node_function_gen(node_attribute,available_functions,node_attribute_data):
    if '@' in node_attribute: # Split out the random number
        node_type = node_attribute.split('@')[0]
        item_number = node_attribute.split('@')[1]
    else:
        node_type = node_attribute
    if node_type in available_functions:
        node_attribute = node_type + '_' + item_number
        submodule_name = 'functionality.'+ node_type
        function_name = 'generate_stuff'
        submodule = importlib.import_module(submodule_name)
        function = getattr(submodule,function_name)
        return function(node_attribute,node_attribute_data)
        # return node_type.generate_stuff(node_attribute,node_attribute_data)
    else:
        return '','','' # BAsically nothing from this goes into scripts
 



if __name__ == "__main__":
    
    json_file_path = "data.json"
    data = json_dataloader(json_file_path)
    function_folder = "./utils/functionality"
    available_functions = function_finder(function_folder)
    json_iterator(data,available_functions)
