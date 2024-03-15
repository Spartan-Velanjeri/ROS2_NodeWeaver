# TODO
# Make sure all node labels and node attributes are unique

import dearpygui.dearpygui as dpg
import json
import random
import string

# Variables

link_info = {}

dpg.create_context()
# callback runs when user attempts to connect attributes
def link_callback(sender, app_data): # On dragging and dropping a link
    # app_data -> (link_id1, link_id2)
    tag = dpg.add_node_link(app_data[0], app_data[1], parent=sender)
    link_info[tag] = [app_data[0],dpg.get_item_parent(app_data[0]),app_data[1],dpg.get_item_parent(app_data[1])] # Tag = Start ID (Node Attribute), Parent (Node name), End ID,Parent (Node name)
    # print(link_info)

# callback runs when user attempts to disconnect attributes
def delink_callback(sender, app_data): # On ctrl+click a link
    # app_data -> link_id
    dpg.delete_item(app_data)
    del link_info[app_data] 
    # node_properties_callback() TODO: Auto-Refresh of Property window after removing link

def add_node_callback(): # On clicking the New_node button
    with dpg.node(label="New_node",parent=NodeEditor):
        with dpg.node_attribute(label="Description",attribute_type=dpg.mvNode_Attr_Static):
            dpg.add_input_text(label="Node Description",hint="Add Node description ...",width=250)

def delete_node_callback(sender,app_data): # On Delete button or Delete Node button
    for i in (dpg.get_selected_nodes(NodeEditor)):
        dpg.delete_item(i)

def rename_node_callback(sender,data): #Works !
    if len(dpg.get_selected_nodes(NodeEditor)) == 1:# and dpg.is_item_clicked(dpg.get_selected_nodes(NodeEditor)[0]):
        input_text_value = dpg.get_value("renaming_tag")
        dpg.configure_item(dpg.get_selected_nodes(NodeEditor)[0],label=input_text_value)
        dpg.delete_item("Rename_Node")

def remove_feature(sender):
    dpg.delete_item(dpg.get_item_parent(sender))

def node_properties_callback():
    #print(link_info)
    clear_window(properties_window)
    if len(dpg.get_selected_nodes(NodeEditor)) == 1:
        #print(f"Node ID: {dpg.get_selected_nodes(NodeEditor)[0]}")
        #print(dpg.get_item_children(dpg.get_selected_nodes(NodeEditor)[0]))
        selected_node_id = dpg.get_selected_nodes(NodeEditor)[0]
        Node_label = (dpg.get_item_label(selected_node_id))
        dpg.add_text(default_value=Node_label,parent=properties_window)
        for key,values in (dpg.get_item_children(selected_node_id)).items(): # For Each Node
            # If you need to further iterate through the list of values:
            for value in values: # For each Node attribute
                #node_attribute_window = dpg.child_window(label=(value),parent=properties_window)
                for key,values in (dpg.get_item_children(value)).items(): # For each widget inside Node Attribute
                        for value in values:
                            if dpg.get_item_label(value) != '^ Remove ^':
                                #print(f"Tag = {value}, Label = {dpg.get_item_label(value)}, content = {str(dpg.get_value(value))}")
                                dpg.add_text(default_value=f"{dpg.get_item_label(value)} = {str(dpg.get_value(value))}"
                                                ,parent=properties_window,bullet=True,color=(0,255,0))
            
        # Check to see if the nodes are part of any connections
        dpg.add_text(default_value="Node Connections",parent=properties_window)            
        for key,values in link_info.items():
            if selected_node_id in values:
                if values.index(selected_node_id) == 1: # Meaning the connection starts from this node
                    dpg.add_text(default_value=f"{Node_label} --> {dpg.get_item_label(values[3])} through attributes \n",parent=properties_window,bullet=True,color=(0,0,255))
                    dpg.add_text(default_value=f"{dpg.get_item_label(values[0])} --> {dpg.get_item_label(values[2])} ",parent=properties_window,bullet=True,color=(255,0,0))
                else:
                    dpg.add_text(default_value=f"{Node_label} <-- {dpg.get_item_label(values[1])} through attributes \n",parent=properties_window,bullet=True,color=(0,0,255))
                    dpg.add_text(default_value=f"{dpg.get_item_label(values[2])} <-- {dpg.get_item_label(values[0])} ",parent=properties_window,bullet=True,color=(255,0,0))        

def json_creator_callback():
    raw_data = {}
    for node_tag in (dpg.get_item_children(NodeEditor)[1]):
        node_data = {}
        for key,values in (dpg.get_item_children(node_tag)).items(): # For Each Node
            for value in values: # For each Node attribute, NOT INCLUDING NODE ATTRIBUTE DETAILS INSIDE JSON
                attribute_data = {}
                attribute_key = value
                attribute_label = (dpg.get_item_label(value))
                #attribute_data["attribute"] = dpg.get_item_label(value)
                for key,values in (dpg.get_item_children(value)).items(): # For each widget inside Node Attribute
                        for value in values:
                            if dpg.get_item_label(value) != '^ Remove ^':
                                #print(f"Tag = {value}, Label = {dpg.get_item_label(value)}, content = {str(dpg.get_value(value))}")
                                attribute_data[dpg.get_item_label(value)] = (dpg.get_value(value)) # Dictionary of widgets inside attribute
                
                # Adding links as well
                for key,values in link_info.items():
                    if attribute_key in values: # 0 or 2nd pos
                        if values.index(attribute_key) == 0: # Meaning the connection starts from this node
                            attribute_data["Connected To"] = dpg.get_item_label(values[2])
                        else:
                            attribute_data["Connected To"] = dpg.get_item_label(values[0])

                node_data[attribute_label] = attribute_data
        raw_data[dpg.get_item_label(node_tag)] = node_data
    # Convert data dictionary to JSON
    json_data = json.dumps(raw_data, indent=4)

    # Write JSON data to a file
    with open("data.json", "w") as json_file:
        json_file.write(json_data)

    print("JSON Export complete!")

def clear_window(window_tag):
    # Clear the window by deleting all items
    #window_id = dpg.get_item_parent("Example Window")
    items = dpg.get_item_children(window_tag) # items is a dict
    for key,values in items.items():
        if len(values): # Check if not empty list
            for value in values:
                dpg.delete_item(value)

def random_number_gen():
    return ''.join(random.choices(string.ascii_lowercase+string.digits,k=7))

def publisher_node_callback():
    # Assuming NodeEditor is the ID of your node editor
    selected_nodes = dpg.get_selected_nodes(NodeEditor)
    if selected_nodes:
        selected_node_id = selected_nodes[0]  # Assuming only one node is selected
        with dpg.node_attribute(label=f"publisher@{random_number_gen()}",parent=selected_node_id,attribute_type=dpg.mvNode_Attr_Output): #Change label to type_#
            dpg.add_input_text(label="message_type",default_value='', width=250)
            dpg.add_input_text(label="topic_name", default_value='',width=250)
            dpg.add_input_int(label="queue_size", default_value=10, width=250)
            dpg.add_button(label="^ Remove ^",callback=remove_feature)

    else:
        print("Please select a node before adding content.")


def subscriber_node_callback():
    selected_nodes = dpg.get_selected_nodes(NodeEditor)
    if selected_nodes:
        selected_node_id = selected_nodes[0]  # Assuming only one node is selected
        with dpg.node_attribute(label=f"subscriber@{random_number_gen()}",parent=selected_node_id,attribute_type=dpg.mvNode_Attr_Input):
            dpg.add_input_text(label="message_type", default_value='',width=250)
            dpg.add_input_text(label="topic_name", default_value='',width=250)
            dpg.add_input_text(label="callback_function", default_value='',width=250)     
            dpg.add_input_int(label="queue_size",default_value=10,width=250)
            dpg.add_button(label="^ Remove ^",callback=remove_feature)

    else:
        print("Please select a node before adding content.")
def service_server_node_callback():
    selected_nodes = dpg.get_selected_nodes(NodeEditor)
    if selected_nodes:
        selected_node_id = selected_nodes[0]  # Assuming only one node is selected
        with dpg.node_attribute(label=f"service_server@{random_number_gen()}",parent=selected_node_id,attribute_type=dpg.mvNode_Attr_Input,shape=2):
            dpg.add_input_text(label="service_type", default_value='',width=250)
            dpg.add_input_text(label="Service_name", default_value='',width=250)
            dpg.add_input_text(label="callback_function", default_value='',width=250)     
            dpg.add_button(label="^ Remove ^",callback=remove_feature)

    else:
        print("Please select a node before adding content.")

def service_client_node_callback():
    selected_nodes = dpg.get_selected_nodes(NodeEditor)
    if selected_nodes:
        selected_node_id = selected_nodes[0]  # Assuming only one node is selected
        with dpg.node_attribute(label=f"service_client@{random_number_gen()}",parent=selected_node_id,attribute_type=dpg.mvNode_Attr_Output,shape=2):
            dpg.add_input_text(label="service_type", default_value='',width=250)
            dpg.add_input_text(label="service_name", default_value='',width=250)
            dpg.add_input_text(label="request_function", default_value='',width=250)     
            dpg.add_button(label="^ Remove ^",callback=remove_feature)

    else:
        print("Please select a node before adding content.")

def action_server_node_callback():
    selected_nodes = dpg.get_selected_nodes(NodeEditor)
    if selected_nodes:
        selected_node_id = selected_nodes[0]  # Assuming only one node is selected
        with dpg.node_attribute(label=f"action_server@{random_number_gen()}",parent=selected_node_id,attribute_type=dpg.mvNode_Attr_Input,shape=4):
            dpg.add_input_text(label="action_type", default_value='',width=250)
            dpg.add_input_text(label="action_name", default_value='',width=250)
            dpg.add_input_text(label="action_callback",width=250,hint="must return result message for the action type")     
            dpg.add_button(label="^ Remove ^",callback=remove_feature)

    else:
        print("Please select a node before adding content.")

def action_client_node_callback():
    selected_nodes = dpg.get_selected_nodes(NodeEditor)
    if selected_nodes:
        selected_node_id = selected_nodes[0]  # Assuming only one node is selected
        with dpg.node_attribute(label=f"action_client@{random_number_gen()}",parent=selected_node_id,attribute_type=dpg.mvNode_Attr_Output,shape=4):
            dpg.add_input_text(label="action_type", default_value='',width=250)
            dpg.add_input_text(label="action_name", default_value='',width=250)
            dpg.add_input_text(label="action_goal_function", default_value='', width=250)     
            dpg.add_input_text(label="action_goal_response_callback", default_value='', width=250)     
            dpg.add_input_text(label="action_get_result_callback", default_value='', width=250)     
            dpg.add_button(label="^ Remove ^",callback=remove_feature)

    else:
        print("Please select a node before adding content.")



def single_click_callback():
    pass

def double_clicker_callback(): # TODO: Currently renaming, make this open a side bar

    if len(dpg.get_selected_nodes(NodeEditor)) == 1: # Meaning one node selected
        with dpg.window(label="Rename Node",tag="Rename_Node"):
            dpg.add_input_text(label="Rename the node as ..",tag='renaming_tag',callback=rename_node_callback,on_enter=True)




## Topic specific nodes (ROS)
'''
Nodes should contain
1. Node Name
2. Additional Functionality of the Node (For pub, sub, static)
3. If receiving (Subs):-
    Callback for the received topic
4. If sending (pubs):-
    Name of Topic 
    Type of Topic
    Variable for data to be published
'''

## Service Specific nodes (ROS)
## Action Specific nodes (ROS)

## Non-ROS Nodes
'''
Nodes that don't have any ROS based Funcs

1. Just helper functions for the node's main function
2. Random Library Functionality (OpenCV and others)
3. Should contain a way to feed in data (Input)
4. Should contain a way to feed out data (Output)
'''

    # Mouse strokes

with dpg.handler_registry():
    #dpg.add_mouse_down_handler(callback=select_node_callback)
    dpg.add_mouse_double_click_handler(callback=double_clicker_callback)
    # dpg.add_key_press_handler()
    dpg.add_mouse_click_handler(callback=node_properties_callback)
    # dpg.add_mouse_drag_handler
    dpg.add_key_press_handler(key=dpg.mvKey_Delete, callback=delete_node_callback)
    # dpg.add_child_window()

    # Viewpoint stuff
viewport_id = dpg.create_viewport(title='NodeWeaver', width=1280, height=720)

    # Main Menu

with dpg.viewport_menu_bar():
    with dpg.menu(label="File"):
        dpg.add_menu_item(label="Save")
        dpg.add_menu_item(label="Save As")

        with dpg.menu(label="Settings"):
            dpg.add_menu_item(label="Setting 1")
            dpg.add_menu_item(label="Setting 2")
    with dpg.menu(label="Widget Items"):
        dpg.add_checkbox(label="Pick Me")
        dpg.add_button(label="Press Me")
        dpg.add_color_picker(label="Color Me")


    #Node Editor Window
with dpg.window(label="Canvas", width=800, height=720):

    # Node Editor menu bar
    with dpg.menu_bar() as canvas_menu_bar:
        with dpg.menu(tag='add_node_more',label="Add Nodes",parent=canvas_menu_bar):
            dpg.add_menu_item(label="Add Publisher Node",callback=publisher_node_callback)
            dpg.add_menu_item(label="Add Subscriber Node",callback=subscriber_node_callback)
            dpg.add_menu_item(label="Add Service Server Node",callback=service_server_node_callback)
            dpg.add_menu_item(label="Add Service Client Node",callback=service_client_node_callback)
            dpg.add_menu_item(label="Add Action Server Node",callback=action_server_node_callback)
            dpg.add_menu_item(label="Add Action Client Node",callback=action_client_node_callback)
        
        dpg.add_menu_item(tag='add_node',label="Add Node",callback=add_node_callback)

        dpg.add_menu_item(tag='generate_ros_pkg',label="Generate ROS Pkg")
        
        dpg.add_menu_item(tag='export_json',label="Export JSON",callback=json_creator_callback)

    with dpg.node_editor(callback=link_callback, delink_callback=delink_callback, minimap=True, minimap_location=dpg.mvNodeMiniMap_Location_BottomRight) as NodeEditor:
        with dpg.node(label="Node 1",tag="node_1"):
            with dpg.node_attribute(label="Node A1",attribute_type=dpg.mvNode_Attr_Output,shape=2):
                dpg.add_input_float(label="F1", width=150,tag='f1')

            with dpg.node_attribute(label="Node A2", attribute_type=dpg.mvNode_Attr_Output):
                dpg.add_input_float(label="F2", width=150)

        with dpg.node(label="Node 2",tag="node_2"):
            with dpg.node_attribute(label="Node A3"):
                dpg.add_input_float(label="F3", width=200)

            with dpg.node_attribute(label="Node A4", attribute_type=dpg.mvNode_Attr_Static):
                dpg.add_input_float(label="F4", width=200)


with dpg.window(tag='properties_window',label="Properties Window", pos=[800,0],width=480, height=720) as properties_window:
    pass


# dpg.set_viewport_min_scale(viewport_id,0.5) # Define minimum scale
# dpg.set_viewport_max_scale(viewport_id, 2.0)  # Define maximum scale
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()