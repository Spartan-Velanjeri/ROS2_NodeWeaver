import dearpygui.dearpygui as dpg

dpg.create_context()

# callback runs when user attempts to connect attributes
def link_callback(sender, app_data):
    # app_data -> (link_id1, link_id2)
    dpg.add_node_link(app_data[0], app_data[1], parent=sender)

# callback runs when user attempts to disconnect attributes
def delink_callback(sender, app_data):
    # app_data -> link_id
    dpg.delete_item(app_data)

def add_button_callback():
    with dpg.node(label="alpha",parent=NodeEditor):
        dpg.add_node_attribute(label="beta",attribute_type=dpg.mvNode_Attr_Output)

with dpg.window(label="Canvas", width=400, height=400):
    dpg.add_button(label="Add Node", callback=add_button_callback)

    with dpg.node_editor(callback=link_callback, delink_callback=delink_callback, minimap=True, minimap_location=dpg.mvNodeMiniMap_Location_BottomRight) as NodeEditor:
        with dpg.node(label="Node 1"):
            with dpg.node_attribute(label="Node A1",attribute_type=dpg.mvNode_Attr_Output):
                dpg.add_input_float(label="F1", width=150)

            with dpg.node_attribute(label="Node A2", attribute_type=dpg.mvNode_Attr_Output):
                dpg.add_input_float(label="F2", width=150)

        with dpg.node(label="Node 2"):
            with dpg.node_attribute(label="Node A3"):
                dpg.add_input_float(label="F3", width=200)

            with dpg.node_attribute(label="Node A4", attribute_type=dpg.mvNode_Attr_Static):
                dpg.add_input_float(label="F4", width=200)

viewport_id = dpg.create_viewport(title='NodeWeaver', width=800, height=600)
# dpg.set_viewport_min_scale(viewport_id,0.5) # Define minimum scale
# dpg.set_viewport_max_scale(viewport_id, 2.0)  # Define maximum scale
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()