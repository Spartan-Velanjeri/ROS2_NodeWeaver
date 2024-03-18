# Protocol Documentation
<a name="top"></a>

## Table of Contents

- [base_layer_service.proto](#base_layer_service-proto)
    - [BaseLayerState](#google-protobuf-BaseLayerState)
    - [BautiroDescriptionMessage](#google-protobuf-BautiroDescriptionMessage)
    - [BautiroUrdfLink](#google-protobuf-BautiroUrdfLink)
    - [BautiroUrdfVisualMesh](#google-protobuf-BautiroUrdfVisualMesh)
    - [BautiroVisualFileRequest](#google-protobuf-BautiroVisualFileRequest)
    - [BoolParameter](#google-protobuf-BoolParameter)
    - [DeveloperFunctionCallResult](#google-protobuf-DeveloperFunctionCallResult)
    - [DeveloperFunctionDefinition](#google-protobuf-DeveloperFunctionDefinition)
    - [DeveloperFunctionList](#google-protobuf-DeveloperFunctionList)
    - [DeveloperFunctionSource](#google-protobuf-DeveloperFunctionSource)
    - [DirectoryMetaData](#google-protobuf-DirectoryMetaData)
    - [DirectoryStreamingMessage](#google-protobuf-DirectoryStreamingMessage)
    - [DirectoryStreamingResponse](#google-protobuf-DirectoryStreamingResponse)
    - [DirectoryTransferCompletedMessage](#google-protobuf-DirectoryTransferCompletedMessage)
    - [DrillHole](#google-protobuf-DrillHole)
    - [DrillJob](#google-protobuf-DrillJob)
    - [DrillMask](#google-protobuf-DrillMask)
    - [FastenerTemplate](#google-protobuf-FastenerTemplate)
    - [FileMetaData](#google-protobuf-FileMetaData)
    - [FileStreamingMessage](#google-protobuf-FileStreamingMessage)
    - [FileTransferCompletedMessage](#google-protobuf-FileTransferCompletedMessage)
    - [Job](#google-protobuf-Job)
    - [MeasureJob](#google-protobuf-MeasureJob)
    - [MissionDataResponse](#google-protobuf-MissionDataResponse)
    - [MissionIdRequest](#google-protobuf-MissionIdRequest)
    - [MissionList](#google-protobuf-MissionList)
    - [MissionMetaData](#google-protobuf-MissionMetaData)
    - [MoveJob](#google-protobuf-MoveJob)
    - [Node](#google-protobuf-Node)
    - [NumericParameter](#google-protobuf-NumericParameter)
    - [ParameterDefinition](#google-protobuf-ParameterDefinition)
    - [RobotTransform](#google-protobuf-RobotTransform)
    - [StartMissionResponse](#google-protobuf-StartMissionResponse)
    - [UpdateTransformMessage](#google-protobuf-UpdateTransformMessage)
  
    - [DrillHole.DrillHoleState](#google-protobuf-DrillHole-DrillHoleState)
  
    - [BaseLayerService](#google-protobuf-BaseLayerService)
  
- [common_types.proto](#common_types-proto)
    - [Matrix4x4](#google-protobuf-Matrix4x4)
  
- [manual_mode_service.proto](#manual_mode_service-proto)
    - [ControlState](#google-protobuf-ControlState)
    - [DriveArmControlMessage](#google-protobuf-DriveArmControlMessage)
    - [DriveRpmControlMessage](#google-protobuf-DriveRpmControlMessage)
    - [SetStateMessage](#google-protobuf-SetStateMessage)
    - [SetStateMessageResponse](#google-protobuf-SetStateMessageResponse)
    - [SetStateMessageResponse.SetStateFinishedMessage](#google-protobuf-SetStateMessageResponse-SetStateFinishedMessage)
    - [SetStateMessageResponse.SetStateProgressMessage](#google-protobuf-SetStateMessageResponse-SetStateProgressMessage)
    - [StartDrillMessage](#google-protobuf-StartDrillMessage)
    - [StartDrillMessage.ManualDrillHole](#google-protobuf-StartDrillMessage-ManualDrillHole)
    - [StartDrillResponse](#google-protobuf-StartDrillResponse)
    - [StartDrillResponse.DrillFinishedMessage](#google-protobuf-StartDrillResponse-DrillFinishedMessage)
  
    - [DriveArmControlMessage.Direction](#google-protobuf-DriveArmControlMessage-Direction)
    - [SetStateMessage.TargetState](#google-protobuf-SetStateMessage-TargetState)
  
    - [ManualModeService](#google-protobuf-ManualModeService)
  
- [Scalar Value Types](#scalar-value-types)



<a name="base_layer_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## base_layer_service.proto



<a name="google-protobuf-BaseLayerState"></a>

### BaseLayerState
contains information about the current capabilities


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| can_switch_to_manual_mode | [bool](#bool) |  | Is Bautiro state ready to switch to manual mode |
| can_start_mission | [bool](#bool) |  | Is Bautiro state ready to accept a new mission. This will enable/disable start buttons in UI |
| mission_in_progress | [MissionMetaData](#google-protobuf-MissionMetaData) | optional | Holds Metadata about the current mission in progress |






<a name="google-protobuf-BautiroDescriptionMessage"></a>

### BautiroDescriptionMessage
Message describing the robot


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| links | [BautiroUrdfLink](#google-protobuf-BautiroUrdfLink) | repeated | collection of links |






<a name="google-protobuf-BautiroUrdfLink"></a>

### BautiroUrdfLink
a single link


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  | Name of the link. If name==&#34;base_link&#34;, parent_id is empty and this is the root |
| parent | [string](#string) |  | Name of the parent. Empty if root link |
| transform_relative_to_parent | [Matrix4x4](#google-protobuf-Matrix4x4) |  |  |
| visuals | [BautiroUrdfVisualMesh](#google-protobuf-BautiroUrdfVisualMesh) | repeated | collection of visuals |






<a name="google-protobuf-BautiroUrdfVisualMesh"></a>

### BautiroUrdfVisualMesh
A visual representation


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| mesh_file_path | [string](#string) |  |  |
| mesh_file_md5_hash | [string](#string) |  |  |
| offset | [Matrix4x4](#google-protobuf-Matrix4x4) |  |  |






<a name="google-protobuf-BautiroVisualFileRequest"></a>

### BautiroVisualFileRequest
Request to download a visual file (.obj or .mtl)


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| file_path | [string](#string) |  |  |






<a name="google-protobuf-BoolParameter"></a>

### BoolParameter
defines a boolean parameter


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| param_name | [string](#string) |  | name of the parameter |
| param_value | [bool](#bool) |  | value of the parameter. Will also be used as the default value |






<a name="google-protobuf-DeveloperFunctionCallResult"></a>

### DeveloperFunctionCallResult
result of the function call. will be shown in GUI


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| message | [string](#string) |  |  |






<a name="google-protobuf-DeveloperFunctionDefinition"></a>

### DeveloperFunctionDefinition
defines a developer function


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  | name and identifier of the function. Unique for every function. |
| description | [string](#string) |  | short description |
| parameters | [ParameterDefinition](#google-protobuf-ParameterDefinition) | repeated | list of the function&#39;s parameters |






<a name="google-protobuf-DeveloperFunctionList"></a>

### DeveloperFunctionList
contains a list of developer fucntions


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| sources | [DeveloperFunctionSource](#google-protobuf-DeveloperFunctionSource) | repeated | list of sources of dev functions |






<a name="google-protobuf-DeveloperFunctionSource"></a>

### DeveloperFunctionSource
A source of developer functions


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  | title/header of this source. A short string |
| functions | [DeveloperFunctionDefinition](#google-protobuf-DeveloperFunctionDefinition) | repeated | the list of functions |






<a name="google-protobuf-DirectoryMetaData"></a>

### DirectoryMetaData
Describes a directory before transfer


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| directory_name | [string](#string) |  |  |
| directory_file_count | [int64](#int64) |  |  |






<a name="google-protobuf-DirectoryStreamingMessage"></a>

### DirectoryStreamingMessage
Streams a directory&#39;s content.
Starts with a DirectoryMetaData.
Then for each file iterating on FileStreamingMessage
Then DirectoryTransferCompletedMessage when done.


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| dir_meta_data | [DirectoryMetaData](#google-protobuf-DirectoryMetaData) |  | once in the beginning |
| file_data | [FileStreamingMessage](#google-protobuf-FileStreamingMessage) |  | iterated per file |
| dir_completed | [DirectoryTransferCompletedMessage](#google-protobuf-DirectoryTransferCompletedMessage) |  | once when done |






<a name="google-protobuf-DirectoryStreamingResponse"></a>

### DirectoryStreamingResponse
used by rpc ImportMission, when import is done


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| success | [bool](#bool) |  | true if import/directory transfer was a success |






<a name="google-protobuf-DirectoryTransferCompletedMessage"></a>

### DirectoryTransferCompletedMessage
Sent when a directory transfer is done






<a name="google-protobuf-DrillHole"></a>

### DrillHole
A Drillhole in the nodetree, attached to a DrillMask


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| local_transform | [Matrix4x4](#google-protobuf-Matrix4x4) |  | this holes&#39;s transform relative to the drillmask |
| name_unique_per_drillmask | [string](#string) |  | Drillhole&#39;s unique name. Unique per Drillmask, but not per NodeTree |
| state | [DrillHole.DrillHoleState](#google-protobuf-DrillHole-DrillHoleState) |  | state of the hole |
| parent_drillmask_name_unique | [string](#string) | optional | the drillmask&#39;s parent (not supplied in manual mode) |
| d_x | [double](#double) |  |  |
| d_y | [double](#double) |  |  |






<a name="google-protobuf-DrillJob"></a>

### DrillJob



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| drillmask_names | [string](#string) | repeated |  |






<a name="google-protobuf-DrillMask"></a>

### DrillMask
A DrillMask in the nodetree, attached to a Fastenertemplate


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| local_transform | [Matrix4x4](#google-protobuf-Matrix4x4) |  | this drillMask&#39;s transform relative to fastenerTemplate&#39;s Node parent. (FastenerTemplate has no transform itself) |
| name_unique_per_node_tree | [string](#string) |  | Drillmask&#39;s unique name |
| drill_holes | [DrillHole](#google-protobuf-DrillHole) | repeated | List of holes to drill |
| parent_fastener_template_name_unique | [string](#string) |  | name of the DrillMask&#39;s parent. |






<a name="google-protobuf-FastenerTemplate"></a>

### FastenerTemplate
A FastenerTemplate in the nodetree, attached to a node


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| drill_masks | [DrillMask](#google-protobuf-DrillMask) | repeated | drillmasks contained in FastenerTemplate |
| name_unique_per_node_tree | [string](#string) |  | FastenerTemplate&#39;s unique name. |
| type_guid | [string](#string) |  |  |
| parent_node_name_unique | [string](#string) |  | name of the FastenerTemplate&#39;s parent. |






<a name="google-protobuf-FileMetaData"></a>

### FileMetaData
Describes a file before transfer


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| file_name | [string](#string) |  |  |
| file_size_bytes | [int64](#int64) |  |  |
| md5_hash | [string](#string) |  |  |






<a name="google-protobuf-FileStreamingMessage"></a>

### FileStreamingMessage
streams any file. Starts with a FileMetaData, then bytes while streaming file content, then FileTransferCompletedMessage when done.


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| file_meta_data | [FileMetaData](#google-protobuf-FileMetaData) |  |  |
| file_chunk_data | [bytes](#bytes) |  |  |
| file_completed | [FileTransferCompletedMessage](#google-protobuf-FileTransferCompletedMessage) |  |  |






<a name="google-protobuf-FileTransferCompletedMessage"></a>

### FileTransferCompletedMessage
Sent when a file transfer is done


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| file_name | [string](#string) |  |  |






<a name="google-protobuf-Job"></a>

### Job



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| id_unique | [string](#string) |  |  |
| absolute_transform | [Matrix4x4](#google-protobuf-Matrix4x4) |  |  |
| enabled | [bool](#bool) |  | this cluster won&#39;t be scheduled |
| drill_job | [DrillJob](#google-protobuf-DrillJob) |  |  |
| move_job | [MoveJob](#google-protobuf-MoveJob) |  |  |
| measure_job | [MeasureJob](#google-protobuf-MeasureJob) |  |  |






<a name="google-protobuf-MeasureJob"></a>

### MeasureJob
not used in production






<a name="google-protobuf-MissionDataResponse"></a>

### MissionDataResponse
deep mission data, aka nodetree. returned by GetMissionData and StreamMissionData


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| node_tree | [Node](#google-protobuf-Node) | repeated | the root nodes of this nodeTree |
| jobs | [Job](#google-protobuf-Job) | repeated | list of jobs in order of operation. Disabled jobs are at the end of the array |
| room_plan_md5 | [string](#string) |  | md5 hash of linked roomplan obj file |






<a name="google-protobuf-MissionIdRequest"></a>

### MissionIdRequest
simple request containing a mission ID


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| mission_id | [int64](#int64) |  | the unique mission id |






<a name="google-protobuf-MissionList"></a>

### MissionList
list containing all missions. Not deep data, just metadata


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| missions | [MissionMetaData](#google-protobuf-MissionMetaData) | repeated | list of all missions stored on CCU |
| current_active_mission_id | [int64](#int64) |  | id of current mission. -1 if not set. |






<a name="google-protobuf-MissionMetaData"></a>

### MissionMetaData
Metadata for a single mission


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| id | [int64](#int64) |  | unique mission id |
| name | [string](#string) |  | mission name, for visual use only |
| predecessor_id | [int64](#int64) |  | mission id this mission is dervied from. -1 if not set. |
| import_date | [Timestamp](#google-protobuf-Timestamp) |  | timestamp of when the mission was imported to CCU |
| room_plan_md5 | [string](#string) |  | md5 hash of linked roomplan obj file |






<a name="google-protobuf-MoveJob"></a>

### MoveJob







<a name="google-protobuf-Node"></a>

### Node
A node in the nodetree


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| children | [Node](#google-protobuf-Node) | repeated | node&#39;s children (if any) |
| local_transform | [Matrix4x4](#google-protobuf-Matrix4x4) |  | transform relative to node&#39;s parent |
| name_unique_per_node_tree | [string](#string) |  | node&#39;s unique name |
| series | [string](#string) |  | node&#39;s series |
| fastener_template | [FastenerTemplate](#google-protobuf-FastenerTemplate) | optional | the fastener template (if set) |
| parent_node_name_unique | [string](#string) | optional | name of the node&#39;s parent. Can be unset if node is root. |






<a name="google-protobuf-NumericParameter"></a>

### NumericParameter
defines a numeric parameter


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| param_name | [string](#string) |  | name of the parameter |
| param_value | [double](#double) |  | value of the parameter. Will also be used as the default value |
| min_value | [double](#double) |  |  |
| max_value | [double](#double) |  |  |
| step_size | [double](#double) |  | step size for increment/decrement in GUI |






<a name="google-protobuf-ParameterDefinition"></a>

### ParameterDefinition
defines a parameter for a developer function


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| param_number | [NumericParameter](#google-protobuf-NumericParameter) |  | numeric parameter |
| param_bool | [BoolParameter](#google-protobuf-BoolParameter) |  | bool parameter |






<a name="google-protobuf-RobotTransform"></a>

### RobotTransform
Message to update the robot&#39;s transformations


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| links | [UpdateTransformMessage](#google-protobuf-UpdateTransformMessage) | repeated | Collection of links that changed. |






<a name="google-protobuf-StartMissionResponse"></a>

### StartMissionResponse
response to StartMission rpc


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| success | [bool](#bool) |  |  |






<a name="google-protobuf-UpdateTransformMessage"></a>

### UpdateTransformMessage
Contains a transformation update of a single link


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| child_id | [string](#string) |  | this is the name of the element to be transformed |
| transform_relative_to_parent | [Matrix4x4](#google-protobuf-Matrix4x4) |  |  |





 


<a name="google-protobuf-DrillHole-DrillHoleState"></a>

### DrillHole.DrillHoleState


| Name | Number | Description |
| ---- | ------ | ----------- |
| UNDEFINED | 0 |  |
| UNDRILLED | 1 |  |
| SCHEDULED | 2 |  |
| FINISHED | 3 |  |
| IGNORE | 4 |  |
| FAILED | 5 |  |


 

 


<a name="google-protobuf-BaseLayerService"></a>

### BaseLayerService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| BaseLayerStateStream | [Empty](#google-protobuf-Empty) | [BaseLayerState](#google-protobuf-BaseLayerState) stream |  |
| GetMissionList | [Empty](#google-protobuf-Empty) | [MissionList](#google-protobuf-MissionList) | get the list of available missions |
| GetRoomPlanObjFile | [MissionIdRequest](#google-protobuf-MissionIdRequest) | [FileStreamingMessage](#google-protobuf-FileStreamingMessage) stream | get a roomplan for a specific mission |
| GetMissionData | [MissionIdRequest](#google-protobuf-MissionIdRequest) | [MissionDataResponse](#google-protobuf-MissionDataResponse) | get data for a specific mission |
| StreamMissionData | [Empty](#google-protobuf-Empty) | [MissionDataResponse](#google-protobuf-MissionDataResponse) stream | stream changes to mission data - a.k.a. &#34;Tracking mode&#34; |
| StartMission | [MissionIdRequest](#google-protobuf-MissionIdRequest) | [StartMissionResponse](#google-protobuf-StartMissionResponse) | start a mission and stream mission events |
| ImportMission | [DirectoryStreamingMessage](#google-protobuf-DirectoryStreamingMessage) stream | [DirectoryStreamingResponse](#google-protobuf-DirectoryStreamingResponse) | HCU sends files contained in a directory to CCU in order to create a new mission |
| GetBautiroDescription | [Empty](#google-protobuf-Empty) | [BautiroDescriptionMessage](#google-protobuf-BautiroDescriptionMessage) | get a description of the bautiro transform tree |
| GetBautiroMesh | [BautiroVisualFileRequest](#google-protobuf-BautiroVisualFileRequest) | [FileStreamingMessage](#google-protobuf-FileStreamingMessage) stream | download a visual mesh file |
| StreamBautiroTransforms | [Empty](#google-protobuf-Empty) | [RobotTransform](#google-protobuf-RobotTransform) stream | stream live transforms |
| GetDeveloperFunctions | [Empty](#google-protobuf-Empty) | [DeveloperFunctionList](#google-protobuf-DeveloperFunctionList) | get all developer functions |
| CallDeveloperFunction | [DeveloperFunctionDefinition](#google-protobuf-DeveloperFunctionDefinition) | [DeveloperFunctionCallResult](#google-protobuf-DeveloperFunctionCallResult) | call a developer function |

 



<a name="common_types-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## common_types.proto



<a name="google-protobuf-Matrix4x4"></a>

### Matrix4x4
Homogenous Matrix 4x4


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| r1c1 | [double](#double) |  |  |
| r1c2 | [double](#double) |  |  |
| r1c3 | [double](#double) |  |  |
| r1c4 | [double](#double) |  |  |
| r2c1 | [double](#double) |  |  |
| r2c2 | [double](#double) |  |  |
| r2c3 | [double](#double) |  |  |
| r2c4 | [double](#double) |  |  |
| r3c1 | [double](#double) |  |  |
| r3c2 | [double](#double) |  |  |
| r3c3 | [double](#double) |  |  |
| r3c4 | [double](#double) |  |  |
| r4c1 | [double](#double) |  | X position |
| r4c2 | [double](#double) |  | Y position |
| r4c3 | [double](#double) |  | Z position |
| r4c4 | [double](#double) |  |  |





 

 

 

 



<a name="manual_mode_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## manual_mode_service.proto



<a name="google-protobuf-ControlState"></a>

### ControlState
contains information of currently available actions


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| can_drive_rpm | [bool](#bool) |  | Can the rpm be driven currently |
| can_raise_lift | [bool](#bool) |  | Can the lift be raised currently (= Can the state be set to FINE_POSITIONING) |
| can_lower_lift | [bool](#bool) |  | Can the lift be lowered currently (= Can the state be set to DRIVING) |
| can_move_arm | [bool](#bool) |  | Can the arm be moved manually |






<a name="google-protobuf-DriveArmControlMessage"></a>

### DriveArmControlMessage
send move commands to the robotic arm


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| direction | [DriveArmControlMessage.Direction](#google-protobuf-DriveArmControlMessage-Direction) |  | direction, relative to base_link |
| speed_mult | [float](#float) |  | a float from 0 to 1 that is a multiplier for the speed applied |






<a name="google-protobuf-DriveRpmControlMessage"></a>

### DriveRpmControlMessage
Contains rpm driving commands


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| forward_backward_axis | [float](#float) |  | a float from -1 to 1 where -1 is backwards and 1 is forward |
| left_right_axis | [float](#float) |  | a float from -1 to 1 where -1 is right and 1 is left |
| speed_mult | [float](#float) |  | a float from 0 to 1 that is a multiplier for the speed applied |






<a name="google-protobuf-SetStateMessage"></a>

### SetStateMessage
Sets a specific state for manual control


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| target_state | [SetStateMessage.TargetState](#google-protobuf-SetStateMessage-TargetState) |  | requested state |






<a name="google-protobuf-SetStateMessageResponse"></a>

### SetStateMessageResponse
response after sending a SetStateMessage.
While the command is in progress, a SetStateProgressMessage is sent.
When done, a SetStateFinishedMessage is sent.


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| progress | [SetStateMessageResponse.SetStateProgressMessage](#google-protobuf-SetStateMessageResponse-SetStateProgressMessage) |  | progress update of current setstate action |
| finished | [SetStateMessageResponse.SetStateFinishedMessage](#google-protobuf-SetStateMessageResponse-SetStateFinishedMessage) |  | sent when setState is done or failed |






<a name="google-protobuf-SetStateMessageResponse-SetStateFinishedMessage"></a>

### SetStateMessageResponse.SetStateFinishedMessage
Sent when done setting state


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| success | [bool](#bool) |  | was setting state a success |






<a name="google-protobuf-SetStateMessageResponse-SetStateProgressMessage"></a>

### SetStateMessageResponse.SetStateProgressMessage
Sent while the state is being set


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| progress | [float](#float) |  | progress update of current setstate action |






<a name="google-protobuf-StartDrillMessage"></a>

### StartDrillMessage
Command to start drilling


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| drill_holes | [StartDrillMessage.ManualDrillHole](#google-protobuf-StartDrillMessage-ManualDrillHole) | repeated | List of holes to drill |






<a name="google-protobuf-StartDrillMessage-ManualDrillHole"></a>

### StartDrillMessage.ManualDrillHole
Definition of a hole to drill


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| local_transform | [Matrix4x4](#google-protobuf-Matrix4x4) |  | this holes&#39;s transform relative to the drillmask |
| name | [string](#string) |  |  |
| depth_meter | [double](#double) |  |  |
| diameter_meter | [double](#double) |  |  |






<a name="google-protobuf-StartDrillResponse"></a>

### StartDrillResponse
Respone while/after drilling


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| progress | [float](#float) |  | When in progress, float from 0-1 |
| finished | [StartDrillResponse.DrillFinishedMessage](#google-protobuf-StartDrillResponse-DrillFinishedMessage) |  | When done |






<a name="google-protobuf-StartDrillResponse-DrillFinishedMessage"></a>

### StartDrillResponse.DrillFinishedMessage
Response when Drilling is done


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| success | [bool](#bool) |  |  |





 


<a name="google-protobuf-DriveArmControlMessage-Direction"></a>

### DriveArmControlMessage.Direction


| Name | Number | Description |
| ---- | ------ | ----------- |
| LEFT | 0 |  |
| RIGHT | 1 |  |
| FORWARD | 2 |  |
| BACKWARD | 3 |  |



<a name="google-protobuf-SetStateMessage-TargetState"></a>

### SetStateMessage.TargetState


| Name | Number | Description |
| ---- | ------ | ----------- |
| UNUSED | 0 | placeholder |
| DRIVING | 10 | mode for driving manually |
| FINE_POSITIONING | 20 | mode for positioning Arm |


 

 


<a name="google-protobuf-ManualModeService"></a>

### ManualModeService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| ControlStateStream | [Empty](#google-protobuf-Empty) | [ControlState](#google-protobuf-ControlState) stream | server streams the current state of allowed actions |
| DriveRpmControlStream | [DriveRpmControlMessage](#google-protobuf-DriveRpmControlMessage) stream | [Empty](#google-protobuf-Empty) | client streams RPM drive commands |
| DriveArmControl | [DriveArmControlMessage](#google-protobuf-DriveArmControlMessage) | [Empty](#google-protobuf-Empty) | unary Arm move commands. returns when the arm has stopped moving |
| SetState | [SetStateMessage](#google-protobuf-SetStateMessage) | [SetStateMessageResponse](#google-protobuf-SetStateMessageResponse) stream | client requests a state change. Service call streams progress and ends with a SetStateFinishedMessage on completion or fail |
| StartDrill | [StartDrillMessage](#google-protobuf-StartDrillMessage) | [StartDrillResponse](#google-protobuf-StartDrillResponse) stream | start drilling at the current tool position |
| Stop | [Empty](#google-protobuf-Empty) | [Empty](#google-protobuf-Empty) | Stop whatever bautiro is doing |

 



## Scalar Value Types

| .proto Type | Notes | C++ | Java | Python | Go | C# | PHP | Ruby |
| ----------- | ----- | --- | ---- | ------ | -- | -- | --- | ---- |
| <a name="double" /> double |  | double | double | float | float64 | double | float | Float |
| <a name="float" /> float |  | float | float | float | float32 | float | float | Float |
| <a name="int32" /> int32 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint32 instead. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="int64" /> int64 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint64 instead. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="uint32" /> uint32 | Uses variable-length encoding. | uint32 | int | int/long | uint32 | uint | integer | Bignum or Fixnum (as required) |
| <a name="uint64" /> uint64 | Uses variable-length encoding. | uint64 | long | int/long | uint64 | ulong | integer/string | Bignum or Fixnum (as required) |
| <a name="sint32" /> sint32 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int32s. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="sint64" /> sint64 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int64s. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="fixed32" /> fixed32 | Always four bytes. More efficient than uint32 if values are often greater than 2^28. | uint32 | int | int | uint32 | uint | integer | Bignum or Fixnum (as required) |
| <a name="fixed64" /> fixed64 | Always eight bytes. More efficient than uint64 if values are often greater than 2^56. | uint64 | long | int/long | uint64 | ulong | integer/string | Bignum |
| <a name="sfixed32" /> sfixed32 | Always four bytes. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="sfixed64" /> sfixed64 | Always eight bytes. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="bool" /> bool |  | bool | boolean | boolean | bool | bool | boolean | TrueClass/FalseClass |
| <a name="string" /> string | A string must always contain UTF-8 encoded or 7-bit ASCII text. | string | String | str/unicode | string | string | string | String (UTF-8) |
| <a name="bytes" /> bytes | May contain any arbitrary sequence of bytes. | string | ByteString | str | []byte | ByteString | string | String (ASCII-8BIT) |

