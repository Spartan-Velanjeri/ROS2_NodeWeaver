# Python Package ccu_util - up-to-come

> **Utilities _for_**
>
> - `ccu_dataservice`
> - `ccu_data_services`
> - `ccu_hcu_abstraction`
>
> _helpers, utilities, static functions_

## TODO

1. - Konvertierung von `Matrix4x4` .. nach `tm`, ... nach `Pose`
     Konvertierung von `Pose`      .. nach `tm`, ... nach `Matrix4x4`

## Dependencies

The libs make usage of - **thus has dependencies** - to

| Typ/Msg           | Package                | origin           |
|------------------ |----------------------- |----------------- |
| Matrix4x4, ...    | ccu_grpc               |                  |
| CMatrix, ...      | ccu_bBautiro           |                  |
| Pose              | geometry_msgs.msg      | ROS              |
| DrillHole         | bautiro_ros_interfaces | bautiro_common   |
