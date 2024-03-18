# **Node:** fpm_plc_driver

This Node offers

|               | topic                       | type                      | defintion        |
| ------------- | --------------------------- | ------------------------- | ---------------- |
|               |                             |                           |                  |
| pub           | `/heartbeat`                | `[Int64]`                 | `std_msgs`       |
| pub           | `/lift_status`              | `[Int32]`                 | `std_msgs`       |
| pub           | `/fpm_??`                   | `[Float32]`               | `std_msgs`       |
|               |                             |                           |                  |
| service       | `/move_lift_absolute_start` | `[MoveLiftAbsoluteStart]` | `bautiro_common` |
| service       | `/move_lift_absolute_stop`  | `[MoveLiftAbsoluteStop]`  | `bautiro_common` |
|               |                             |                           |                  |
| action-server | `/move_lift_absolute`       | `[MoveLiftAbsolute]`      | `bautiro_common` |

## **Action-Server `/move_lift_absolute`** [`[CcuMode.msg]`]

## `ResponseCode.msg` _(bautiro_ros_interfaces)_

used Codes in this node

```text
SERVICE_ALREADY_BUSY_PARALLEL_CALL_NOT_ALLOWED
ORIGINALSERVER_NOTREADY_TIMEOUT
ORIGINALSERVER_NOTACCEPTS_GOALREQUEST_NOFURTHERINFO

1000 # TODO !
```
