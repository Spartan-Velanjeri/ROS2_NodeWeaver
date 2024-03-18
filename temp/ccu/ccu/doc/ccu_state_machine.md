# CONCEPT `CCU StateMachine Mission Control/Operational`

## Monitoring/Diagnosis

Each module contains of the following conditions:

| Conditions    | Description                                              |
| ------------- | -------------------------------------------------------- |
| isHomed       | Is true when the module is homed                         |
| isReady       | True when the module is able to receive commands         |
| hasWarning    | True when warning occuurs, module is still operational   |
| hasError      | True when error occuurs, module is not operational       |
| hasFatalError | True when fatal error occuurs human interaction required |

## RPM

### Conditions

| Conditions | Module       | Name                               | Description                                                                                    |
| ---------- | ------------ | ---------------------------------- | ---------------------------------------------------------------------------------------------- |
| hasWarning | RPM          | Battery low                        | Battery SoC is low charging required, base able to move, no drilliing                          |
| hasError   | RPM          | Battery empty                      | Battery is empty required not operational                                                      |
| hasError   | RPM          | Drive blocked                      | Drives are not able to move within current limits                                              |
| hasError   | RPM          | Drive communication errors         | Drives are not able to move within current limits                                              |
| hasWarning | Localization | localization error big             | The localization error is big RPM is able to navigate, repositioning for drilling may required |
| hasError   | Localization | localization error too big         | The localization error is too big RPM is not able to navigate                                  |
| hasWarning | Navigation   | Obstacle in current path           | obstacle within current planned path, replan                                                   |
| hasError   | Navigation   | Obstacle in current path           | obstacle within current planned path, no new plan found                                        |
| hasWarning | Navigation   | Obstacle at future drill position  | obstacle at future drill position, user is informed to remove the obstacle                     |
| hasError   | Navigation   | Obstacle at current drill position | obstacle atcurrent  drill position, navigation is cancled user is informed                     |

### Actions

| Task             | Description                                                  |
| ---------------- | ------------------------------------------------------------ |
| move to position | RPM is able to navigate to goal position                     |
| velocity command | manual mode RPM is controlled by operator via remote control |

## PTU

### Conditions PTU

| Conditions | Module | Name                 | Description                                                                     |
| ---------- | ------ | -------------------- | ------------------------------------------------------------------------------- |
| hasWarning | PTU    | High torque          | drilling effort is higher then expected                                         |
| hasError   | PTU    | Max Torque           | drilling effort is too high                                                     |
| hasWarning | PTU    | High temperatur      | temperature is higher then expected                                             |
| hasError   | PTU    | Max temperature      | temperature is too high                                                         |
| hasWarning | PTU    | drill bit wear       | drill bit has to be changed, changing drill bit is scheduled, still operational |
| hasError   | PTU    | drill bit locked     | drill bit is locked                                                             |
| hasError   | PTU    | drill bit broken     | drill bit needs to be replaced                                                  |
| hasError   | PTU    | drill depth exceeded | drill hole is deeper than expected                                              |

### Actions PTU

| Task                | Description                           |
| ------------------- | ------------------------------------- |
| drill speed command | drill machine is enabled and speed up |

## FPM

### Conditions FPM

| Conditions | Module                    | Name          | Description                                                                                           |
| ---------- | ------------------------- | ------------- | ----------------------------------------------------------------------------------------------------- |
| ishomed    | lifting unit              | liftinghomed  | the liftig unit is homed                                                                              |
| isready    | lifting unit              | liftingready  | the liftig unit is ready                                                                              |
| ishomed    | handling unit             | handlinghomed | the handling unit is homed                                                                            |
| isready    | handling unit             | handlingready | the handling unit is ready                                                                            |
| isready    | precise localization unit | preclocready  | the precise loacalization is ready                                                                    |
| hasError   | dust extraction           | Max level     | dust extraction container is full operator is requested to empty the container. No operation possible |
| hasWarning | dust extraction           | High level    | empty dust extraction container, operational                                                          |

### Actions FPM

| Task                                  | Description                                                                                                                                                                                               |
| ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| home lifting unit                     | homing of the lifting unit is triggered                                                                                                                                                                   |
| enable lifting unit                   | lifting unit enabled                                                                                                                                                                                      |
| move lifting unit                     | move lifting to desired position                                                                                                                                                                          |
| home handling unit                    | homing of the handling unit is triggered                                                                                                                                                                  |
| transport pose handling unit          | move handling to transport position                                                                                                                                                                       |
| lifting pose handling unit            | move handling to lifting position                                                                                                                                                                         |
| service pose handling unit            | move handling to service position for such as exchange PTU                                                                                                                                                |
| speed tcp cmd handling unit           | move handling to position                                                                                                                                                                                 |
| visualize tcp pose                    | visualize the drill position in real world                                                                                                                                                                |
| pose tcp cmd handling unit            | move handling to position                                                                                                                                                                                 |
| drill hole cmd handling unit          | drilling motion handling                                                                                                                                                                                  |
| drill pattern cmd cu                  | drilling pattern                                                                                                                                                                                          |
| start dust extraction                 | start dust extraction                                                                                                                                                                                     |
| stop dust extraction                  | stop dust extraction                                                                                                                                                                                      |
| mark succesfull drilled holes         | After the the drill process succeeded, mark the position with it's ID                                                                                                                                     |
| mark failed drilled holes             | If the drill process fails, mark the position                                                                                                                                                             |
| locate handling unit base             | Use the leica system to precisle locate the handling unit                                                                                                                                                 |
| check reachability of drill positions | based on the current RPM position, check if all start and end positions are within in the workspace of the handling unit, from start to end position configuration of the handling unit should not change |
| change drill bit                      | drill bit change motion handling unit                                                                                                                                                                     |
