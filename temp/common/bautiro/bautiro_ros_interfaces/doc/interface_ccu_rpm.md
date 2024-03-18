# Interface CCU <-> RPM
## 1. Technische Voraussetzungen

```text
┌─── Central CU ──────┐           ┌─── RPM CU ────────────┐
│  ┌── ROS2─────┐     │           │  ┌ ROS2  ────┐        │
│  │ ccu_node   │--------------------│ rpm_node  │        │
│  └────────────┘     │           │  └───────────┘        │
└─────────────────────┘           └───────────────────────┘
```

Für die Kommunikation zwischen `CCU` und `RPM` läuft auf der `Central Control Unit`
eine ROS2 Node (`ccu_node`) und auf der `RPM Control Unit` ein ROS2 Node (`ccu_node`)
als Endpoint.

## 2. DoD

### 2.1 DoD Network-Access-Schicht

- CCU `1` <-> `1` RPM
- LAN/ETHERNET

### 2.2 DoD Internet-Schicht

- CCU statische IP: `192.168.0.1`
- RPM-CU statische IP: `192.168.0.2`
- PORT an CCU: `8080`
- PORT an RPM: `7070`

### 2.3 DoD Host-Schicht

- Leitungsüberwachung
  - ergibt sich aus dem ROS2 Framework [e.g. QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
- Kommunikationsarten
  - ergibt sich aus dem ROS2 Framework [e.g. Topics](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#)

## 3. **Topics zwischen CCU und RPM**

Es gibt derzeit folgende
[Topic](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)(s)
 zwischen `ccu_node` und `rpm_node` :

| topic                   | type              | desc                          |
| ----------------------- | -----             | ----------------------------- |
| `/ccu/joystick_to_rpm`  | Float32MultiArray | Joystick-Signal an RPM        |
| `...`                   |                   |                               |

Die Einzelnen Array Felder hierbei

```py
x_axis  =  joystick_to_rpm[0]    # Wertebereich -1.0 .... +1.0
y_axis  =  joystick_to_rpm[1]    # Wertebereich -1.0 .... +1.0
```


## 4. **CCU-RPM Interface im Kontext**

![hcu-ccu-rpm-overview](img/ccu.drawio.svg)

## Funktion bzw. Zeitliches Verhalten bezüglich `/ccu/joystick_to_rpm`

Joystick-Signal von `HCU` via `/hcu_input/joystick`  wird sofort bei Eintreffen
(abhängig vom Zustand\*)
ohne Verzögerung weitergeleitet als `/ccu/joystick_to_rpm` Message an das RPM.
Dort dient es als Eingangs/Ansteuerungs-Größe der Endstufen (Ketten-Antrieb).

\* Zustand heute nur Manuelle RPM-Ansteuerung

## Topics zwischen HCU und CCU

Alle topics/Daten die für das HCU relevant sind werden vom CCU auf topics gepublished
die das prefix `/ccu_display/..` haben.

> | topic                         | type       | desc                          |
> | ----------------------------  | -----      | ----------------------------- |
> | `/ccu_display/heartbeat`      | Int64      | periodic keepalive signal     |
> | `/ccu_display/mode_main`      | Int32      | robot main operating mode?    |
> | `/ccu_display/battery_status` | Int32      | status code battery state     |
> | `/ccu_display/battery_level`  | Float32    | battery charge level          |

## CCU Services

Dei CCU Services > `republish()`

### republish

> `republish()`
>
> - (re)published letzte Nachrichten auf topics mit `/ccu_display/*`
> - returns an int response code: [CCU Service Response Codes](#ccu-service-response-codes)
> - Nutzbar um nach einem (re)connect up-to-date zu kommen

### command_api

> `command_api(int32)`
>
> - executes a parameterless function by functionID
>   - 0000 `stopNow`
>   - 0001 `stopAfterXY`
>   - 0101 `setManualModeRpmControl`
>   - 0102 `setManualModeLiftControl`
>   - 0103 `setManualModeDisabled`
>   - 9000 `enableDebugMode`
>   - 9001 `setDebugState_1`
>   - 9002 `setDebugState_2`
>   - 9003 `setDebugState_3`
> - returns an int response code: [CCU Service Response Codes](#ccu-service-response-codes)

## CCU Service Response Codes

> | code   | name                |
> | ------ | ------------------- |
> | `0000` | ok - general        |
> | `0001` | ok - republishing   |
> | `1000` | error - ...         |
> | `....` | error - ...         |

## HCU

### HCU Topics

Alle topics/Daten die vom HCU zum CCU gehen haben das prefix `/hcu_input/..`.

> | topic                   | type               | desc                          |
> | ----------------------- | -----              | ----------------------------- |
> | `/hcu_input/heartbeat`  | Int32              | periodic keepalive signal     |
> | `/hcu_input/joystick`   | Float32MultiArray  | joystick input axis           |