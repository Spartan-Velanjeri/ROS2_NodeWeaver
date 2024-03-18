#############################################
FPM Coordinator
#############################################

..
    headers
    ~~~~~~~~
    <<<<<<<<
    ~~~~~~~~
    --------


~~~~~~~~~~~~
Introduction
~~~~~~~~~~~~
FPM (fine positioning module) coordinator is the software module that implements following features:
    * Observes status of the FMP HW devices and reacts on state changes
    * Implements state/mode machine of the FPM module
    * Reacts on state transition requests from the CCU module

~~~~~~~~~~~~~~~~~~~~~~~~~~~
Summary of the requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~
bla bla

~~~~~~~~~~~~~~~~~~~~~~~
Architecture and design
~~~~~~~~~~~~~~~~~~~~~~~
bla bla

Hardware devices
<<<<<<<<<<<<<<<<
FPM observes state of the following units (devices):
    * Lift
    * Robot
    * Vacuum
    * PTU (drill unit)
    * Leica
    * Ceiling sensor
    * Inclination sensor


State/mode machine
<<<<<<<<<<<<<<<<<<

FPM coordinator implements the following states of the FPM unit:
 .. image:: img/states_table.drawio.png

States
~~~~~~~~~~~~~~~~
Conditions for transitions from current into "target" state is described in the chapters
below. For every state are given possible transitions and conditions for the transition.
State of the units are described with the following legend:

 .. image:: img/legend.drawio.png


NOT_READY
*********
Initial state after the system is powered on. Possible transitions to the following states:
    * READY
    * FAULT

 .. image:: img/not_ready.drawio.png


READY
*****
Possible transitions to the following states:
    * TRANSPORT
    * SERVICE
    * MANUAL
    * AUTOMATIC
    * FAULT

 .. image:: img/ready.drawio.png

TRANSPORT
*********
Possible transitions to the following states:
    * SERVICE
    * MANUAL
    * AUTOMATIC
    * FAULT

 .. image:: img/transport.drawio.png

SERVICE
*******
Possible transitions to the following states:
    * READY
    * TRANSPORT
    * MANUAL
    * AUTOMATIC
    * FAULT

..
 .. image:: img/service.drawio.png

MANUAL
******
Possible transitions to the following states:
    * TRANSPORT
    * SERVICE
    * AUTOMATIC
    * FAULT

..
 .. image:: img/manual.drawio.png

AUTOMATIC
*********
Possible transitions to the following states:
    * READY
    * FAULT

..
 .. image:: img/automatic.drawio.png


External Interfaces
<<<<<<<<<<<<<<<<<<<



Tests
<<<<<
