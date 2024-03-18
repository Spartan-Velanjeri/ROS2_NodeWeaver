# FPM Overview

Software documentation for the fine positioning module (FPM). General processes and architecture are described, in particular common informations shared between the submodules. Informations related to a single submodule are documented in the dedicated submodule section.

The following submodules are existing:
  - **Coordinator FPM**: High level commands 
The FPM module consist of following sub modules:
- **Motion manager**: manages the handling and lifiting unit, planing and execution
- **Power tool unit (PTU)**: Contains power tool and it's control
- **Fine localization**: precised localization based on a tachymeter 
- **Ceiling profile detection**: inspection of the workspace and surrounding area beneath the ceiling.



**Upcoming features/modules:**
- **marking**: marking holes/pattern at the ceiling
- **automatic tool change**: drill bit changer, involves commads for handling unit, PTU and drill bit changer


##Outline:
[Signal and power design](signals_and_power_diagram.md): Diagrams illustrating modules and interfaces 
[FPM Drill sequence and control](fpm_drill_sequence_and_control.md): Drilling a hole comprises, handling unit, power tool unit. Commands and controls are distributed over several hardware devices!    
[FPM Handling Unit](fpm_handling_unit_control.md): Planing and exection for positioning 
