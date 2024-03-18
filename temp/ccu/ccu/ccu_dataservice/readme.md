# CDataServise (Python Package)

Python package that implements *core-business-logic* of `ccu_data_service`

Neither depends on `ROS`*.msg* nor `gRPC`*.proto*

## Dependencies

see [setup.py](setup.py).

## Example usage

```python
from ccu_dataservice.core import DSContext, CoreDS, HcuDS

ctx = DSContext()
c = CoreDS(ctx)
h = HcuDS(ctx)

for m in h.list_missions():
  print(m)  # print all missions
```

## Changes TP20 -> TP21 update (BAUTIRO-642)

Delta in XSD:

```text
- Angabe "version" in TaskPlan jetzt "required"
- Attribut "taskname" im TaskPlan neu.
- DrillJob: BaseObjectMl in BaseObjectMg umbenannt
- OnSiteMarker: BaseObjectMl in BaseObjectMg umbenannt
- MoveJob
- Moving/MoveJobList1D neu
- MoveJob neu; Damit kann (optional) eine Endpose für das RPM vorgegeben werden.
- MoveJobRef in JobRefList1D neu
- JobRefList1D optional, aber wenn vorhanden, dann muss mindestens 1 Job definiert sein.
- DestMoveJobRef (JobRefList1D) neu; Damit wird eine Endpose aus der Liste der definierten MoveJobs ausgewählt.
- Attribute "name" für MoveJob, DrillJob und MeasureJob jetzt "required" (vorher "optional").
- Element "MarkerPlan" neu. Hier können jetzt verschiedene Ansichten des Markerplanes eingetragen werden (optional). Hinweis: Wird keine Ansicht vorgegeben, kann diese auch nicht in der Aufstellanleitung (XSLT) verarbeitet werden; der MarkerPlan-Ansicht(en) fehlt bzw. fehlen dann dort.
- OnSiteMarkerList1D jetzt optional. Falls sie gebraucht werden sollte, existieren ohnehin Verlinkungen, die deren Existenz erzwingen
```

- files & folders updated in folder `xsd` according to new TP21.xsd and modified NodeTre.xsd
- files renamed (validator21, loader21)
- adaption to structural changes in business logic:
  -
  - BaseObjectMl got renamed to BaseObjectMg when coordinates are global (not relative).
  - end
