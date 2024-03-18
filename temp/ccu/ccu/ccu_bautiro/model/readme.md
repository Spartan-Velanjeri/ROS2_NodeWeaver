# folder: model for ccu_bautiro Domain Model

> covers the needs for
>
> - loading a `TaskPlan` into *own* model: `CWorkPlan`
> - having `CMetaData` files managing several *plans, rooms, missions*.
> - having `CProcessData` files storing mission events data.

contains file [**model/ccu_bautiro.ecore**](model/ccu_bautiro.ecore) -
an **ecore** model (= *UML class diagram*) of CCU relevant `BAUTIRO` domain data.

## Usage

`*.ecore` files best viewed and edited with **eclipse modeling**.

### eclipse usage

1. Download modeling flavour in latest Version.
   - [eclipse-modeling-2023-06-R-win32-x86_64](http://ftp-stud.fht-esslingen.de/pub/Mirrors/eclipse/technology/epp/downloads/release/2023-06/R/eclipse-modeling-2023-06-R-win32-x86_64.zip)
   - [eclipse-modeling-2023-06-R-linux-gtk-x86_644](http://ftp-stud.fht-esslingen.de/pub/Mirrors/eclipse/technology/epp/downloads/release/2023-06/R/eclipse-modeling-2023-06-R-linux-gtk-x86_64.tar.gz).

2. unzip and launch  - e.g. from  `~/Download/eclipse/eclipse`

3. choose any location as workspace  - e.g. `~/workspace`

4. `open` or `import` this root folder with `.project` as project in your workspace.

5. right-click `ccu_bautiro.ecore` and *open with ...* **\[ecore Editor\]**

## updates tp20 -> tp21

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
