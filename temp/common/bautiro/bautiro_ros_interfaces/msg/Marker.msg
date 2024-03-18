string marker_id
geometry_msgs/PoseStamped pose
string code
string parent
int64 state

# state predefines
int64 UNDEFINED=0     # this shall never happen
int64 PLANNED=1
int64 MEASURED=2


##### : Marker.msg relevant XSD-documentation #####

# marker_id
#       _reflects_
# 'name' in [OnSiteMarker[TaskPlan-2.1.xsd]]
### EN: Unique name of the on-site-marker.
### DE: Eindeutiger Name eines OnSiteMarkers.

# geometry_msgs/PoseStamped pose
### EN: Pose of the On-Site-Marker related to the PKS.
#       Note that the orientation on the marker is included.
#       The usage of this value depends on the state attribute.
#       If state="planned" then the BaseObjectMl represents the planned value.
#       If state="measured" then the BaseObjectMl represents the measured value.
### DE: Vorgeschlagene oder tatsächliche Pose des OnSiteMarkes gegenüber dem PKS.
#       Ob der Wert als "vorgeschlagen" oder als "tatsächlich" interpretiert werden kann,
#       hängt vom Inhalt des Attributes "state" ab.
#
## 'time stamp' in [std_msgs/Header.msg] in [PoseStamped]
#       _reflects_
## 'string time' in [OnSiteMarker[TaskPlan-2.1.xsd]] / [CMarker[cbautrio.ecore]]
### EN: Date and time of the measurement of the single on-site-marker.
### DE: Zeitpunkt der durchgeführten Vermessung des OnSiteMarkers.

# string code
### EN: Data Type for reflex markers. Used for core-markers and on-site-markers as well.
#       Only defined types (facets) of markers will be accepted.

# string parent
### EN: Parent name of the on-site-marker. Use this optional attribute to store
#       the name of the core marker where the on-site-marker is derived from.
### DE: Diese optionale Bezeichnung gibt den Namen des jenigen CoreMarkers an,
#       von welchem der OnSiteMarker abgeleitet wurde.

# int64 state
### EN: Actual state of the on-site-marker.
#       The BaseObjectMl has to be used in the correct manner.
### DE: Aktuelle Situation des OnSiteMarkers.
#       Zum Einen gibt er an, ob der OnSiteMarker bereits vermessen wurde oder nicht
#       und zum Anderen muss diese Vermessung an derjenigen Pose, die durch die BaseObjectMl
#       des OnSiteMarkers angegeben ist, erfolgt sein.
#       Umgekehrt bedeutet dies:
#       Ist state="planned", dann ist der Wert in BaseObjectMl
#       als Positioniervorschlag für den anzubringenden Marker zu verstehen.
#       Wurde dieser dann tatsächlich vermessen, muss die BaseObjectMl auf den
#       gemessenen Wert gesetzt werden. Erst dann darf state="measured" gesetzt werden.

# state predefines
# [TaskPlan-2.1.xsd] (Marker)string state [restrict:  "planned" or "measured"]
### EN: Data type for on-site-markers state.
#       It describes the actual interpretation of the pose ralated to the marker.
#       The value "planned" defines that the pose on-site-marker is just proposed and NOT measured.
#       That means that the pose of this marker is unknown in the real world.
### DE: Datentyp für den Zustand eines OnSiteMarker.
#       Er definiert, wie die angegebene Pose (BaseObjectMl) des OnSiteMarkers
#       interpretiert werden muss. Ist der Wert "planned", so muss die
#       Pose-Information des Markers als "Vorschlag" interpretiert werden.
#       Der Marker muss wahrscheinlich noch angebracht werden. Die tatsächliche
#       Position des Markers im Bohrraum ist also unbekannt.
#       Ist der Wert jedoch "measured" dann kann davon ausgegangen werden,
#       dass der Marker im Bohrraum an der beschriebenen Stelle existiert
#       und die Pose-Koordinate stimmt.

