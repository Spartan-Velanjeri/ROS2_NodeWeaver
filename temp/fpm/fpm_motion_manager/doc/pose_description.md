Basekoordinatensystem 
Bautiro / UR
x   /
y   /
z   / 
x rechts
y nach unten
z in fahrt richtung


force_max_height: 635mm
offset_probing: 50mm
pre_drill_positon: 20mm ceiling_distance

Kontakt mit der Deckehöhe bei der Höhe mit maximal möglicher  Kraft $force_max_height Kontakt z

pre_drill_start_pose: Start des Bohrvorgangs ${force_max_height-ceiling_distance} 

initial_joint_pose: Diese Positon wird verwendet um den Abstastvorgang zu starten ${force_max_height-offset_probing}, 


Zentrum vom optimalen Arbeitsraum x/y/z (0/-635/330) 

measurement: Um auf die Measurement pose zu kommen, soll der Arm den Footprint nur minimal überragen. 


transport_joint_pose_1: initial_joint_pose 
                        initial_joint_pose.6+=-90 

transport_joint_pose_1_1: initial_joint_pose 
                        initial_joint_pose.y+=-500 

transport_joint_pose_2: transport_joint_pose_1
                        transport_joint_pose_1.1+=180
transport_joint_pose_3: transport_joint_pose_1                       
                        transport_joint_pose_1.y+=50

driving_:

Arbeitsraum Arm:
Kreis x/Z Ebene mit: 
Mittelpunkt im optimalen Arbeitsraum x/y/z (0/-635/330)
Radius 450mm

780/-120

niedrigster Punkt

y = -500

Optimierungmöglichkeiten
Messbreich in Winkelbereiche zerteilen um den Arm nur aus dem Sichtbereich zu schwenken.
