###########################
Package ``ccu_dataservice``
###########################

.. attention:: ``Loader`` supports **Taskplan 2.0**


Context
  | ccu_dataservice is *outsourced* from ccu_data_services
  | As a **modular** concept it is independent from ROS2.

.. uml:: ccu_dataservice_context.puml

Dependencies
  project dependencies
    - CBautiro : The model
    - TaskPlan.XSD : as in BIM_CONNECTOR

  public/ open source packages
    - pyecore : for loading/storing model in XML-resource
    - lxml : XSD-validation

.. uml:: ccu_dataservice_dependencies.puml


.. rubric:: Class diagram

.. uml:: ccu_dataservice_class_diagramm.puml

.. rubric:: Description

- ``BaseManager`` deals actually with files and persistency.
  It is an abstract class. The same pattern can be applied for
  *Taskplan, Workplan, Roomplan, Mission*.
- ``Validator`` proofs ``Taskplan.xml`` and inhibits/allows further processing.
- ``Loader`` loads ``Taskplan.xml`` into ``Workplan`` model: ccu_bautiro

.. note::
   | To find out more about ``metadata`` etc..
   | Read :doc:`ccu_dataservice_concept_mission_data_and_folder`



Folder-structure ``ccu_dataservice``
====================================

During import and editing
  #. ``Taskplan.xml`` gets stored in ``taskplan/1`` folder.
  #. ``metadata`` gets created, containing *date-of-import* etc.
  #. At current state: ``Mission`` contain no more info than ``all Jobs from TaskPlan``

During Mission Execution
  ``processing_data<#event_number>`` gets created during mission execution

.. uml:: ccu_dataservice_folders.puml

.. note::

   .. toctree::
     :maxdepth: 1

     ccu_dataservice_usb_sudo_mount_issue
     ccu_dataservice_concept_mission_data_and_folder
