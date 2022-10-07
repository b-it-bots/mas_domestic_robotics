MAS Domestic Robotics
=====================

This repository contains the implementation of a (mostly) generic architecture for domestic robots. Some parts of the documentation are, however, dedicated to the Toyota HSR, which is the platform we currently work with.

Quick Start Guide
***********************
.. toctree::
   :maxdepth: 1

   quick-start/architecture
   quick-start/capabilities
   quick-start/requirements

Setup
***********************

.. toctree::
   :maxdepth: 1

   setup/development-pc
   setup/connecting-to-hsr
   setup/startup-components

Tutorials
***********************
.. toctree::
   :maxdepth: 1

   tutorials/creating-and-using-an-occupancy-grid-map
   tutorials/creating-a-state-machine-based-task
   tutorials/dmp-based-arm-motion
   tutorials/trajectory-recording
   tutorials/moving-between-named-locations
   tutorials/object-detection-and-pose-estimation
   tutorials/object-grasping

Implemented Demo Scenarios
**************************
.. toctree::
   :maxdepth: 1

   demo-scenarios/patrol
   demo-scenarios/simple-pick-and-place
   demo-scenarios/context-aware-object-hand-over

Actions
***********************

Actions are well-defined functionalities that a robot can use for solving tasks; they particularly correspond to actions as used in classical planning.
All actions in our system have an action server, which implements and manages the action execution, and an action client, which exposes an interface for triggering an action.

.. toctree::
   :maxdepth: 1

   actions/manipulation
   actions/navigation
   actions/perception

Pre-Implemented Behaviours
**************************

Behaviours are reusable SMACH states that can be used for implementing complex tasks. Some of these behaviours interact with the actions described above.

.. toctree::
   :maxdepth: 1

   behaviours/manipulation
   behaviours/navigation
   behaviours/perception

Knowledge Base
***********************

.. toctree::
   :maxdepth: 1

   knowledge_base/storage
   knowledge_base/ontology
