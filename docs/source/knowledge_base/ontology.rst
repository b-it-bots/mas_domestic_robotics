
        def execute(self, userdata):
            if self.kb_interface.is_surface_empty(self.surface_prefix):
                self.say('{0} is empty'.format(self.surface_prefix))
                return 'empty'
            return 'not_empty'
       
       

An example of a state that interacts with the ontology instead of the
online knowledge base is given in the implementation addressing the
`RoboCup@Home where is this
task <https://github.com/b-it-bots/mas_domestic_robotics/blob/kinetic/mdr_planning/mdr_scenarios/mdr_robocup_tasks/mdr_where_is_this/ros/src/mdr_where_is_this/scenario_states/describe_location.py>`__;
there, we query the ontology for the location of objects since we need
to explain (in natural language) where a certain item (e.g. the couch)
can be found in the environment.

Summary
-------

This short tutorial briefly discussed why a domestic robot needs to
possess as much knowledge as possible, how we deal with the problem of
knowledge modelling and representation, what kind of tools we have
developed for interacting with our knowledge base, as well as how those
tools are used throughout our domestic code base.

The most important highlights are as follows:

- Our knowledge base is split into three segments (encyclopedic knowledge represented in an OWL ontology, symbolic knowledge base using ROSPlan, and a simple world model using mongodb_store)
- Our ``mas_knowledge_base`` package defines utilities for interacting with the three elements of the knowledge base
- Both action clients and states from which behaviours are composed are able to interact with and update the knowledge base

More advanced examples of how we use the knowledge base can be found
throughout the ``mas_domestic_robotics`` repository, particularly in the
`mdr_planning <https://github.com/b-it-bots/mas_domestic_robotics/tree/kinetic/mdr_planning>`__
metapackage.
