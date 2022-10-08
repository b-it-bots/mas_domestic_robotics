Overview
================
Architecture of perception and various utilities for processing point clouds and images existing in the
``mas_perception_libs`` package will be discussed in this seciton. The object recognition
pipeline follows this architecture

.. image:: ../../images/perception_arch_recognition.png

Some design reasons for the above architecture are:

- Relying solely on point clouds for detection is not always reliable, especially in domestic environment, because of the variety of materials and object shapes. For example, RGB-D cameras have a particularly hard time with plastic objects, as can be seen in the figure below.
- Exciting new detection algorithms tend to deal with RGB-only images, which are often less affected by object materials. However, many of these use deep learning (DL) libraries in Python, whereas the old detection code is written in C++.
- The fast moving pace of DL demands some form of abstraction for easing integration of new algorithms.

.. image:: ../../images/bad_objects.png

    