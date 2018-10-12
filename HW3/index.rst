.. Homework 3 documentation master file, created by
   sphinx-quickstart on Fri Oct 12 11:46:21 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Homework 3's documentation!
======================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. HW3 documentation master file, created by
   sphinx-quickstart on Fri Oct 12 11:27:24 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to HW3's documentation!
===============================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. Backup:


.. Homework documentation master file, created by
   sphinx-quickstart on Fri Aug 31 19:29:32 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Kyle's homework documentation!
====================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



**Homework 3**
==============================

Kyle MacMillan



**Text Addition - Uncommon Locomotion**
====================


Common locomotion can be `straightforward <http://roboscience.org/book/html/Motion/Locomotion.html#wheels>`_ or `arbitrarily complex <https://ieeexplore.ieee.org/document/933184>`_. There are also uncommon forms of locomotion in robotics which we will explore here. This is meant to get you thinking about what might be necessary should you decide to make a robot of "non-standard" type.


Paddling
=================

In nature paddling is everywhere that there is water, in robotics we rarely think of it. There are `plenty <http://www.robotic-fish.net/>`_ of `examples <http://robotics.sciencemag.org/content/3/16/eaar3449>`_ of robotic paddling. This paddling motion can be obtained through circular motion (motors), linear actuators, or non-continuous servo motors. Movement through a fluid is not as straightforward as movement on a surface, so our methods of tracking environmental location based on concepts such as wheel rotation(s) is not feasible. Paddling motion has to be considered a force vector(s) with an applied resistance vector. Up and down movement has to be considered as well. That movement can be through buoyancy control, additional paddles, or `hydroplanes <https://en.wikipedia.org/wiki/Diving_plane>`_.



Slithering
=================

Snakes do not have legs yet they move through a series of `vector motions <https://www.researchgate.net/publication/3114211_Design_and_Motion_Planning_of_a_Mechanical_Snake>`_. There are four types of snake movement: 

- Lateral undulation
- Rectilinear locomotion
- Sidewinding
- Concertina progression

Snake motion is somewhat similar to mechanum wheel motion in that the direction of motion is not parallel to the direction of force. Slithering robots require several points to push from in opposing directions in order to generate forward thrust. One of the primary benefits of a robot capable of slithering is that it can be fit into much smaller spaces than a wheeled robot of similar size. The ability to bend corners allows for exploration of smaller spaces. 


Hopping
=================

Another form of locomotion is hopping, such as how a kangaroo moves around. One company, `Festo <https://www.festo.com/cms/en-us_us/index.htm>`_, has built a `robotic kangaroo <https://www.youtube.com/watch?time_continue=66&v=mWiNlWk1Muw>`_ that conserves energy as it hops which is an essential part of any system that wishes to implement a form of hopping locomotion. To navigate a hopping robot there must be pivot structures in the feet so the hoping force can be applied in a different orientation. Also, for terrain navigation it has a parabolic movement, meaning you have to ensure there are no obstacles in the parabolic arc while path planning. A robot like this may be ideal in low-gravity situations.


Conclusion
=================

There are a variety of "non-standard" options for robot locomotion, each with their own challenges and benefits. It is important to be aware of options prior to making a decision for your robotic solution. 