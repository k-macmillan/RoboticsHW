.. Homework documentation master file, created by
   sphinx-quickstart on Fri Aug 31 19:29:32 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Kyle's homework documentation!
====================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`


**Homework 1**
==============================

Kyle MacMillan

31 August 2018

**Problems**:

    Ch1: 1, 8, 15 
    
    Ch2: 9, 10, 11, 29, 31
    
    Text Addition

=================
Chapter 1
=================

1.  **How would you define a robot?**

    The book defines a robot as follows, "A robot is seen as a sophisticated machine that replaces human effort". In class we further defined a "robot" as a construct that has a changing meaning over time. "I" would define a robot as a mechatronic entity that is capable of performing actions. Yes, this is very broad because robot is a general term.

::

8.  **Do you think the Robotic Appliance and Robotic Agent partitioning is a more effective way to classify robots? Why or why not?**

    I do like the classification. Grouping is essential so robots can fit certain functions of the end-user. This also helps people less familiar with the art understand that robots are built for specific purposes, which isn't always apparent. The book makes note that when a classification restricts innovation it is dead to us. I absolutely agree, but again, bounding things really helps to limit scope and so it definitely has a purpose.

::

15. **Work in robotics can replace people with machines. This results in job loss. Discuss the ethics of working in the robotics industry.**

    I could literally write a book on this topic, and if you summed all the posts I've made across all the social media I use I'm sure you could piece together said book. That being said, I'll keep this "brief". 

    Working for a living is a concept that will die within the millenial's lifetime. It is what I believe to be an inevitability, accelerated by capitalism's desires to make maximum profit. Coming to this conclusion I decided I want to work in AI or robotics so I can make the machines that replace humans; I want to accelerate it, to progress humanity to the next epoch. That being said I am fully aware it is going to cause immense issues as people are unable to find jobs but that is quite frankly not my problem. That is a societal issue. Planned obsolecense, consumerism, food shortages, and a lot of what is wrong with the world can go away if we accelerate robotic growth. Accelerating job loss is a good thing if the government did their job. Automated vehicles will save tens of thousands of lives annually, the job loss is a concern but is heavily outweighed by the benefit. In my opinion it is on the government's shoulders to recognize (listen to the very loud groups that have been saying it for years) and effect change that will benefit the people, not the corporations.

=================
Chapter 2
=================
9. 

.. image:: _static/Problem9.jpg

.. highlight:: python

10. Code:

    .. literalinclude:: Problem10.py

11. Code:

    .. literalinclude:: Problem11.py


29. Code:

    .. literalinclude:: Problem29.py


31. **Show that the differential drive kinematic equations are non-holonomic constraints.**

    Position and orientation are supposed to be independent for holomonic functions.
    With differential drive if you are only able to travel in a straight line then the kinematic equation would be holonomic, but 
    when calculating the position we use an equation of the form: t = d1/v1 = d2/v2, which is not holonomic. The book has a description of a holonomic constraint as follows:
    "A holonomic constraint only depends on the coordinates and time and does not depend on derivatives". 

    If both wheels moved at an equal rate then they could be pulled out as a constant of integration but since a differential drive robot has variable wheel speeds we are not able to claim the equations are holonomic.


=================
Text Addition
=================

**Ethics**
=================

When discusssing robots and their place in our world, one should address their incredible ability to perform a repetitive task with perfection every time. Robots are known for this, so why should we discuss it? The reality of the situation is that it puts workers out of a job. This is not something new, jobs have been replaced by machines throughout recorded history [reference] chapter 1. 

So if jobs have been replaced for millenia and we are still able to find jobs why should we discuss this topic? Robotics is an exploding field; much like industrial machines helped pave the way during the Industrial Revolution, we are quickly approaching a "Robot Revolution".

We are developing robots to perform every-day tasks for us because we enjoy automating mundane, boring tasks. Some tasks are dangerous, so we employ robots to do those tasks for us. We can make a lot of money from having robots perform some tasks more efficiently than humans. These robots have come gradually over many, many years and as a result most people are familiar with them and do not feel threatened by them, and why should they?

In 2013 Oxford published `The Future of Employment`_ which stated an estimated 47% of jobs in the United States are at risk. Some considered this report alarmist or scaremongering but numerous studies have been taken since and they all display varying degrees of the same prediction. 

.. _The Future of Employment: https://www.oxfordmartin.ox.ac.uk/downloads/academic/The_Future_of_Employment.pdf

Great minds of the world say robots will take our jobs, so what's their to debate now? *Is it ethical to program or build a robot that is going to take someone's job?* 

Transportation jobs in the United States make up approximately 9% of our workforce according to the
`Bureau of Transportation Statistics`_

.. _Bureau of Transportation Statistics: https://www.bts.gov/content/transportation-related-labor-force-employment-united-states-1990-2016

With automated vehicles we are poised to replace the majority of this workforce with robotic workers. Is it ethical to replace these workers? Is it ethical to *not* replace these workers?

In 2010 there was an estimated 5,419,000 `vehicle crashes`_, of which there were 33,000 deaths and 2,239,000 injuries. `It is estimated`_ that taking steering wheels out of human hands will save 300,000 lives per decade. People will continue to die to vehicular accidents so long as humans are behind the wheel. Not all accidents can be avoided and unfortunately some lives will still be lost with automated vehicles driving.

.. _vehicle crashes: https://en.wikipedia.org/wiki/Motor_vehicle_fatality_rate_in_U.S._by_year
.. _It is estimated: https://www.theatlantic.com/technology/archive/2015/09/self-driving-cars-could-save-300000-lives-per-decade-in-america/407956/ 

What about factory or fast food workers? Is it ethical to replace them? Is it ethical to *not* replace them? A machine can produce the same perfect part every time without getting sick, cutting corners, or having to get spun-up on the task before they generate at peak efficiency. How many times has your order been wrong at a restaurant? A robot can make the perfect burger every time and it can come to you without looking like it has been sat on. 

So if there are huge benefits to having robots perform easily repeatable tasks what do we do with the millions that will be unemployed? That is beyond the scope of robotics, but it is an inevitable problem that robots will cause.