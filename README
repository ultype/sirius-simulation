Sirius is a Simulation of rocket
From version 0.1.0, it is ported form Rocket Simulation in 6DOF.
The aim is to provide high fidelity of Rocket Launch and In-flight Dynamics.

= Dependencies =
 - NASA Trick 17
   - Github NASA/trick
   - Internal mirror: rTRICK repo
 - GCC 5.4
 - Python 2.7

= Installation =
Execute in shell at repo root
```
    trick-CP
```

= Running =
== Golden Model ==
Execute in shell at repo root
```
    ./S_main_<platform>.exe RUN_test/input.py
```
Result will be stored in RUN_test Directory

= Tests =
Build and Run Golden Model
Use the comparison tool to compare the result
```
    python2.7 tool/generate_error.py RUN_test/golden.csv RUN_test/test.csv
```
Examine the resulting file result.csv, which contains the % error between each data points. It should display all zeros.

= Documents =
== Dependencies ==
 - StarUML: staruml.io
 - Graphviz: install via apt-get

== Component Diagram ==
The UML of rocket components is in ./docs/Rocket.mdj
Please use 'StarUML' to view it

== Component Topology ==
With graphviz installed
```
cd docs
dot topology.dot -Tpng > topology.png
```
Then you can use a typical image viewer to view it.

