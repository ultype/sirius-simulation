Sirius is a Simulation of rocket
From version 0.1.0, it is ported form Rocket Simulation in 6DOF.
The aim is to provide high fidelity of Rocket Launch and In-flight Dynamics.

# Dependencies 3
 - NASA Trick 17
   - Github NASA/trick
   - Internal mirror: rTRICK repo
 - GCC 5.4
 - Python 2.7

# Installation
Execute in shell at repo root
```
    trick-CP
```

# Running
## Golden Model
Execute in shell at repo root
```
    ./S_main_<platform>.exe RUN_test/golden/golden.py
```
Result will be stored in RUN_test/golden Directory

## Monte Carol
Execute in shell at repo root
```
    ./S_main_<platform>.exe RUN_test/monte/monte.py
```
Result will be stored in RUN_test/monte Directory

# Tests

## Automative testing
A auto-testing is preset and should be run every time change is applied.
To run, issue:
```
    ./tools/run_test.sh
```
which will build, execute and compare against golden data.
Result will be displayed on screen.

The testing is based on average result relative error against golden data < 1E-6

## Golden Model (Not runnable for master branch due to refactor)
Build and Run Golden Model
Use the comparison tool to compare the result
```
    python2.7 tool/generate_error.py RUN_test/golden.csv RUN_test/test.csv
```
Examine the resulting file result.csv, which contains the % error between each data points. It should display all zeros.

# Documents
## Dependencies
 - StarUML: staruml.io
 - Graphviz: install via apt-get

## Component Diagram
The UML of rocket components is in ./docs/Rocket.mdj
Please use 'StarUML' to view it

## Component Topology
With graphviz installed
```
cd docs
dot topology.dot -Tpng > topology.png
```
Then you can use a typical image viewer to view it.

