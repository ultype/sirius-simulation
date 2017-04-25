Sirius is a Simulation of rocket
From version 0.1.0, it is ported form Rocket Simulation in 6DOF.
The aim is to provide high fidelity of Rocket Launch and In-flight Dynamics.


# Dependencies
 - NASA Trick 17.0.5
   - Github [NASA/trick](https://github.com/nasa/trick)
   - Internal mirror: rTRICK repo (Deprecated)
 - GCC 5.4
 - Python 2.7
 - [Armadillo Matrix Library](http://arma.sourceforge.net/)
 - Boost Serialization
## Models
You will need to init and update the submodules for the model codes
Issue the following commands after cloning
```bash
    $ git submodule init
    $ git submodule update
```

# Type of Simulation
These simulation as different S_defines resides in exe/ directory.
 - single-node
 - SIL : Software in the loop
 - PIL : Processor in the loop

# Building
Execute in shell at root of simulation you want to run
```
    trick-CP
```

# Running
## Golden Model
Execute in shell at repo root
```
    ./S_main_<platform>.exe RUN_test/golden/golden-<date>.py
```
The input file will have date prefix, only the latest is guarantee to execute.

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
To run, at simulation root, issue:
```
    ./test.sh
```
which will build, execute and compare against golden data.
Result will be displayed on screen.

The testing is based on average result relative error against golden data < 1E-5

## Golden Model
Build and Run Golden Model
Use the comparison tool to compare the result
```
    python2.7 tool/generate_error.py RUN_test/golden-<data>.csv RUN_test/test.csv
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

# Detailed documentation
See: (https://tainan.tispace.com/folder/Simulator/SIRIUS/SIRIUS)
