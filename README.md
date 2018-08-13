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
 - SIL : Software in the loop
 - PIL : Processor in the loop
 - HIL : Hardware in the loop

# Building + Running
## Golden Model
Execuate the SIL Mater+Slave
```
    cd exe/SIL/master
    ./SIL.sh
```

Execuate the PIL Master
```
    cd exe/PIL/master
    ./Deploy_EGSE_PIL.sh
```

Execuate the HIL Master
```
    cd exe/HIL/master
    ./Deploy_EGSE_HIL.sh
```

Clean HIL/PIL/SIL image, object files, .csv, log
```
   ./deep_clean_exe.sh
```
# Tests

## Automative testing
A auto-testing is preset and should be run every time change is applied.
To run, at simulation root, issue:
```
    ./SIL.sh
```
which will build, execute and compare against golden data.
Result will be displayed on screen.

The testing is based on average result relative error against golden data < 1E-5


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
