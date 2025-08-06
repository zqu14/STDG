# STDG

#### Introduction

This repository includes the source code used for STDG algorithm

----

#### Dependency Version
| Dependency | Value |
| --------- | -----:|
| OMNeT++  | 5.6.2 |
| INET     | inet-4.2.6|
| PICO     | 1.14.2 |

----

#### Key Files
`src/RenewalGenerating.ned` The simualtion network.

`simulations/omnetpp.ini` The configuration file.

`src/arrivalCheck/arrivalChecker.cc` Used to detect the arrivals and collisions, and record collision data.

`src/mobility/DroneLinearMobility.cc` The mobility model for drones, it is an extension version of INET LinearMobility model.

`src/beaconing/renewalbeaconing/NodeGenerator.cc` The node generator model, used to generate and initialize drones.

`src/beaconing/processor/Processor.cc` The implementation of STDG algorithm.

`src/roadSystem/roadSystem.cc` The road system model, storing the road system information read by XML file

----
