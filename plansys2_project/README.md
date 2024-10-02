# PlanSys2 Project in Automated planning for the workflow of a workstation in Python

## Description

This is a simple example that shows the basic operation of PlanSys2. A simple PDDL model with problem file have been integrated. Actions simulate their execution.

## How to run



```
ros2 launch plansys2_project plansys2_project.launch.py
```

Commands for problem file:

```
set instance r1 robot
set instance r2 robot
set instance r3 robot
set instance central_warehouse location
set instance l2 location
set instance l3 location
set instance bolt1 bolt
set instance bolt2 bolt
set instance bolt3 bolt
set instance valve1 valve
set instance valve2 valve
set instance tool1 tool
set instance tool2 tool
set instance b1 box
set instance b2 box
set instance b3 box
set instance w1 workstation
set instance w2 workstation
set instance w3 workstation
set instance car1 carrier
set instance car2 carrier
set instance car3 carrier
set instance cap_50 capacity
set instance cap_20 capacity
set instance cap_30 capacity

set predicate (empty b1)
set predicate (empty b2)
set predicate (empty b3)

set predicate (belong b1 central_warehouse)
set predicate (belong b2 central_warehouse)
set predicate (belong b3 central_warehouse)

set predicate (atl r1 central_warehouse)
set predicate (atl r2 central_warehouse)
set predicate (atl r3 central_warehouse)

set predicate (hascapacity car1 cap_50)
set predicate (hascapacity car2 cap_20)
set predicate (hascapacity car3 cap_30)




set goal (and 
(attachedtoWS bolt1 w1) 
(attachedtoWS valve1 w1) 
(attachedtoWS tool1 w1) 
(attachedtoWS bolt2 w2)
(attachedtoWS valve2 w2)
(attachedtoWS bolt3 w3)
(attachedtoWS tool2 w3)
)      # Sets the goal
get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```
