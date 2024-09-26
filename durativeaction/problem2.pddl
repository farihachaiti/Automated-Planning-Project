(define (problem durativeaction) (:domain durativeaction)
    (:objects 
        r1 r2 r3 - robot
        central_warehouse l2 l3 - location 
        bolt1 bolt2 bolt3 - bolt
        valve1 valve2 - valve
        tool1 tool2 - tool
        b1 b2 b3 - box
        w1 w2 w3 - workstation
        car1 car2 car3 - carrier
        cap_50 cap_20 cap_30 - capacity
    )
    (:init 

        (empty b1)
        (empty b2)
        (empty b3)
        (belong b1 central_warehouse)
        (belong b2 central_warehouse)
        (belong b3 central_warehouse)
        (atl r1 central_warehouse)  
        (atl r2 central_warehouse)
        (atl r3 central_warehouse)
        (hascapacity car1 cap_50)
        (hascapacity car2 cap_20)
        (hascapacity car3 cap_30)
        (hascarrier r1 car1)
        (hascarrier r2 car2)
        (hascarrier r3 car3)
    )
    
    (:goal (and 
        (not(fully-loaded r1 b1 car1))
        (not(fully-loaded r2 b2 car2))
        (not(fully-loaded r3 b3 car3))
        (atl r1 central_warehouse) 
        (atl r2 central_warehouse) 
        (atl r3 central_warehouse) 
        (attachedtoWS bolt1 w1) 
        (attachedtoWS valve1 w1) 
        (attachedtoWS tool1 w1) 
        (attachedtoWS bolt2 w2)
        (attachedtoWS valve2 w2)
        (attachedtoWS bolt3 w3)
        (attachedtoWS tool2 w3)
    ))

       (:metric minimize (total-time))
 
)