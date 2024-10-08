(define (problem problem1)
    (:domain workstation)
    (:objects 
        r - robot
        central_warehouse l2 l3 - location 
        bolt valve tool - content
        b - box
        w1 w2 w3 - workstation
    )
    (:init 
        (not(hasWS central_warehouse))
        (not(hasmultipleWS central_warehouse))
        (empty b)
        (hasWS l2)
        (hasmultipleWS l2)
        (hasWS l3)
        (not(hasmultipleWS l3))
        (belong b central_warehouse)
        (atl r central_warehouse)  
        (not(adjacent w1 central_warehouse)) 
        (not(adjacent w2 central_warehouse)) 
        (not(adjacent w3 central_warehouse)) 
        (adjacent w1 l2) 
        (needsupplies w1)
        (adjacent w2 l2) 
        (needsupplies w2)
        (adjacent w3 l2) 
        (needsupplies w3)
		(is-bolt bolt) 
        (is-valve valve) 
        (is-tool tool)
        (not(loaded r b))
    )

    (:goal (and 
        (exists (?c1 - content ?c2 - content) 
            (and 
                (attachedtoWS bolt w1) 
                (attachedtoWS valve w1) 
                (attachedtoWS tool w1) 
                (or (and (is-bolt ?c1) (is-valve ?c2)) 
                    (and (is-valve ?c1) (is-bolt ?c2))
                    (and (is-bolt ?c1) (is-tool ?c2)
                    (and (is-valve ?c1) (is-tool ?c2))
                    (and (is-tool ?c1) (is-valve ?c2))
                    (and (is-tool ?c1) (is-bolt ?c2)))
                    )
            )
        )
        (exists (?c1 - content ?c2 - content) 
            (and 
                (attachedtoWS bolt w2)
                (attachedtoWS valve w2)
                (or (and (is-bolt ?c1) (is-valve ?c2)) 
                    (and (is-valve ?c1) (is-bolt ?c2)))
            )
        )
        (exists (?c1 - content ?c2 - content) 
            (and 
                (attachedtoWS bolt w3)
                (attachedtoWS tool w3)
                (or (and (is-bolt ?c1) (is-tool ?c2)) 
                    (and (is-tool ?c1) (is-bolt ?c2)))
            )
        )
        (empty b)
        (not(loaded r b))
        (atl r central_warehouse) 
        (belong b central_warehouse)
        (not(hasmultipleWS central_warehouse)) 
        (not(hasWS central_warehouse))
    ))
)
