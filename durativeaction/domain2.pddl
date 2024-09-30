( define (domain durativeaction)
    (:requirements :typing :adl :fluents :durative-actions :negative-preconditions :disjunctive-preconditions)
    (:types
        location
        workstation
        content ; content types like bolt, valve, and tool will be subtypes
        bolt - content
        valve - content
        tool - content
        robot
        box
        carrier
        capacity
    )


    (:predicates 
        (hascapacity ?car - carrier ?cap - capacity)
        (hascarrier ?r - robot ?car - carrier)
        (adjacent ?w - workstation ?l - location) 
        (atl ?r - robot ?l - location)
        (belong ?b - box ?l - location) 
        (attached ?c - content ?l - location)
        (attachedtoWS ?c - content ?w - workstation)
        (wshasbox ?w - workstation ?b - box)  
        (loaded ?r - robot ?b - box) 
        (fully-loaded ?r - robot ?b - box ?car - carrier)
        (empty ?b - box) 
        (contain ?b - box ?c - content)
        (loadedcarrier ?car - carrier)  
        (box_under_capacity ?car - carrier ?cap - capacity ?b - box)    
        (needsupplies ?w - workstation)
        )


    ; check actions which are using multiple same type parameters. try reducing to 1. applies to all domains.
    (:durative-action loadcarrier
        :parameters (?b - box ?r - robot ?car - carrier ?cap - capacity ?l - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (loaded ?r ?b))
            (at start (belong ?b ?l))
            (at start (hascapacity ?car ?cap))
            (over all (hascarrier ?r ?car))
        )
        :effect (and (at start (loadedcarrier ?car)) (at end (fully-loaded ?r ?b ?car)))

    )



    (:durative-action fillbox
        :parameters (?b - box ?r - robot ?c - content ?l - location ?car - carrier ?cap - capacity)
        :duration (= ?duration 3)
        :condition (and (at start (hascapacity ?car ?cap)) (over all (hascarrier ?r ?car)) (at start (empty ?b)) (over all (atl ?r ?l)) (over all (belong ?b ?l)))
        :effect (and (at start (contain ?b ?c)) (at end (not(empty ?b))))
    )

    (:durative-action emptybox
        :parameters (?b - box ?c - content ?w - workstation ?car - carrier)
        :duration (= ?duration 2)
        :condition (and (over all (wshasbox ?w ?b)) (at start(not(loadedcarrier ?car))) (at start (contain ?b ?c)))    
        :effect (and 
            (at start (attachedtoWS ?c ?w))
            (at end (empty ?b)))
    )

    
    (:durative-action pickupbox
        :parameters (?r - robot ?c - content ?b - box ?l - location)
        :duration (= ?duration 2)
        :condition (and (at start (contain ?b ?c)) (over all (not(empty ?b))) (at start (not(loaded ?r ?b))))  
        :effect (and (at start (belong ?b ?l)) (at end (loaded ?r ?b)))      
    )
    
    (:durative-action move
        :parameters (?r - robot ?l ?l1 - location ?b - box ?w - workstation ?car - carrier ?cap - capacity)
        :duration (= ?duration 5)
        :condition (and (over all (box_under_capacity ?car ?cap ?b)) (over all (loadedcarrier ?car)) (over all (fully-loaded ?r ?b ?car)) (at start (atl ?r ?l)) (at start (not(atl ?r ?l1))) (over all (adjacent ?w ?l1)) (at start (needsupplies ?w)))
        :effect (and (at start(atl ?r ?l)) (at end (atl ?r ?l1)))
    )
    
    (:durative-action deliver
        :parameters (?r - robot ?b - box ?l - location ?w - workstation ?c - content ?car - carrier)
        :duration (= ?duration 3)
        :condition (and (over all (contain ?b ?c)) (at start (loadedcarrier ?car)) (over all (atl ?r ?l)) (at start (fully-loaded ?r ?b ?car)) (at start (needsupplies ?w)))
        :effect (and (at start (not(loadedcarrier ?car))) (at end (wshasbox ?w ?b)))      
    )
    
    (:durative-action return
        :parameters (?b - box ?r - robot ?l - location ?car - carrier ?c - content ?w - workstation)
        :duration (= ?duration 3)
        :condition (and (over all (empty ?b)) (over all (attachedtoWS ?c ?w)))
        :effect (and (at start (not(fully-loaded ?r ?b ?car))) (at end (atl ?r ?l)) (at start(attachedtoWS ?c ?w)) (at end(attachedtoWS ?c ?w)))
    )  
)

 

    
