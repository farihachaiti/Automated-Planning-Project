(define (domain carrier)
    (:requirements :strips :typing :negative-preconditions :conditional-effects :disjunctive-preconditions)
    (:types
        location  
        workstation    ; * there are several connected locations in the harbor
        bolt - content
        valve - content
        tool - content
        content     ; * is attached to a location, it holds a pallet and a
        robot         ; * holds at most 1 container, only 1 robot per location
        box        ; * belongs to a location to pickup containers
        carrier
        capacity
    )
    (:predicates 
        (hascapacity ?car - carrier ?cap - capacity)
        (hascarrier ?r - robot ?car - carrier)
        (adjacent ?w - workstation ?l - location) 
        (hasWS ?l - location)
        (hasmultipleWS ?l - location)
        (atl ?r - robot ?l - location)
        (belong ?b - box ?l - location) 
        (attached ?c - content ?l - location)
        (attachedtoWS ?c - content ?w - workstation)
        (wshasbox ?w - workstation ?b - box)  
        (loaded ?r - robot ?b - box) 
        (fully-loaded ?r - robot ?b - box ?car - carrier)
        (empty ?b - box) 
        (contain ?b - box ?c - content)
        (connected ?l1 ?l2 - location) 
        (loadedcarrier ?car - carrier)  
        (box_under_capacity ?car - carrier ?cap - capacity ?b - box)    
        (needsupplies ?w - workstation)
        (caratloc ?car - carrier ?l - location)
        (is-bolt ?c - content)
        (is-valve ?c - content)
        (is-tool ?c - content)
        )
    

    (:action loadcarrier
        :parameters (?b - box ?r - robot ?c - content ?car - carrier ?cap - capacity ?l ?l1 - location ?w - workstation)
        :precondition (and (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w) (loaded ?r ?b) (connected ?l ?l1) (atl ?r ?l) (not(atl ?r ?l1)) (belong ?b ?l) (hascapacity ?car ?cap) (hascarrier ?r ?car))
        :effect (and (not(loaded ?r ?b)) (box_under_capacity ?car ?cap ?b) (fully-loaded ?r ?b ?car) (loadedcarrier ?car) (atl ?r ?l) (not(atl ?r ?l1)) (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w)
            (when (box_under_capacity ?car ?cap ?b) (fully-loaded ?r ?b ?car))   ; Attach bolt if it's a bolt
            (when (not(box_under_capacity ?car ?cap ?b)) (not(fully-loaded ?r ?b ?car)))) ; Attach valve if it's a valv
    )



    (:action fillbox
        :parameters (?b - box ?r - robot ?c - content ?l - location ?car - carrier ?cap - capacity)
        :precondition (and (hascapacity ?car ?cap) (hascarrier ?r ?car) (not(loadedcarrier ?car)) (or (is-bolt ?c) (is-valve ?c) (is-tool ?c)) (empty ?b) (atl ?r ?l) (belong ?b ?l)  (not(hasmultipleWS ?l)) (not(hasWS ?l)))
        :effect (and (contain ?b ?c) (not(attached ?c ?l)) (not(empty ?b)) (atl ?r ?l) (belong ?b ?l) (not(loaded ?r ?b)))
    )

    (:action emptybox
        :parameters (?r - robot ?b - box ?c - content ?l - location ?w - workstation ?car - carrier)
        :precondition (and (wshasbox ?w ?b) (not(loadedcarrier ?car)) (contain ?b ?c) (not(fully-loaded ?r ?b ?car)) (caratloc ?car ?l) (atl ?r ?l))    
        :effect (and 
            (empty ?b)                     ; The box becomes empty            ; The robot is no longer loaded with the box
            (attachedtoWS ?c ?w)           ; The content is attached to the workstation
            (when (is-bolt ?c) (attachedtoWS ?c ?w))   ; Attach bolt if it's a bolt
            (when (is-valve ?c) (attachedtoWS ?c ?w)) ; Attach valve if it's a valve
            (when (is-tool ?c) (attachedtoWS ?c ?w))   ; Attach tool if it's a tool
        )
    )

    
    (:action pickupbox
        :parameters (?r - robot ?c - content ?b - box ?l ?l1 - location ?w - workstation ?car - carrier ?cap - capacity)
        :precondition (and (contain ?b ?c) (not(attached ?c ?l)) (not(empty ?b)) (atl ?r ?l) (belong ?b ?l) (not(loaded ?r ?b)))
        :effect (and (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w) (loaded ?r ?b) (connected ?l ?l1) (atl ?r ?l) (not(atl ?r ?l1)) (belong ?b ?l) (hascapacity ?car ?cap) (hascarrier ?r ?car))      
    )
    
    (:action move
        :parameters (?r - robot ?l ?l1 - location ?b - box ?w - workstation ?c - content ?car - carrier ?cap - capacity)
        :precondition (and (not(loaded ?r ?b)) (box_under_capacity ?car ?cap ?b) (loadedcarrier ?car) (fully-loaded ?r ?b ?car) (atl ?r ?l) (not(atl ?r ?l1)) (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w))
        :effect (and (contain ?b ?c) (loadedcarrier ?car) (atl ?r ?l1) (caratloc ?car ?l1) (fully-loaded ?r ?b ?car) (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w))
    )
    
    (:action deliver
        :parameters (?r - robot ?b - box ?l - location ?w - workstation ?c - content ?car - carrier)
        :precondition (and (contain ?b ?c) (loadedcarrier ?car) (atl ?r ?l) (caratloc ?car ?l) (fully-loaded ?r ?b ?car) (hasWS ?l) (adjacent ?w ?l) (needsupplies ?w))
        :effect (and (wshasbox ?w ?b) (not(loadedcarrier ?car)) (contain ?b ?c) (not(fully-loaded ?r ?b ?car)) (caratloc ?car ?l) (atl ?r ?l))      
    )
    
    (:action return
        :parameters (?b - box ?r - robot ?l - location ?car - carrier ?c - content ?w - workstation ?cap - capacity)
        :precondition (and (empty ?b) (attachedtoWS ?c ?w) (or (is-bolt ?c) (is-valve ?c) (is-tool ?c)))
        :effect (and (atl ?r ?l) (empty ?b) (not(loadedcarrier ?car)) (hascapacity ?car ?cap) (hascarrier ?r ?car) (belong ?b ?l) (not(hasmultipleWS ?l)) (not(hasWS ?l))
            (when (is-bolt ?c) (attachedtoWS ?c ?w))   ; Attach bolt if it's a bolt
            (when (is-valve ?c) (attachedtoWS ?c ?w)) ; Attach valve if it's a valve
            (when (is-tool ?c) (attachedtoWS ?c ?w))  
        )
    )

)

 

    
