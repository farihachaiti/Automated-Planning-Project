(define (domain workstation)
    (:requirements :strips :typing :negative-preconditions :conditional-effects :disjunctive-preconditions)
    (:types
        location  
        workstation    ; * there are several connected locations in the harbor
        content          ; * is attached to a location, it holds a pallet and a
        robot         ; * holds at most 1 container, only 1 robot per location
        box         ; * belongs to a location to pickup containers
    )

    (:predicates 
        (adjacent ?w - workstation ?l - location) 
        (hasWS ?l - location)
        (hasmultipleWS ?l - location)
        (atl ?r - robot ?l - location)
        (belong ?b - box ?l - location) 
        (attached ?c - content ?l - location)
        (attachedtoWS ?c - content ?w - workstation)
        (wshasbox ?w - workstation ?b - box) 
        (loaded ?r - robot ?b - box) 
        (empty ?b - box) 
        (contain ?b - box ?c - content)
        (needsupplies ?w - workstation)
        (connected ?l1 ?l2 - location) 
        (is-bolt ?c - content)
        (is-valve ?c - content)
        (is-tool ?c - content)
     
    )
    
    (:action fillbox
        :parameters (?b - box ?r - robot ?c - content ?l - location)
        :precondition (and (or (is-bolt ?c) (is-valve ?c) (is-tool ?c)) (not(loaded ?r ?b)) (empty ?b) (atl ?r ?l) (belong ?b ?l) (not(hasmultipleWS ?l)) (not(hasWS ?l)))
        :effect (and (contain ?b ?c) (not(attached ?c ?l)) (not(empty ?b)) (atl ?r ?l) (belong ?b ?l) (not(loaded ?r ?b)))
    )

    (:action emptybox
        :parameters (?r - robot ?b - box ?c - content ?l - location ?w - workstation)
        :precondition (and 
            (wshasbox ?w ?b)               ; The workstation has the box
            (not(loaded ?r ?b))            ; The robot is not already loaded with the box
        )
        :effect (and 
            (empty ?b)                     ; The box becomes empty            ; The robot is no longer loaded with the box
            (attachedtoWS ?c ?w)           ; The content is attached to the workstation
            (when (is-bolt ?c) (attachedtoWS ?c ?w))   ; Attach bolt if it's a bolt
            (when (is-valve ?c) (attachedtoWS ?c ?w)) ; Attach valve if it's a valve
            (when (is-tool ?c) (attachedtoWS ?c ?w))   ; Attach tool if it's a tool
        )
    )

    
    (:action pickupbox
        :parameters (?r - robot ?c - content ?b - box ?l ?l1 - location ?w - workstation)
        :precondition (and (contain ?b ?c) (not(attached ?c ?l)) (not(empty ?b)) (atl ?r ?l) (belong ?b ?l) (not(loaded ?r ?b)))
        :effect (and (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w) (loaded ?r ?b) (connected ?l ?l1) (atl ?r ?l) (not(atl ?r ?l1)) (belong ?b ?l))      
    )
    
    (:action move
        :parameters (?r - robot ?l ?l1 - location ?b - box ?w - workstation ?c - content)
        :precondition (and (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w) (connected ?l ?l1) (loaded ?r ?b) (atl ?r ?l) (not(atl ?r ?l1)) (belong ?b ?l))
        :effect (and (hasWS ?l1) (adjacent ?w ?l1) (needsupplies ?w) (contain ?b ?c) (atl ?r ?l1) (belong ?b ?l1) (loaded ?r ?b))
    )
    
    (:action deliver
        :parameters (?r - robot ?b - box ?l - location ?w - workstation ?c - content)
        :precondition (and (hasWS ?l) (adjacent ?w ?l) (needsupplies ?w) (contain ?b ?c) (belong ?b ?l) (loaded ?r ?b) (atl ?r ?l))
        :effect (and (wshasbox ?w ?b) (not(loaded ?r ?b)))      
    )
    
    (:action return
        :parameters (?b - box ?r - robot ?l - location ?car - carrier ?c - content ?w - workstation)
        :precondition (and (empty ?b) (attachedtoWS ?c ?w) (or (is-bolt ?c) (is-valve ?c) (is-tool ?c)) )
        :effect (and (atl ?r ?l) (empty ?b) (not(loaded ?r ?b)) (belong ?b ?l) (not(hasmultipleWS ?l)) (not(hasWS ?l))
            (when (is-bolt ?c) (attachedtoWS ?c ?w))   ; Attach bolt if it's a bolt
            (when (is-valve ?c) (attachedtoWS ?c ?w)) ; Attach valve if it's a valve
            (when (is-tool ?c) (attachedtoWS ?c ?w)))
    )
    
)
