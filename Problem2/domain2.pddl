(define (domain domain2)

(:requirements :strips :typing :negative-preconditions :equality)

(:types
    location                            ; locations
    warehouse ws_location - location    ; there can't be workstation in warehouse

    locatable - object      ; parent type for agents, supplies, boxes, workstations, carriers
                            ; to allow located(locatable, location)

    agent - locatable       ; the robotic agents
    agent_type_1 drone - agent

    carrier - locatable     ; the carrier for the agents

    supply - locatable      ; supplies needed by the workstations

    box - locatable         ; boxes containing needed supplies

    workstation - locatable ; there are a number of workstations

    quantity - object       ; quantity type, used fot the increase/decrease and keep track of capacity
)

(:constants
    q0 - quantity  ; empty quantity constant
)

(:predicates
    (connected ?l1 ?l2 - location)          ; location ?l1 is connected to ?l2 - the "roadmap"

    (located ?x - locatable ?l - location) ; ?x is at location ?l

    (contains ?b - box ?s - supply)         ; box ?b contains supply ?s
    (isempty ?b - box)                      ; box ?b is empty

    (loaded ?c - carrier ?b - box)          ; box ?b is loaded on carrier ?c
    (unloaded ?b - box)                     ; box ?b is unloaded, not on an agent

    (has ?w - workstation ?s - supply)      ; workstation ?w has supply ?s
    (needs ?w - workstation ?s - supply)     ; workstation ?w needs supply ?s

    (has_carrier ?a - agent ?c - carrier)       ; agent ?a has carrier ?c

    ; increment decrement predicates
    (inc ?q ?q_new - quantity)
    (dec ?q ?q_new - quantity)

    ; predicate to handle the capacity of the carrier
    (available_slots ?c - carrier ?q - quantity)        ; true <=> ?c has ?q available spots left 
                                                                ; available_slots(?c q0) == TRUE -> ?c is full
    
    ; predicate to keep track of the number of filled boxes in the carrier
    ; used for checking when the agent can go back to warehouse, that is when he finished the deliveries, so no filled boxes
    (current_filled_boxes ?c - carrier ?q - quantity)
)


;;; ---------------------
;;; fill empty actions
;;; ---------------------


; agent ?a fills box ?b with supply ?s at location ?l
(:action fill
    :parameters (?a - agent ?b - box ?s - supply ?l - location)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) (located ?s ?l)     ; agent, box, supply in the same location
        (isempty ?b) (unloaded ?b)                          ; box empty and unloaded
    )
    :effect (and 
        (not (isempty ?b)) (contains ?b ?s)
    )
)

; agent ?a empty box ?b in location ?l, leaving supply ?s to workstation ?w
(:action empty
    :parameters (?a - agent ?b - box ?s - supply ?w - workstation ?l - location)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) (located ?w ?l)     ; agent, box, workstation in the same location
        (contains ?b ?s) (needs ?w ?s)                      ; box must contain the needed supply for the workstation
        (unloaded ?b)                                       ; box unloaded
    )
    :effect (and 
        (has ?w ?s) (not (needs ?w ?s))         ; cause the workstation to have it 
        (not (contains ?b ?s)) (isempty ?b)     ; and the box becomes empty 
    )
)


;;; ---------------------
;;; move action for agent, when it moves without the carrier
;;; If for some reason initially the agent is not in the same location of the carrier
;;; this action is used, after the agent reaches the carrier, they always moves together
;;; ---------------------

(:action move-to-carrier
    :parameters (?a - agent ?c - carrier ?from ?to - location)
    :precondition (and 
        (located ?a ?from)          ; agent must be in ?from
        (not (located ?c ?from))    ; carrier must not be in ?from
        (has_carrier ?a ?c)         ; ?c must be ?a carrier

        (connected ?from ?to)       ; ?from, ?to must be connected
    )
    :effect (and 
        (not (located ?a ?from)) (located ?a ?to)
    )
)


;;; ---------------------
;;; move actions for agent moving the carrier
;;; ---------------------

; agent ?a moves carrier ?c from location ?from to location ?to, 
; where ?to is not the warehouse
(:action move-carrier-to-workstations
    :parameters (?a - agent ?c - carrier ?from - location ?to - ws_location)
    :precondition (and 
        (located ?a ?from)                  ; agent must be in ?from
        (located ?c ?from)                  ; carrier ?c must be agent ?a carrier
        (connected ?from ?to)               ; ?from, ?to must be connected
        (has_carrier ?a ?c)                 ; ?c must be ?a carrier
        
        ; an agent can move to a workstation location also without filled boxes
        ; for example it may go to a workstation-location to load empty boxes and bring them back to warehouse
        ; so I don't add the following precondition: (not (current_filled_boxes ?c q0))
    )  
    :effect (and 
        (not (located ?a ?from)) (located ?a ?to) 
        (not (located ?c ?from)) (located ?c ?to)
    )
)

; agent ?a moves carrier ?c from location ?from to warehouse ?to, 
; to return at warehouse the carrier must be without filled boxes (it must have no more supply to deliver)
(:action move-carrier-to-warehouse
    :parameters (?a - agent ?c - carrier ?from - location ?to - warehouse)
    :precondition (and 
        (located ?a ?from)              ; agent must be in ?from
        (located ?c ?from)              ; carrier ?c must be agent ?a carrier
        (connected ?from ?to)           ; ?from, ?to must be connected 
        (has_carrier ?a ?c)             ; ?c must be ?a carrier
        (current_filled_boxes ?c q0)    ; agent can go back to warehouse only if carrier has no supplies left to deliver
    )    
    :effect (and                        ; update agent and carrier position
        (not (located ?a ?from)) (located ?a ?to)
        (not (located ?c ?from)) (located ?c ?to)
    )
)


;;; ---------------------
;;; load unload actions
;;; since not all planners support the conditional effect (if the box is filled or not gives me different effects),
;;; so, I duplicated the actions for the empty boxes and the filled box 
;;; ---------------------

; agent ?a loads filled box ?b on carrier ?c at location ?l
(:action load-filled
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new ?n ?n_new - quantity)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) (located ?c ?l) ; agent, box, carrier in the same location
        (has_carrier ?a ?c)                             ; carrier ?c must be agent ?a carrier
        (unloaded ?b)                                   ; box unloaded 
        (not (isempty ?b))                              ; box must be filled
        
        (available_slots ?c ?q)
        (dec ?q ?q_new)
        (not (= ?q q0))                   ; check if carrier not full

        (current_filled_boxes ?c ?n)
        (inc ?n ?n_new)
    )
    :effect (and
        (loaded ?c ?b) (not (unloaded ?b))      ; box is loaded
        (not (located ?b ?l))                   ; box no more at l

        (available_slots ?c ?q_new) (not (available_slots ?c ?q))   ; update quantity in carrier

        (current_filled_boxes ?c ?n_new)    ; increase the number of filled boxes
        (not (current_filled_boxes ?c ?n))
    )
)

; agent ?a loads empty box ?b on carrier ?c at location ?l
(:action load-empty
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new - quantity)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) (located ?c ?l) ; agent, box, carrier in the same location
        (has_carrier ?a ?c)                             ; carrier ?c must be agent ?a carrier
        (unloaded ?b)                                   ; box unloaded 
        (isempty ?b)                                    ; box must be empty
        
        (available_slots ?c ?q)
        (dec ?q ?q_new)
        (not (= ?q q0))                   ; check if carrier not full
    )
    :effect (and
        (loaded ?c ?b) (not (unloaded ?b))      ; box is loaded
        (not (located ?b ?l))                   ; box no more at l

        (available_slots ?c ?q_new) (not (available_slots ?c ?q))   ; update quantity in carrier
    )
)

; agent ?a unloads filled box ?b from carrier ?c at location ?l
(:action unload-filled
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new ?n ?n_new - quantity)
    :precondition (and 
        (located ?a ?l) (located ?c ?l)     ; agent, carrier in the same location  
        (has_carrier ?a ?c)                 ; carrier ?c must be agent ?a carrier
        (loaded ?c ?b)                      ; box ?b should be loaded on carrier ?c
        (not (isempty ?b))                  ; box must be filled
        
        (available_slots ?c ?q)
        (inc ?q ?q_new)

        (current_filled_boxes ?c ?n)
        (dec ?n ?n_new)
    )
    :effect (and 
        (unloaded ?b) (not (loaded ?c ?b))  ; box is unloaded
        (located ?b ?l)                     ; box is now located at unload location

        (available_slots ?c ?q_new) (not (available_slots ?c ?q))   ; update quantity in carrier

        (current_filled_boxes ?c ?n_new)    ; decrease the number of filled boxes
        (not (current_filled_boxes ?c ?n))
    )
)

; agent ?a unloads empty box ?b from carrier ?c at location ?l
(:action unload-empty
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new - quantity)
    :precondition (and 
        (located ?a ?l) (located ?c ?l)     ; agent, carrier in the same location  
        (has_carrier ?a ?c)                 ; carrier ?c must be agent ?a carrier
        (loaded ?c ?b)                      ; box ?b should be loaded on carrier ?c
        (isempty ?b)                        ; box must be empty
        
        (available_slots ?c ?q)
        (inc ?q ?q_new)
    )
    :effect (and 
        (unloaded ?b) (not (loaded ?c ?b))  ; box is unloaded
        (located ?b ?l)                     ; box is now located at unload location
        (available_slots ?c ?q_new) (not (available_slots ?c ?q))   ; update quantity in carrier
    )
)

)
