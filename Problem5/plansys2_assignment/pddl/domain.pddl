(define (domain domain5)

(:requirements :strips :typing :durative-actions)

(:types
    location - object                            ; locations
    warehouse - location
    ws_location - location    ; there can't be workstation in warehouse

    locatable - object      ; parent type for agents, supplies, boxes, workstations, carriers
                            ; to allow located(locatable, location)

    agent - locatable       ; the robotic agents
    ;agent_type_1 drone - agent		; in plansys2 if I leave them it doesn't work

    carrier - locatable     ; the carrier for the agents

    supply - locatable      ; supplies needed by the workstations

    box - locatable         ; boxes containing needed supplies

    workstation - locatable ; there are a number of workstations

    quantity - object       ; quantity type, used fot the increase/decrease and keep track of capacity
)

(:predicates
    (connected ?l1 ?l2 - location)          ; location ?l1 is connected to ?l2 - the "roadmap"

    (located ?x - locatable ?l - location) ; ?x is at location ?l
    (notlocated ?c - carrier ?l - location) ; opposite of located but just for carrier, 
                                            ; tfd and optic does not support negative preconditions
                                            ; and carrier is the only type for which I was using (not (located(...)))

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
    (is_not_zero ?q - quantity)     ; needed for lack of negative preconditions support
    (is_zero ?q - quantity)                    ; needed to remove :constants q0 not supported

    ; predicate to handle the capacity of the carrier
    (available_slots ?c - carrier ?q - quantity)        ; true <=> ?c has ?q available spots left 
                                                        ; available_slots(?c q0) == TRUE -> ?c is full
    
    ; predicate to keep track of the number of filled boxes in the carrier
    ; used for checking when the agent can go back to warehouse, that is when he finished the deliveries, so no filled boxes
    (current_filled_boxes ?c - carrier ?q - quantity)

    (not_busy ?a - agent)       ; agent ?a is not busy, nou doing another action, 
                         ; this avoid the agent to compute multiple action at the same time
)


;;; ---------------------
;;; fill empty actions
;;; ---------------------


; agent ?a fills box ?b with supply ?s at location ?l
(:durative-action fill
    :parameters (?a - agent ?b - box ?s - supply ?l - location)
    :duration(= ?duration 1)
    :condition (and 
        (over all (located ?a ?l)) (over all (located ?b ?l)) (over all (located ?s ?l))     ; agent, box, supply in the same location
        (at start (isempty ?b)) (over all (unloaded ?b))        ; box empty and unloaded
        (at start (not_busy ?a))
    )
    :effect (and 
        (at start (not (isempty ?b))) (at end (contains ?b ?s))
        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)

; agent ?a empty box ?b in location ?l, leaving supply ?s to workstation ?w
(:durative-action empty
    :parameters (?a - agent ?b - box ?s - supply ?w - workstation ?l - location)
    :duration(= ?duration 1)
    :condition (and 
        (over all (located ?a ?l)) (over all (located ?b ?l)) (over all (located ?w ?l))     ; agent, box, workstation in the same location
        (at start (contains ?b ?s)) (at start (needs ?w ?s))                      ; box must contain the needed supply for the workstation
        (over all (unloaded ?b))                                       ; box unloaded
        (at start (not_busy ?a))
    )
    :effect (and 
        (at end (has ?w ?s)) (at start (not (needs ?w ?s)))         ; cause the workstation to have it 
        (at start (not (contains ?b ?s))) (at end (isempty ?b))     ; and the box becomes empty 
        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)


;;; ---------------------
;;; move action for agent, when it moves without the carrier
;;; If for some reason initially the agent is not in the same location of the carrier
;;; this action is used, after the agent reaches the carrier, they always moves together
;;; ---------------------

(:durative-action move_to_carrier
    :parameters (?a - agent ?c - carrier ?from ?to - location)
    :duration(= ?duration 3)
    :condition (and 
        (at start (located ?a ?from))          ; agent must be in ?from
        (over all (notlocated ?c ?from))    ; carrier must not be in ?from, it's the agent that moves, so carrier over all should be not in from
        (over all (has_carrier ?a ?c))         ; ?c must be ?a carrier

        (over all (connected ?from ?to))       ; ?from, ?to must be connected
        (at start (not_busy ?a))
    )
    :effect (and 
        (at start (not (located ?a ?from))) (at end (located ?a ?to))
        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)


;;; ---------------------
;;; move actions for agent moving the carrier
;;; ---------------------

; agent ?a moves carrier ?c from location ?from to location ?to, 
; where ?to is not the warehouse
(:durative-action move_carrier_to_workstations
    :parameters (?a - agent ?c - carrier ?from - location ?to - ws_location)
    :duration(= ?duration 5)
    :condition (and 
        (at start (located ?a ?from))                  ; agent must be in ?from
        (at start (located ?c ?from))                  ; carrier ?c must be agent ?a carrier, wrt the action move-to-carrier, here also the carrier moves so I use at start and not over all
        (over all (connected ?from ?to))               ; ?from, ?to must be connected
        (over all (has_carrier ?a ?c))                 ; ?c must be ?a carrier
        
        (at start (not_busy ?a))
        ; an agent can move to a workstation location also without filled boxes
        ; for example it may go to a workstation-location to load empty boxes and bring them back to warehouse
        ; so I don't add the following precondition: (not (current_filled_boxes ?c q0))
    )  
    :effect (and 
        (at start (not (located ?a ?from))) (at end (located ?a ?to)) 
        (at start (not (located ?c ?from))) (at end (located ?c ?to)) 
        (at start (notlocated ?c ?from)) (at end (not (notlocated ?c ?to))) ; needed for the lack of support on negative preconditions
        
        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)

; agent ?a moves carrier ?c from location ?from to warehouse ?to, 
; to return at warehouse the carrier must be without filled boxes (it must have no more supply to deliver)
(:durative-action move_carrier_to_warehouse
    :parameters (?a - agent ?c - carrier ?from - location ?to - warehouse ?q - quantity)
    :duration(= ?duration 5)
    :condition (and 
        (at start (located ?a ?from))              ; agent must be in ?from
        (at start (located ?c ?from))              ; carrier ?c must be agent ?a carrier
        (over all (connected ?from ?to))           ; ?from, ?to must be connected 
        (over all (has_carrier ?a ?c))             ; ?c must be ?a carrier
        
        ; plansys2 does not support constants so...
        ; (over all (current_filled_boxes ?c q0))    ; agent can go back to warehouse only if carrier has no supplies left to deliver
        (over all (current_filled_boxes ?c ?q))
        (over all (is_zero ?q))
        
        (at start (not_busy ?a))
    )    
    :effect (and                        ; update agent and carrier position
        (at start (not (located ?a ?from))) (at end (located ?a ?to))
        (at start (not (located ?c ?from))) (at end (located ?c ?to))
        (at start (notlocated ?c ?from)) (at end (not (notlocated ?c ?to))) ; needed for the lack of support on negative preconditions
        
        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)


;;; ---------------------
;;; load unload actions
;;; since not all planners support the conditional effect (if the box is filled or not gives me different effects),
;;; so, I duplicated the actions for the empty boxes and the filled box 
;;; ---------------------

; agent ?a loads filled box ?b on carrier ?c at location ?l
(:durative-action load_filled
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new ?n ?n_new - quantity ?s - supply)
    :duration(= ?duration 3)
    :condition (and 
        (over all (located ?a ?l)) (over all (located ?c ?l)) ; agent, box, carrier in the same location
        (at start (located ?b ?l))  ; at start of action the box is in ?l, but during the action action it's loaded on the agent, so it's no more in ?l                              
        (over all (has_carrier ?a ?c))                             ; carrier ?c must be agent ?a carrier
        (at start (unloaded ?b))                                   ; box unloaded 
        (over all (contains ?b ?s))                                ; box must be filled
        
        (at start (available_slots ?c ?q))
        (over all (dec ?q ?q_new))
        (at start (is_not_zero ?q))                   ; check if carrier not full

        (at start (current_filled_boxes ?c ?n))
        (over all (inc ?n ?n_new))
        (at start (not_busy ?a))
    )
    :effect (and
        (at end (loaded ?c ?b)) (at start (not (unloaded ?b)))      ; box is loaded
        (at end (not (located ?b ?l)))                   ; box no more at l

        (at end (available_slots ?c ?q_new)) (at start (not (available_slots ?c ?q)))   ; update quantity in carrier

        (at end (current_filled_boxes ?c ?n_new))    ; increase the number of filled boxes
        (at start (not (current_filled_boxes ?c ?n)))

        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)

; agent ?a loads empty box ?b on carrier ?c at location ?l
(:durative-action load_empty
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new - quantity)
    :duration(= ?duration 2)
    :condition (and 
        (over all (located ?a ?l)) (at start (located ?b ?l)) (over all (located ?c ?l)) ; agent, box, carrier in the same location
        (over all (has_carrier ?a ?c))                             ; carrier ?c must be agent ?a carrier
        (at start (unloaded ?b))                                   ; box unloaded 
        (over all (isempty ?b))                                    ; box must be empty
        
        (at start (available_slots ?c ?q))
        (over all (dec ?q ?q_new))
        (at start (is_not_zero ?q))                   ; check if carrier not full
        (at start (not_busy ?a))
    )
    :effect (and
        (at end (loaded ?c ?b)) (at start (not (unloaded ?b)))      ; box is loaded
        (at end (not (located ?b ?l)))                   ; box no more at l

        (at end (available_slots ?c ?q_new)) (at start (not (available_slots ?c ?q)))   ; update quantity in carrier
        
        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)

; agent ?a unloads filled box ?b from carrier ?c at location ?l
(:durative-action unload_filled
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new ?n ?n_new - quantity ?s - supply)
    :duration(= ?duration 3)
    :condition (and 
        (over all (located ?a ?l)) (over all (located ?c ?l))     ; agent, carrier in the same location  
        (over all (has_carrier ?a ?c))                 ; carrier ?c must be agent ?a carrier
        (at start (loaded ?c ?b))                      ; box ?b should be loaded on carrier ?c
        (over all (contains ?b ?s))                  ; box must be filled
        
        (at start (available_slots ?c ?q))
        (over all (inc ?q ?q_new))

        (at start (current_filled_boxes ?c ?n))
        (over all (dec ?n ?n_new))
        (at start (not_busy ?a))
    )
    :effect (and 
        (at end (unloaded ?b)) (at start (not (loaded ?c ?b)))  ; box is unloaded
        (at end (located ?b ?l))                     ; box is now located at unload location

        (at end (available_slots ?c ?q_new)) (at start (not (available_slots ?c ?q)))   ; update quantity in carrier

        (at end (current_filled_boxes ?c ?n_new))    ; decrease the number of filled boxes
        (at start (not (current_filled_boxes ?c ?n)))

        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)

; agent ?a unloads empty box ?b from carrier ?c at location ?l
(:durative-action unload_empty
    :parameters (?a - agent ?c - carrier ?b - box ?l - location ?q ?q_new - quantity)
    :duration(= ?duration 2)
    :condition (and 
        (over all (located ?a ?l)) (over all (located ?c ?l))     ; agent, carrier in the same location  
        (over all (has_carrier ?a ?c))                 ; carrier ?c must be agent ?a carrier
        (at start (loaded ?c ?b))                      ; box ?b should be loaded on carrier ?c
        (over all (isempty ?b))                        ; box must be empty
        
        (at start (available_slots ?c ?q))
        (over all (inc ?q ?q_new))
        (at start (not_busy ?a))
    )
    :effect (and 
        (at end (unloaded ?b)) (at start (not (loaded ?c ?b)))  ; box is unloaded
        (at end (located ?b ?l))                     ; box is now located at unload location
        
        (at end (available_slots ?c ?q_new)) (at start (not (available_slots ?c ?q)))   ; update quantity in carrier

        (at start (not (not_busy ?a))) (at end (not_busy ?a))
    )
)

)
