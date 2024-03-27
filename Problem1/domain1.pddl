(define (domain domain1)

(:requirements :strips :typing)

(:types
    location                ; different locations (example: central_warehouse)

    locatable               ; parent type for agents, supplies, boxes, workstations
                            ; to allow located(locatable, location)

    agent - locatable       ; the robotic agents
    agent_type_1 - agent

    supply - locatable      ; supplies needed by the workstations

    box - locatable         ; boxes containing needed supplies

    workstation - locatable ; there are a number of workstations
)

(:predicates
    (connected ?l1 ?l2 - location)          ; location ?l1 is connected to ?l2 - the "roadmap"

    (located ?x - locatable ?l - location) ; ?x is at location ?l

    (contains ?b - box ?s - supply)         ; box ?b contains supply ?s
    (isempty ?b - box)                      ; box ?b is empty

    (loaded ?a - agent ?b - box)            ; box ?b is loaded on agent ?a
    (free ?a - agent)                          ; agent ?a is free, not loaded
    (unloaded ?b - box)                     ; box ?b is unloaded, not on an agent

    (has ?w - workstation ?s - supply)      ; workstation ?w has supply ?s
    (needs ?w - workstation ?s - supply)     ; workstation ?w needs supply ?s
)

; agent ?a fills box ?b with supply ?s at location ?l
(:action fill
    :parameters (?a - agent ?b - box ?s - supply ?l - location)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) (located ?s ?l) ; agent, box, supply in the same location
        (isempty ?b)                                    ; box empty
        (unloaded ?b)                                   ; box unloaded
    )
    :effect (and 
        (not (isempty ?b)) (contains ?b ?s)
    )
)

; agent ?a empty box ?b in location ?l, leaving supply ?s to workstation ?w
(:action empty
    :parameters (?a - agent ?b - box ?s - supply ?w - workstation ?l - location)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) (located ?w ?l) ; agent, box, workstation in the same location
        (contains ?b ?s) (needs ?w ?s)                  ; box must contain the needed supply for the workstation
        (unloaded ?b)                                   ; box unloaded
    )
    :effect (and 
        (has ?w ?s) (not (needs ?w ?s))         ; cause the workstation to have it 
        (not (contains ?b ?s)) (isempty ?b)     ; and the box becomes empty 
    )
)

; agent ?a moves from location ?from to location ?to
; if loaded with a box also the box moves, but
; the movement of the box loaded on the agent is modeled by load/unload actions
(:action move
    :parameters (?a - agent ?from ?to - location)
    :precondition (and 
        (located ?a ?from)      ; agent should be in ?from
        (connected ?from ?to)   ; ?from, ?to must be connected 
    )
    :effect (and 
        (not (located ?a ?from)) (located ?a ?to)
    )
)

; agent ?a loads box ?b at location ?l
(:action load
    :parameters (?a - agent ?b - box ?l - location)
    :precondition (and 
        (located ?a ?l) (located ?b ?l) ; agent, box in the same location
        (free ?a) (unloaded ?b)         ; agent must be free (no box on it) and the box unloaded (not on other agents)
    )
    :effect (and 
        (loaded ?a ?b) 
        (not (unloaded ?b)) 
        (not (free ?a))         ; agent loaded
        (not (located ?b ?l))   ; box no more located ?l
    )
)

; agent ?a unloads box ?b at location ?l
(:action unload
    :parameters (?a - agent ?b - box ?l - location)
    :precondition (and 
        (located ?a ?l) ; agent must be in location ?l
        (loaded ?a ?b)  ; agent must be laoded
    )
    :effect (and 
        (free ?a) (unloaded ?b) (not (loaded ?a ?b))    ; agent is free and box unloaded from agent
        (located ?b ?l)                                 ; box is now located at unload location
    )
)

)