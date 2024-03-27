(define (problem problem1) (:domain domain1)
(:objects 

    ; locations
    central_warehouse loc1 loc2 - location

    ; 2 boxes
    box1 box2 - box

    ; 3 workstations
    ws1 ws2 ws3 - workstation

    ; robotic agent
    agent1 - agent_type_1

    ; supplies
    bolt valve tool - supply
)

(:init
    
    ; all boxes at central_warehouse
    (located box1 central_warehouse)
    (located box2 central_warehouse)

    ; all the boxes are empty and unloaded (not on agents)
    (isempty box1)
    (unloaded box1)
    (isempty box2)
    (unloaded box2)

    ; all content/supply at central_warehouse
    (located bolt central_warehouse)
    (located valve central_warehouse)
    (located tool central_warehouse)

    ; robotic agent at central_warehouse and free
    (located agent1 central_warehouse)
    (free agent1)


    ; workstations ws1 and ws2 at location_ws
    ; workstation ws3 at central_warehouse
    (located ws1 loc1)
    (located ws2 loc1)
    (located ws3 loc2)

    ; central_warehouse and loc1 are connected
    (connected central_warehouse loc1)
    (connected loc1 central_warehouse)
    ; central_warehouse and loc2 are connected
    (connected central_warehouse loc2)
    (connected loc2 central_warehouse)


    ; needs == not have     
    ; ws1 needs only tool
    (needs ws1 tool)

    ; ws2 needs no supply
    
    ; ws3 needs bolt valve tool
    (needs ws3 bolt)
    (needs ws3 valve)
    (needs ws3 tool)

)

(:goal (and
    ; ws1 has tool
    (has ws1 tool)

    ; ws2 has no supply / not need supply
    
    ; ws3 has bolt valve tool
    (has ws3 bolt)
    (has ws3 valve)
    (has ws3 tool)
))

)
