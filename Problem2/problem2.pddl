(define (problem problem2) (:domain domain2)
(:objects 

    ; locations
    central_warehouse - warehouse 
    loc1 loc2 loc3 - ws_location

    ; 2 boxes
    box1 box2 box3 - box

    ; 3 workstations
    ws1 ws2 ws3 ws4 - workstation

    ; robotic agent
    agent1 - agent_type_1

    ; carrier for agent1
    carrier1 - carrier

    ; supplies
    bolt valve tool - supply

    ; define quantities up to the largest quantity that appears in the problem
    ; q0 already defined as constant in domain 
    q1 q2 q3 q4 - quantity
)

(:init
    
    ; all boxes at central_warehouse
    (located box1 central_warehouse)
    (located box2 central_warehouse)
    (located box3 central_warehouse)

    ; all the boxes are empty and unloaded (not on agents)
    (isempty box1)
    (unloaded box1)
    (isempty box2)
    (unloaded box2)
    (isempty box3)
    (unloaded box3)

    ; all content/supply at central_warehouse
    (located bolt central_warehouse)
    (located valve central_warehouse)
    (located tool central_warehouse)

    ; robotic agent and carrier at central_warehouse and free
    (located agent1 central_warehouse)
    (located carrier1 central_warehouse)

    ; carrier1 is agent1's carrier
    (has_carrier agent1 carrier1)

    ; carrier1 has maximum capacity 3 and initially has no filled boxes
    (available_slots carrier1 q3)
    (current_filled_boxes carrier1 q0)

    ; workstations ws1 and ws2 at loc1
    ; workstation ws3 at loc2
    (located ws1 loc1)
    (located ws2 loc2)
    (located ws3 loc2)
    (located ws4 loc3)

    ; central_warehouse and loc1 are connected
    (connected central_warehouse loc1)
    (connected loc1 central_warehouse)
    ; central_warehouse and loc2 are connected
    (connected central_warehouse loc2)
    (connected loc2 central_warehouse)
    ; loc3 connected to loc2
    (connected loc3 loc2)
    (connected loc2 loc3)


    ; needs == not have     
    ; ws1 needs only tool
    (needs ws1 tool)

    ; ws2 needs no supply
    
    ; ws3 needs bolt valve tool
    (needs ws3 bolt)
    (needs ws3 valve)
    (needs ws3 tool)

    ; ws4 needs valve
    (needs ws4 valve)

    ; define increnet decrement predicates
    (inc q0 q1) (inc q1 q2) (inc q2 q3) (inc q3 q4)
    (dec q4 q3) (dec q3 q2) (dec q2 q1) (dec q1 q0)
)

(:goal (and
    ; ws1 has tool
    (has ws1 tool)

    ; ws2 not need supply
    
    ; ws3 has bolt valve tool
    (has ws3 bolt)
    (has ws3 valve)
    (has ws3 tool)

    ; ws4 has valve
    (has ws4 valve)
))

)
