(define
    (problem problem5)
    (:domain domain5)
    (:objects 

central_warehouse - warehouse 
loc1 - ws_location
loc2 - ws_location
loc3 - ws_location
box1 - box
box2 - box 
box3 - box
ws1 - workstation
ws2 - workstation
ws3 - workstation
ws4 - workstation
agent1 - agent
carrier1 - carrier
bolt - supply 
valve - supply
tool - supply
q0 - quantity
q1 - quantity
q2 - quantity
q3 - quantity
q4 - quantity

    )
    (:init
    
(located box1 central_warehouse)
(located box2 central_warehouse)
(located box3 central_warehouse)
(isempty box1)
(unloaded box1)
(isempty box2)
(unloaded box2)
(isempty box3)
(unloaded box3)
(located bolt central_warehouse)
(located valve central_warehouse)
(located tool central_warehouse)
(located agent1 central_warehouse)
(located carrier1 central_warehouse)
(notlocated carrier1 loc1)
(notlocated carrier1 loc2)
(notlocated carrier1 loc3)
(not_busy agent1)
(has_carrier agent1 carrier1)
(available_slots carrier1 q4)
(current_filled_boxes carrier1 q0)
(located ws1 loc1)
(located ws2 loc2)
(located ws3 loc2)
(located ws4 loc3)
(connected central_warehouse loc1)
(connected loc1 central_warehouse)
(connected central_warehouse loc2)
(connected loc2 central_warehouse)
(connected loc3 loc2)
(connected loc2 loc3)
(needs ws1 tool)
(needs ws3 bolt)
(needs ws3 valve)
(needs ws3 tool)
(needs ws4 valve)
(inc q0 q1)
(inc q1 q2)
(inc q2 q3)
(inc q3 q4)
(dec q4 q3)
(dec q3 q2)
(dec q2 q1)
(dec q1 q0)
(is_zero q0)
(is_not_zero q1)
(is_not_zero q2)
(is_not_zero q3)
(is_not_zero q4)

    )
    (:goal (and
    (has ws1 tool) (has ws3 bolt) (has ws3 valve) (has ws3 tool) (has ws4 valve)
        
    )))