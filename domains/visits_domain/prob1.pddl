(define (problem prob)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 - region
     C3PO - robot
     o1 o2 - object
     
)
(:init
     (robot_in C3PO r4)
     (= (act-cost) 0)
     (= (dummy) 0)
     (= (object_in o1 r2) 1)
     (= (object_in o2 r3) 1)
     (= (empty C3PO) 0)
    

)
(:goal 
     (and (=(object_in o1 r1) 1) 
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)