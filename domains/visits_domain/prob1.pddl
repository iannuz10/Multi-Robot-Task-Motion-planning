(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 - region
     R2D2 C3PO BB8 - robot
     
)
(:init
    (robot_in R2D2 r0)
    (robot_in C3PO r4)
    (robot_in BB8 r3)
    (= (act-cost) 0)
    (= (dummy) 0)

)
(:goal 
     (and (visited r1)  (visited r2)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)


