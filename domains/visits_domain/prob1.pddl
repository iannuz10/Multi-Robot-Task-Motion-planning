(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 - region
     R2D2 C3PO - robot
     
)
(:init
    (robot_in C3PO r5)
    (robot_in R2D2 r0)
    (= (act-cost) 0)
    (= (dummy) 0)

)
(:goal 
     (and (visited r4 R2D2) (visited r8 C3PO)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)