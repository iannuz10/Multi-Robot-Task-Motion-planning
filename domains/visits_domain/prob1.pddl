(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 - region
     R2D2 C3PO BB8 AP5 - robot
     
)
(:init
    (robot_in R2D2 r0)
    (robot_in C3PO r4)
    (robot_in BB8 r3)
    (robot_in AP5 r7)
    (= (act-cost) 0)
    (= (dummy) 0)

)
(:goal 
     (and (visited r1 BB8)  (visited r2 C3PO) (visited r5 R2D2) (visited r1 AP5)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)
