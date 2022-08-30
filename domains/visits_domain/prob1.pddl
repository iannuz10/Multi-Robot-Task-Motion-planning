(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 - region
     R2D2 C3PO BB8 AP5 BD1 - robot
     
)
(:init
    (robot_in R2D2 r8)
    (robot_in C3PO r1)
    (robot_in BB8 r0)
    (robot_in AP5 r5)
    (robot_in BD1 r9)
    (= (act-cost) 0)
    (= (dummy) 0)

)
(:goal 
     (and (visited r3 BB8)  (visited r9 C3PO) (visited r5 R2D2) (visited r2 AP5) (visited r1 BD1)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)