(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11 r12 r13 r14 r15 r16 r17 r18 r19 r20 r21 r22 r23 r24 - region
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
     (and (visited r18 BB8)  (visited r8 C3PO) (visited r15 R2D2) (visited r20 AP5) (visited r24 BD1)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)


