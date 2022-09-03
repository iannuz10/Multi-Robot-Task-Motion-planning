(define (problem prob)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11 r12 r13 r14 r15 r16 r17 r18 r19 r20 r21 r22 r23 r24 - region
     C3PO BB8 - robot
     o1 o2 - object
     
)
(:init
     (robot_in C3PO r1)
     (robot_in BB8 r19)
     (= (act-cost) 0)
     (= (dummy) 0)
     (= (object_in o1 r11) 1)
     (= (object_in o2 r12) 1)
     (= (empty C3PO) 0)
     (= (empty BB8) 0)
    

)
(:goal 
     (and (=(object_in o1 r3) 1)  (=(object_in o2 r3) 1)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)