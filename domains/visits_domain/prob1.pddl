(define (problem prob)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11 r12 r13 r14 r15 r16 r17 r18 r19 r20 r21 r22 r23 r24 r25 r26 r27 r28 r29 r30 r31 r32 r33 r34 r35 - region
     C3PO BB8 R2D2 AP5 - robot
     o1 o2 o3 o4 - object
     
)
(:init
     (robot_in C3PO r1)
     (robot_in BB8 r20)
     (robot_in R2D2 r34)
     (robot_in AP5 r32)
     (= (act-cost) 0)
     (= (dummy) 0)
     (= (object_in o1 r1) 1)
     (= (object_in o2 r12) 1)
     (= (object_in o3 r14) 1)
     (= (object_in o4 r4) 1)
     (= (empty AP5) 0)
     (= (empty C3PO) 0)
     (= (empty BB8) 0)
     (= (empty R2D2) 0)


)
(:goal 
     (and (=(object_in o1 r3) 1)  (=(object_in o2 r3) 1) (=(object_in o3 r25) 1) (=(object_in o4 r25) 1) 
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)