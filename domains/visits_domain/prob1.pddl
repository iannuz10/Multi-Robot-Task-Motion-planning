(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11 r12 r13 r14 r15 r16 r17 r18 r19 r20 r21 r22 r23 r24 r25 r26 r27 r28 r29 r30 r31 r32 r33 r34 r35 r36 r37 r38 r39 r40 r41 r42 r43 r44 r45 r46 r47 r48 r49 r50 r51 r52 r53 r54 r55 r56 r57 r58 r59 r60 r61 r62 r63 r64 r65 - region
     R2D2 C3PO BB8 AP5 BD1 IG11 - robot
     
)
(:init
    (robot_in R2D2 r8)
    (robot_in C3PO r46)
    (robot_in BB8 r33)
    (robot_in AP5 r54)
    (robot_in BD1 r26)
    (robot_in IG11 r19)
    (= (act-cost) 0)
    (= (dummy) 0)

)
(:goal 
     (and (visited r13 BB8)  (visited r26 C3PO) (visited r60 R2D2) (visited r28 AP5) (visited r46 BD1) (visited r50 IG11)
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost) )
)