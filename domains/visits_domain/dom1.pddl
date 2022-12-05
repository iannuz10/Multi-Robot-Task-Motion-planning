(define (domain localization)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents )


(:types  robot region object
)

(:predicates
  (robot_in ?v - robot ?r - region) (visited ?r - region ?v - robot) 
       )

(:functions 
  (act-cost) (triggered ?from ?to - region ?v - robot) (dummy) (empty ?v - robot) (object_in ?o - object ?r - region) (holding ?v - robot ?o - object)
)

(:durative-action goto_region
  :parameters (?v - robot ?from ?to - region)
  :duration (= ?duration 100)
  :condition (and (at start (robot_in ?v ?from)))
         :effect (and (at start (not (robot_in ?v ?from))) (at start (increase (triggered ?from ?to ?v) 1))
  (at end (robot_in ?v ?to)) (at end (assign (triggered ?from ?to ?v) 0)) (at end (visited ?to ?v))  
                (at end (increase (act-cost) (dummy))))
)

(:durative-action pick_up
  :parameters (?v - robot ?at - region ?o - object)
  :duration (= ?duration 10)
  :condition (and (at start (= (empty ?v) 0)) (at start (robot_in ?v ?at)) (at start(= (holding ?v ?o) 0)) (at start (= (object_in ?o ?at) 1))  (over all (robot_in ?v ?at)))
  :effect (and (at end (assign (empty ?v) 1)) (at end (assign (object_in ?o ?at) 0)) (at end(assign (holding ?v ?o) 1))
                (at end (increase (act-cost) 5))
  )     
)

(:durative-action put_down
  :parameters (?v - robot ?at - region ?o - object)
  :duration (= ?duration 10)
  :condition (and (at start (robot_in ?v ?at)) (at start (= (empty ?v) 1)) (at start (= (object_in ?o ?at) 0))(over all (robot_in ?v ?at)) (at start(= (holding ?v ?o) 1)) )
  :effect (and (at end (assign (empty ?v) 0)) (at end (assign (object_in ?o ?at) 1)) (at end(assign (holding ?v ?o) 0))
                (at end (increase (act-cost) 5))
  )
)

)