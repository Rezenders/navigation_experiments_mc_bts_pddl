(define (domain patrol)
    (:requirements :typing :fluents :durative-actions :negative-preconditions)

    (:types
        robot
        waypoint
    )

    (:predicates
        (path ?wp1 ?wp2 - waypoint)
        (robot_at ?r - robot ?wp - waypoint)
        (recharge_station ?wp - waypoint)
        (patrolled ?r - robot ?wp - waypoint)
    )

    (:functions
      (battery_level ?r - robot)
    )

    (:durative-action move
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration ( = ?duration 5)
        :condition (and
            (at start(robot_at ?r ?wp1))
            (at start(path ?wp1 ?wp2))
            (at start(> (battery_level ?r) 30))
        )
        :effect (and
            (at start(not(robot_at ?r ?wp1)))
            (at end (decrease (battery_level ?r) 25))
            (at end(robot_at ?r ?wp2))
            (at end(patrolled ?r ?wp2))
        )
    )

    (:durative-action move_recharge_station
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration ( = ?duration 5)
        :condition (and
            (at start(robot_at ?r ?wp1))
            (at start(path ?wp1 ?wp2))
            (at start(recharge_station ?wp2))
            (at start(< (battery_level ?r) 30))
            (over all(> (battery_level ?r) 0))
        )
        :effect (and
            (at start(not(robot_at ?r ?wp1)))
            (at end (decrease (battery_level ?r) 25))
            (at end(robot_at ?r ?wp2))
            (at end(patrolled ?r ?wp2))
        )
    )

    (:durative-action recharge
        :parameters (?r - robot ?wp - waypoint)
        :duration ( = ?duration 5)
        :condition (and
            (over all(robot_at ?r ?wp))
            (over all(recharge_station ?wp))
           )
        :effect (and (at end (assign (battery_level ?r) 100)))
    )

)

/* set goal (and(patrolled r2d2 wp1)(patrolled r2d2 wp2)(patrolled r2d2 wp3)(patrolled r2d2 wp4)(patrolled r2d2 wp5)(patrolled r2d2 wp6)(robot_at r2d2 wp_r)) */
