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
        (battery_charged ?r - robot)
        (battery_low ?r - robot)
    )

    (:functions
    )

    (:durative-action move
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration ( = ?duration 5)
        :condition (and
            (at start (robot_at ?r ?wp1))
            (at start (path ?wp1 ?wp2))
            (over all (battery_charged ?r))
        )
        :effect (and
            (at start(not(robot_at ?r ?wp1)))
            (at end(robot_at ?r ?wp2))
            (at end(patrolled ?r ?wp2))
        )
    )

    (:durative-action move_recharge_station
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration ( = ?duration 5)
        :condition (and
            (at start (robot_at ?r ?wp1))
            (at start (path ?wp1 ?wp2))
            (at start (recharge_station ?wp2))
            (at start (battery_low ?r))
        )
        :effect (and
            (at start(not(robot_at ?r ?wp1)))
            (at end(robot_at ?r ?wp2))
        )
    )

    (:durative-action recharge
        :parameters (?r - robot ?wp - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (over all(robot_at ?r ?wp))
            (over all(recharge_station ?wp))
           )
        :effect (and
          (at end (battery_charged ?r))
          (at end (not(battery_low ?r)))
        )
    )

)
