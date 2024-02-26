(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
mode
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
  (path ?wp1 ?wp2 - waypoint)
  (robot_at ?r - robot ?wp - waypoint)
  (recharge_station ?wp - waypoint)
  (patrolled ?r - robot ?wp - waypoint)
  (battery_charged ?r - robot)
  (battery_low ?r - robot)
  (nav_sensor ?r - robot)
  (current_mode ?m - mode)
  (battery_low_mode ?m - mode)
  (normal_mode ?m - mode)
  (degraded_mode ?m - mode)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp1))
        (at start (path ?wp1 ?wp2))
        (over all (battery_charged ?r))
        (over all (nav_sensor ?r))
        (over all (current_mode ?m))
        (over all (normal_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
        (at end(patrolled ?r ?wp2))
    )
)

(:durative-action move_recharge_station
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp1))
        (at start (path ?wp1 ?wp2))
        (at start (recharge_station ?wp2))
        (at start (battery_low ?r))
        (over all (current_mode ?m))
        (over all (battery_low_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action recharge
    :parameters (?r - robot ?wp - waypoint ?m - mode)
    :duration ( = ?duration 10)
    :condition (and
        (over all(robot_at ?r ?wp))
        (over all(recharge_station ?wp))
        (over all (current_mode ?m))
        (over all (battery_low_mode ?m))
       )
    :effect (and
      (at end (battery_charged ?r))
      (at end (not(battery_low ?r)))
    )
)

(:durative-action degraded_move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 6)
    :condition (and
        (at start(robot_at ?r ?wp1))
        (over all(battery_charged ?r))
        (over all(nav_sensor ?r))
        (over all(degraded_mode ?m))
        (over all(current_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action reconfig_system
    :parameters (?r - robot ?m1 ?m2 - mode)
    :duration ( = ?duration 10)
    :condition (and
        (at start(current_mode ?m1))
    )
    :effect (and
        (at end(not(current_mode ?m1)))
        (at end(current_mode ?m2))
    )
)

(:durative-action recover_nav_sensor
    :parameters (?r - robot ?m - mode)
    :duration ( = ?duration 1)
    :condition (and
        (at start(degraded_mode ?m))
        (at start(current_mode ?m))
    )
    :effect (and
        (at end(nav_sensor ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
