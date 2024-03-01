(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

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
  (laser_ok ?r - robot)
  (laser_failed ?r - robot)
  (camera_ok ?r - robot)
  (camera_failed ?r - robot)
  (current_mode ?m - mode)
  (battery_low_mode ?m - mode)
  (normal_mode ?m - mode)
  (degraded_mode ?m - mode)
  (degraded_energy_mode ?m - mode)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action navigate_to_pose
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp1))
        (at start (path ?wp1 ?wp2))
        (over all (battery_charged ?r))
        (over all (laser_ok ?r))
        (over all (current_mode ?m))
        (over all (normal_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
        (at end(patrolled ?r ?wp2))
    )
)

(:durative-action navigate_to_recharge_station
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp1))
        (at start (path ?wp1 ?wp2))
        (at start (recharge_station ?wp2))
        (at start (battery_low ?r))
        (over all (laser_ok ?r))
        (over all (current_mode ?m))
        (over all (battery_low_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action navigate_to_recharge_station_degraded
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp1))
        (at start (path ?wp1 ?wp2))
        (at start (recharge_station ?wp2))
        (at start (battery_low ?r))
        (over all (laser_failed ?r))
        (over all (camera_ok ?r))
        (over all (current_mode ?m))
        (over all (degraded_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action navigate_to_pose_degraded
    :parameters (?r - robot ?wp1 ?wp2 - waypoint ?m - mode)
    :duration ( = ?duration 6)
    :condition (and
        (at start (robot_at ?r ?wp1))
        (at start (path ?wp1 ?wp2))
        (over all (battery_charged ?r))
        (over all (laser_failed ?r))
        (over all (camera_ok ?r))
        (over all (current_mode ?m))
        (over all (degraded_mode ?m))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
        (at end(patrolled ?r ?wp2))
    )
)

(:durative-action recharge
    :parameters (?r - robot ?wp - waypoint ?m - mode)
    :duration ( = ?duration 10)
    :condition (and
        (at start (battery_low ?r))
        (over all (robot_at ?r ?wp))
        (over all (recharge_station ?wp))
        (over all (current_mode ?m))
       )
    :effect (and
      (at end (battery_charged ?r))
      (at end (not(battery_low ?r)))
    )
)

(:durative-action reconfigure_system
    :parameters (?r - robot ?m1 ?m2 - mode)
    :duration ( = ?duration 10)
    :condition (and
        (at start(current_mode ?m1))
    )
    :effect (and
        (at start(not(current_mode ?m1)))
        (at end(current_mode ?m2))
    )
)
);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
