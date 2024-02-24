( define ( problem problem_1 )
( :domain patrol )
( :objects
	r2d2 - robot
  wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp_r wp_failure - waypoint
)
( :init
  (robot_at r2d2 wp_r)
  (recharge_station wp_r)
  (battery_charged r2d2)

  (path wp1 wp2)
  (path wp2 wp3)
  (path wp3 wp4)
  (path wp4 wp5)
  (path wp5 wp6)
  (path wp6 wp7)

  (path wp1 wp_r)
  (path wp2 wp_r)
  (path wp3 wp_r)
  (path wp4 wp_r)
  (path wp5 wp_r)
  (path wp6 wp_r)
  (path wp7 wp_r)

  (path wp_r wp1)
  (path wp_r wp2)
  (path wp_r wp3)
  (path wp_r wp4)
  (path wp_r wp5)
  (path wp_r wp6)
  (path wp_r wp7)

  (path wp_failure wp_r)
)
( :goal
  ( and
    (patrolled r2d2 wp1)
    (patrolled r2d2 wp2)
    (patrolled r2d2 wp3)
    (patrolled r2d2 wp4)
    (patrolled r2d2 wp5)
    (patrolled r2d2 wp6)
    (patrolled r2d2 wp7)
  )
)
)
