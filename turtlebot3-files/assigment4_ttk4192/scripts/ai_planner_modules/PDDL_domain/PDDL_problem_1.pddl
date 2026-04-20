(define (problem robplan_problem)
  (:domain robplan)

  (:objects
    turtlebot0 - robot
    camera_eo0 - camera_eo
    camera_ir0 - camera_ir
    robot_arm0 - robot_arm

    valve0 valve1 - valve
    pump0 pump1 - pump
    charger0 charger1 charger2 - charger

    waypoint0 waypoint1 waypoint2 waypoint3 waypoint4 waypoint5 waypoint6 - waypoint

    d01 d02 d03 d04 d05 d06 - route
    d10 d12 d13 d14 d15 d16 - route
    d20 d21 d23 d24 d25 d26 - route
    d30 d31 d32 d34 d35 d36 - route
    d40 d41 d42 d43 d45 d46 - route
    d50 d51 d52 d53 d54 d56 - route
    d60 d61 d62 d63 d64 d65 - route
  )

  (:init
    (= (speed turtlebot0) 0.18)

    ;; from wp0
    (connects d01 waypoint0 waypoint1)
    (connects d02 waypoint0 waypoint2)
    (connects d03 waypoint0 waypoint3)
    (connects d04 waypoint0 waypoint4)
    (connects d05 waypoint0 waypoint5)
    (connects d06 waypoint0 waypoint6)

    ;; from wp1
    (connects d10 waypoint1 waypoint0)
    (connects d12 waypoint1 waypoint2)
    (connects d13 waypoint1 waypoint3)
    (connects d14 waypoint1 waypoint4)
    (connects d15 waypoint1 waypoint5)
    (connects d16 waypoint1 waypoint6)

    ;; from wp2
    (connects d20 waypoint2 waypoint0)
    (connects d21 waypoint2 waypoint1)
    (connects d23 waypoint2 waypoint3)
    (connects d24 waypoint2 waypoint4)
    (connects d25 waypoint2 waypoint5)
    (connects d26 waypoint2 waypoint6)

    ;; from wp3
    (connects d30 waypoint3 waypoint0)
    (connects d31 waypoint3 waypoint1)
    (connects d32 waypoint3 waypoint2)
    (connects d34 waypoint3 waypoint4)
    (connects d35 waypoint3 waypoint5)
    (connects d36 waypoint3 waypoint6)

    ;; from wp4
    (connects d40 waypoint4 waypoint0)
    (connects d41 waypoint4 waypoint1)
    (connects d42 waypoint4 waypoint2)
    (connects d43 waypoint4 waypoint3)
    (connects d45 waypoint4 waypoint5)
    (connects d46 waypoint4 waypoint6)

    ;; from wp5
    (connects d50 waypoint5 waypoint0)
    (connects d51 waypoint5 waypoint1)
    (connects d52 waypoint5 waypoint2)
    (connects d53 waypoint5 waypoint3)
    (connects d54 waypoint5 waypoint4)
    (connects d56 waypoint5 waypoint6)

    ;; from wp6
    (connects d60 waypoint6 waypoint0)
    (connects d61 waypoint6 waypoint1)
    (connects d62 waypoint6 waypoint2)
    (connects d63 waypoint6 waypoint3)
    (connects d64 waypoint6 waypoint4)
    (connects d65 waypoint6 waypoint5)

    ;; route lengths
    ;; from wp0
    (= (route-length d01) 1.77)
    (= (route-length d02) 3.24)
    (= (route-length d03) 4.00)
    (= (route-length d04) 5.11)
    (= (route-length d05) 2.45)
    (= (route-length d06) 4.04)

    ;; from wp1
    (= (route-length d10) 1.77)
    (= (route-length d12) 1.53)
    (= (route-length d13) 2.51)
    (= (route-length d14) 3.43)
    (= (route-length d15) 2.14)
    (= (route-length d16) 2.33)

    ;; from wp2
    (= (route-length d20) 3.24)
    (= (route-length d21) 1.53)
    (= (route-length d23) 1.85)
    (= (route-length d24) 1.99)
    (= (route-length d25) 2.98)
    (= (route-length d26) 1.07)

    ;; from wp3
    (= (route-length d30) 4.00)
    (= (route-length d31) 2.51)
    (= (route-length d32) 1.85)
    (= (route-length d34) 3.16)
    (= (route-length d35) 2.42)
    (= (route-length d36) 1.08)

    ;; from wp4
    (= (route-length d40) 5.11)
    (= (route-length d41) 3.43)
    (= (route-length d42) 1.99)
    (= (route-length d43) 3.16)
    (= (route-length d45) 4.95)
    (= (route-length d46) 2.09)

    ;; from wp5
    (= (route-length d50) 2.45)
    (= (route-length d51) 2.14)
    (= (route-length d52) 2.98)
    (= (route-length d53) 2.42)
    (= (route-length d54) 4.95)
    (= (route-length d56) 3.09)

    ;; from wp6
    (= (route-length d60) 4.04)
    (= (route-length d61) 2.33)
    (= (route-length d62) 1.07)
    (= (route-length d63) 1.08)
    (= (route-length d64) 2.09)
    (= (route-length d65) 3.09)

    ;; robot initial position
    (at turtlebot0 waypoint0)

    ;; object locations
    (at valve0 waypoint1)
    (at valve1 waypoint2)
    (at pump0 waypoint5)
    (at pump1 waypoint6)

    ;; charger locations
    (at charger0 waypoint0)
    (at charger1 waypoint3)
    (at charger2 waypoint4)

    ;; equipment
    (available_equipment camera_eo0)
    (available_equipment camera_ir0)
    (available_equipment robot_arm0)

    ;; initial task state
    (no_valve_checked_eo valve0)
    (no_valve_checked_eo valve1)
    (no_photo_ir pump0)
    (no_photo_ir pump1)

    ;; battery
    (battery_ok turtlebot0)
  )

  (:goal
    (and
      (at turtlebot0 waypoint3)
      (valve_checked_eo valve0)
      (valve_checked_eo valve1)
      (photo_ir pump0)
      (photo_ir pump1)
    )
  )

  (:metric minimize (total-time))
)