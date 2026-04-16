(define (domain robplan)
  (:requirements :typing :durative-actions :strips :fluents)

  (:types
    subject route waypoint - object
    robot equipment valve pump charger - subject
    camera_eo robot_arm - equipment
  )

  (:predicates
    (at ?s - subject ?wp - waypoint)

    (available_equipment ?eq - equipment)

    (connects ?r - route ?from_wp - waypoint ?to_wp - waypoint)

    (no_photo ?pmp - pump)
    (photo ?pmp - pump)

    (no_valve_checked ?vlv - valve)
    (valve_checked ?vlv - valve)

    (no_manipulation ?vlv - valve)
    (manipulated ?vlv - valve)

    (battery_low ?rob - robot)
    (battery_ok ?rob - robot)
  )

  (:functions
    (route-length ?r - route)
    (speed ?rob - robot)
  )

  (:durative-action move_robot
    :parameters (?rob - robot ?from_wp - waypoint ?to_wp - waypoint ?r - route)
    :duration (= ?duration (/ (route-length ?r) (speed ?rob)))
    :condition (and
      (at start (at ?rob ?from_wp))
      (at start (connects ?r ?from_wp ?to_wp))
    )
    :effect (and
      (at start (not (at ?rob ?from_wp)))
      (at end (at ?rob ?to_wp))
    )
  )

  (:durative-action inspect_valve
    :parameters (?rob - robot ?wp - waypoint ?cam - camera_eo ?vlv - valve)
    :duration (= ?duration 10)
    :condition (and
      (over all (at ?rob ?wp))
      (at start (at ?vlv ?wp))
      (at start (available_equipment ?cam))
      (at start (no_valve_checked ?vlv))
    )
    :effect (and
      (at start (not (available_equipment ?cam)))
      (at start (not (no_valve_checked ?vlv)))
      (at end (valve_checked ?vlv))
      (at end (available_equipment ?cam))
    )
  )

  (:durative-action take_picture_pump
    :parameters (?rob - robot ?wp - waypoint ?cam - camera_eo ?pmp - pump)
    :duration (= ?duration 10)
    :condition (and
      (over all (at ?rob ?wp))
      (at start (at ?pmp ?wp))
      (at start (available_equipment ?cam))
      (at start (no_photo ?pmp))
    )
    :effect (and
      (at start (not (available_equipment ?cam)))
      (at start (not (no_photo ?pmp)))
      (at end (photo ?pmp))
      (at end (available_equipment ?cam))
    )
  )

  (:durative-action manipulate_valve
    :parameters (?rob - robot ?wp - waypoint ?arm - robot_arm ?vlv - valve)
    :duration (= ?duration 10)
    :condition (and
      (over all (at ?rob ?wp))
      (at start (at ?vlv ?wp))
      (at start (available_equipment ?arm))
      (at start (valve_checked ?vlv))
      (at start (no_manipulation ?vlv))
    )
    :effect (and
      (at start (not (available_equipment ?arm)))
      (at start (not (no_manipulation ?vlv)))
      (at end (manipulated ?vlv))
      (at end (available_equipment ?arm))
    )
  )

  (:durative-action charge_robot
    :parameters (?rob - robot ?wp - waypoint ?chg - charger)
    :duration (= ?duration 15)
    :condition (and
      (over all (at ?rob ?wp))
      (at start (at ?chg ?wp))
      (at start (battery_low ?rob))
    )
    :effect (and
      (at start (not (battery_low ?rob)))
      (at end (battery_ok ?rob))
    )
  )
)