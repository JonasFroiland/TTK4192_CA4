(define (domain robplan)

  (:requirements :typing :durative-actions :strips :fluents)



  (:types

    subject route waypoint - object

    robot equipment valve pump charger - subject

    camera_eo camera_ir robot_arm - equipment

  )



  (:predicates

    (at ?s - subject ?wp - waypoint)



    (available_equipment ?eq - equipment)



    (connects ?r - route ?from_wp - waypoint ?to_wp - waypoint)



    (no_photo_ir ?pmp - pump)

    (photo_ir ?pmp - pump)



    (no_valve_checked_eo ?vlv - valve)

    (valve_checked_eo ?vlv - valve)



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



  (:durative-action check_seals_valve_picture_EO

    :parameters (?rob - robot ?wp - waypoint ?cam - camera_eo ?vlv - valve)

    :duration (= ?duration 3)

    :condition (and

      (over all (at ?rob ?wp))

      (at start (at ?vlv ?wp))

      (at start (available_equipment ?cam))

      (at start (no_valve_checked_eo ?vlv))

    )

    :effect (and

      (at start (not (available_equipment ?cam)))

      (at start (not (no_valve_checked_eo ?vlv)))

      (at end (valve_checked_eo ?vlv))

      (at end (available_equipment ?cam))

    )

  )





  (:durative-action check_pump_picture_ir

    :parameters (?rob - robot ?wp - waypoint ?cam - camera_ir ?pmp - pump)

    :duration (= ?duration 3)

    :condition (and

      (over all (at ?rob ?wp))

      (at start (at ?pmp ?wp))

      (at start (available_equipment ?cam))

      (at start (no_photo_ir ?pmp))

    )

    :effect (and

      (at start (not (available_equipment ?cam)))

      (at start (not (no_photo_ir ?pmp)))

      (at end (photo_ir ?pmp))

      (at end (available_equipment ?cam))

    )

  )



  (:durative-action charge_battery

    :parameters (?rob - robot ?wp - waypoint ?chg - charger)

    :duration (= ?duration 10)

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