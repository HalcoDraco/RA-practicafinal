(define (problem juego-inicial)
  (:domain tresenraya)

  (:objects
    UR3E - robot
    x1 x2 x3 - ficha
    o1 o2 o3 - ficha
    x o - tipo
    c1 c2 c3 c4 c5 c6 c7 c8 c9 - casilla
    f1 f2 f3 f4 - fuera 
  )
  (:init
    ;; estado del robot
    (handEmpty UR3E)

    ;; tipos de ficha
    (tieneTipo x1 x)
    (tieneTipo x2 x)
    (tieneTipo x3 x)
    (tieneTipo o1 o)
    (tieneTipo o2 o)
    (tieneTipo o3 o)

    ;; fichas colocadas
    (in x1 c7)   ; X en abajo izquierda
    (in o1 c1)   ; O en arriba izquierda

    ;; fichas aún por colocar
    (in x2 f1)
    (in o2 f3)
    (in x3 f2)
    (in o3 f4)

    ;; casillas ocupadas
    (ocupada c1)
    (ocupada c7)

    ;; turno actual
    (juegaX)
  )

  (:goal 
  (or
    ;; Primer objetivo
    (and
      (in x1 c6)
      (in x2 c3)
      (in x3 c9)
      (in o1 c1)
      (in o2 c5)
      (in o3 c8)
    )
    ;; Segundo objetivo alternativo
    (and
      (in x1 c6)
      (in x2 c3)
      (in x3 c5)   ;; diferente
      (in o1 c1)
      (in o2 c6)   ;; diferente
      (in o3 c8)
    )
  )
)
)