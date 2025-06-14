(define (domain tresenraya)

  

  (:types 
    robot ficha tipo 
    lugar
    casilla fuera - lugar ; subtipos de lugar: casilla (parte del tablero) y fuera (fuera del tablero)
  )

  (:predicates
    (handEmpty ?r - robot)
    (holding ?r - robot ?f - ficha)
    (tieneTipo ?f - ficha ?t - tipo) ; la ficha tiene un tipo (X o O)
    (casillaEstaFuera ?l - lugar) ; la casilla está fuera del tablero
    (casillaEstaDentro ?l - lugar) ; la casilla está dentro del tablero
    (in ?f - ficha ?l - lugar)  ; la ficha está en un lugar (casilla o fuera)
    (ocupada ?l - lugar) ; la casilla está ocupada por una ficha
    (juegaX)   ; indica que le toca a X
    (juegaO)   ; indica que le toca a O
  )

  ;; PICK desde cualquier lugar (casilla o fuera)
  (:action pick
    :parameters (?r - robot ?f - ficha ?l - lugar)
    :precondition 
      (and 
        (handEmpty ?r) ; solo puede recoger si la mano está vacía
        (in ?f ?l) ; la ficha debe estar en el lugar especificado
        (or 
          (and  
            (tieneTipo ?f x) 
            (juegaX) 
            (or 
              (casillaEstaFuera ?l)
              (and
                (casillaEstaDentro ?l)
                (not 
                  (exists (?f2 - ficha ?l2 - lugar)
                    (and (tieneTipo ?f2 x) (casillaEstaFuera ?l2))
                  )
                )
              )
            )
          ); Si es X y le toca a X
          (and 
            (tieneTipo ?f o)
            (juegaO)
            (or 
              (casillaEstaFuera ?l)
              (and
                (casillaEstaDentro ?l)
                (not 
                  (exists (?f2 - ficha ?l2 - lugar)
                    (and (tieneTipo ?f2 o) (casillaEstaFuera ?l2))
                  )
                )
              )
            )
          ) ; Si es O y le toca a O
        )
      )
    :effect (and
      (holding ?r ?f)
      (not (handEmpty ?r))
      (not (in ?f ?l)) ; la ficha ya no está en ese lugar
      (when (ocupada ?l) (not (ocupada ?l))) ;; si era una casilla, la desocupa
      (when (tieneTipo ?f x) (and (not (juegaX)) (juegaO))) ;; si es X, se pasa el turno a O
      (when (tieneTipo ?f o) (and (not (juegaO)) (juegaX))) ;; si es O, se pasa el turno a X
    )
  )

  ;; PLACE en una casilla libre
  (:action place
    :parameters (?r - robot ?f - ficha ?c - casilla)
    :precondition (and
      (holding ?r ?f) ; el robot debe estar sujetando la ficha
      (not (ocupada ?c)) ; la casilla debe estar libre
    )
    :effect (and
      (in ?f ?c) ; la ficha pasa a estar en la casilla
      (ocupada ?c) ; la casilla pasa a estar ocupada
      (handEmpty ?r)
      (not (holding ?r ?f))
    )
  )
)
