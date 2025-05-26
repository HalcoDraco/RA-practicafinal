(define (domain tresenraya)

  

  (:types 
    ficha tipo 
    lugar
    casilla fuera - lugar ; subtipos de lugar: casilla (parte del tablero) y fuera (fuera del tablero)
  )

  (:predicates
    (handEmpty)
    (holding ?f - ficha)
    (tieneTipo ?f - ficha ?t - tipo) ; la ficha tiene un tipo (X o O)
    (in ?f - ficha ?l - lugar)  ; la ficha está en un lugar (casilla o fuera)
    (ocupada ?l - lugar) ; la casilla está ocupada por una ficha
    (juegaX)   ; indica que le toca a X
    (juegaO)   ; indica que le toca a O
  )

  ;; PICK desde cualquier lugar (casilla o fuera)
  (:action pick
    :parameters (?f - ficha ?l - lugar)
    :precondition (and 
      (handEmpty) ; solo puede recoger si la mano está vacía
      (in ?f ?l) ; la ficha debe estar en el lugar especificado
      (or 
      (and (tieneTipo ?f x) (juegaX)) ; Si es X y le toca a X
      (and (tieneTipo ?f o) (juegaO)) ; Si es O y le toca a O
    )
          ;; Si está en una casilla, debe no haber más fichas de su tipo en 'fuera'
      (or
        (not (exists (?l - casilla) (in ?f ?l))) ; está en fuera
        (not (exists (?f2 - ficha ?l2 - fuera ?t - tipo)
          (and
            (tieneTipo ?f2 ?t)
            (tieneTipo ?f ?t)
            (in ?f2 ?l2)
            (not (= ?f2 ?f))
          )
            )
        )
      )
    )
    
    
    :effect (and
      (holding ?f)
      (not (handEmpty))
      (not (in ?f ?l)) ; la ficha ya no está en ese lugar
      (when (ocupada ?l) (not (ocupada ?l))) ;; si era una casilla, la desocupa
      (when (tieneTipo ?f x) (and (not (juegaX)) (juegaO))) ;; si es X, se pasa el turno a O
      (when (tieneTipo ?f o) (and (not (juegaO)) (juegaX))) ;; si es O, se pasa el turno a X
    )
  )

  ;; PLACE en una casilla libre
  (:action place
    :parameters (?f - ficha ?c - casilla)
    :precondition (and
      (holding ?f) ; el robot debe estar sujetando la ficha
      (not (ocupada ?c)) ; la casilla debe estar libre
    )
    :effect (and
      (in ?f ?c) ; la ficha pasa a estar en la casilla
      (ocupada ?c) ; la casilla pasa a estar ocupada
      (handEmpty)
      (not (holding ?f))
    )
  )
)