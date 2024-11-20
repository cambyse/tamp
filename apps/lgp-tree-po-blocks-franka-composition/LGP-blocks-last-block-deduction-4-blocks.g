#deduction of the last block if they have all been identified..(is it rigorous?)
Rule {
  W, X, Y, Z, T
  { (block W) (block X) (block Y) (block Z) (id T)
    (identified W) (identified X) (identified Y) (identified Z)!
    (UNEQUAL W X) (UNEQUAL W Y)
    (UNEQUAL X Y)
    (NOT_OBSERVABLE is Z T)}
  { (identified Z) (is Z T) (NOT_OBSERVABLE is Z T)!}
}
