Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates
location
block
side
table
in_sight    # block identification part is visible
holding     # object is held by an agent
hand_empty  # hand is free
on_table    # object X is on the table
identified  # object X as been identified, the agent knows which block it is
is
colored
bottom_facing
needs_flipping 
flipped
orientation_correct
observed    # observed X means that the side X has been observed
object_put_down
is_final_action

# keyword
NOT_OBSERVABLE
UNEQUAL
TRUE
FALSE

## constants
block_1
side_0
side_1
side_2
side_3
side_4
side_5
tableC

## initial state
START_STATE {
(table tableC) 
(location tableC)
(block block_1)
(side side_0) (side side_1) (side side_2) (side side_3) (side side_4) (side side_5)
(on_table block_1 tableC)
(bottom_facing side_5)
(hand_empty) 
(flipped FALSE)
(is_final_action FALSE)
}

EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE colored side_0)
}
{
(NOT_OBSERVABLE colored side_1)
}
{
(NOT_OBSERVABLE colored side_2)
}
{
(NOT_OBSERVABLE colored side_3)
}
{
(NOT_OBSERVABLE colored side_4)
}
{
(NOT_OBSERVABLE colored side_5)
}
}

BELIEF_START_STATE{ 
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
{
()=0.166666667
}
}

### Termination RULES 
Rule {
  { (identified block_1) (hand_empty)! } # 
  { (is_final_action FALSE)! (is_final_action TRUE) }
}

Rule {
  { (identified block_1) (orientation_correct) } # 
  { (is_final_action FALSE)! (is_final_action TRUE) }
}

Rule {
  { (identified block_1) (orientation_correct) (hand_empty)! } # 
  { (QUIT) }
}

Rule {
  { (identified block_1) (orientation_correct) (object_put_down) } # 
  { (QUIT) }
}

### Reward
REWARD {
}

### Tasks definitions
Include = 'LGP-blocks-multi-sides-actions-observations.g'
