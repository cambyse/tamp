Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates
block
location
table
id	    # block identifier

in_sight   # block identification part is visible
holding     # object is held by an agent
hand_empty  # hand is free
on_table    # object X is on the table
on	    # object X is on object Y
clear       # object X top is clear
identified  # object X as been identified, the agnt knows which block it is
is

# keyword
NOT_OBSERVABLE
UNEQUAL

## constants
block_1
block_2 
block_3 
block_4
block_a  #block identifier
block_b  #block identifier
block_c  #block identifier
block_d  #block identifier
tableR

## initial state
START_STATE { 
(table tableR) 
(location tableR)
(block block_1) (block block_2) (block block_3) (block block_4)
(id block_a) (id block_b) (id block_c) (id block_d)
(UNEQUAL block_1 block_2) (UNEQUAL block_1 block_3) (UNEQUAL block_1 block_4)
(UNEQUAL block_2 block_3) (UNEQUAL block_2 block_4)
(UNEQUAL block_3 block_4)
(clear block_3) (clear block_4) (clear tableR)
(on_table block_1 tableR) (on_table block_2 tableR) (on block_3 block_1) (on block_4 block_2)
(hand_empty) 
}

EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE is block_4 block_d)
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE is block_2 block_a)
}
{
(NOT_OBSERVABLE is block_4 block_d)
(NOT_OBSERVABLE is block_2 block_c)
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE is block_3 block_a)
}
{
(NOT_OBSERVABLE is block_4 block_d)
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_4 block_d)
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE is block_3 block_a)
}
{
(NOT_OBSERVABLE is block_4 block_d)
(NOT_OBSERVABLE is block_2 block_c)
(NOT_OBSERVABLE is block_3 block_b)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_4 block_d)
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_3 block_b)
(NOT_OBSERVABLE is block_2 block_a)
}

{
(NOT_OBSERVABLE is block_3 block_d)
(NOT_OBSERVABLE is block_4 block_c)
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE is block_2 block_a)
}
{
(NOT_OBSERVABLE is block_2 block_d)
(NOT_OBSERVABLE is block_4 block_c)
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE is block_3 block_a)
}
{
(NOT_OBSERVABLE is block_3 block_d)
(NOT_OBSERVABLE is block_4 block_c)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_d)
(NOT_OBSERVABLE is block_4 block_c)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE is block_3 block_a)
}
{
(NOT_OBSERVABLE is block_2 block_d)
(NOT_OBSERVABLE is block_4 block_c)
(NOT_OBSERVABLE is block_3 block_b)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_d)
(NOT_OBSERVABLE is block_4 block_c)
(NOT_OBSERVABLE is block_3 block_b)
(NOT_OBSERVABLE is block_2 block_a)
}

{
(NOT_OBSERVABLE is block_3 block_d)
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_4 block_b)
(NOT_OBSERVABLE is block_2 block_a)
}
{
(NOT_OBSERVABLE is block_2 block_d)
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_4 block_b)
(NOT_OBSERVABLE is block_3 block_a)
}
{
(NOT_OBSERVABLE is block_3 block_d)
(NOT_OBSERVABLE is block_2 block_c)
(NOT_OBSERVABLE is block_4 block_b)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_d)
(NOT_OBSERVABLE is block_2 block_c)
(NOT_OBSERVABLE is block_4 block_b)
(NOT_OBSERVABLE is block_3 block_a)
}
{
(NOT_OBSERVABLE is block_2 block_d)
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_4 block_b)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_d)
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_4 block_b)
(NOT_OBSERVABLE is block_2 block_a)
}

{
(NOT_OBSERVABLE is block_3 block_d)
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE is block_4 block_a)
}
{
(NOT_OBSERVABLE is block_2 block_d)
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_3 block_b)
(NOT_OBSERVABLE is block_4 block_a)
}
{
(NOT_OBSERVABLE is block_3 block_d)
(NOT_OBSERVABLE is block_2 block_c)
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE is block_4 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_d)
(NOT_OBSERVABLE is block_2 block_c)
(NOT_OBSERVABLE is block_3 block_b)
(NOT_OBSERVABLE is block_4 block_a)
}
{
(NOT_OBSERVABLE is block_2 block_d)
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_1 block_b)
(NOT_OBSERVABLE is block_4 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_d)
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_2 block_b)
(NOT_OBSERVABLE is block_4 block_a)
}

}

BELIEF_START_STATE{ 
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
{
()=0.041666667
}
}

### Termination RULES 
Rule {
  { (on block_a block_b) (on block_b block_c) (on block_c block_d) (hand_empty) } # 
  { (QUIT) }
}

### Reward
REWARD {
#  tree{
#    leaf{ { (grasped handR screwdriverHandle) }, r=10. }
#    weight=1.
#  }
}
### Tasks definitions
Include = 'LGP-blocks-actions-observations.g'
Include = 'LGP-blocks-last-block-deduction-4-blocks.g'

