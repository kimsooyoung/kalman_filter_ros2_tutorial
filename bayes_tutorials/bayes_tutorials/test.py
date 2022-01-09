import numpy as np

belief  = [0,0.333,0,0.333,0,0,0,0.333,0,0]

def perfect_move(belief, distance):

    length = len(belief)

    new_belief = [0] * length

    for i in range(length):
        print(i, distance, (i+distance) % length)
        new_belief[(i+distance) % length] = belief[i]

    print(new_belief)

perfect_move(belief, 2)


p = [0.1, 0, 0.3, 0.6, 0]
print(np.random.choice([1,2,3,4,5], p=p))
