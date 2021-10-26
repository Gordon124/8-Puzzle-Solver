import sys
import puzz
import pdqpq

MAX_SEARCH_ITERS = 100000
GOAL_STATE = puzz.EightPuzzleBoard("012345678")

def solve_puzzle(start_state, strategy):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state: an EightPuzzleBoard object indicating the start state for the search
        flavor: a string indicating which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
            'path' - a list of 2-tuples representing the path from the start state to the goal state 
                (both should be included), with each entry being a (str, EightPuzzleBoard) pair 
                indicating the move and resulting state for each action.  Omitted if the search 
                fails.
            'path_cost' - the total cost of the path, taking into account the costs associated 
                with each state transition.  Omitted if the search fails.
            'frontier_count' - the number of unique states added to the search frontier at any
                point during the search.
            'expanded_count' - the number of unique states removed from the frontier and expanded 
                (i.e. have successors generated).
    """
    results = {
        'path': [],
        'path_cost': 0,
        'frontier_count': 0,
        'expanded_count': 0
    } # pass in results for each function
    state = puzz.EightPuzzleBoard(str(start_state))
    if strategy == 'bfs':
        return bfs(state, results)
    else:
        return ucs(state,results,str(strategy))

""" this function just determines the ucs_cost from each pair
    of predecessor and successor
"""
def t_cost(s, p):
    suc = s.pretty().replace("\n", "").split(" ") 
    pre = p.pretty().replace("\n", "").split(" ") 
    return int(pre[suc.index('.')])**2



def direction(curState,successor):
    for s in curState.successors():
        
        if curState.successors()[s] == successor:
            return s

def priortyCalc(curState,stateInDirection,searchType,curCost):
    if searchType == "bfs":
        return 0
    elif searchType == "ucost":
        return curCost + t_cost(stateInDirection,curState)
    elif searchType == "greedy-h1":
        return misPlaced(stateInDirection)
    elif searchType == "greedy-h2":
        return mhttanDistance(stateInDirection,False)
    elif searchType == "greedy-h3":
        return mhttanDistance(stateInDirection,True)
    elif searchType == "astar-h1":
        return misPlaced(stateInDirection) + curCost + t_cost(stateInDirection,curState) - misPlaced(curState) #+ astar_h(searchType, stateInDirection)
    elif searchType == "astar-h2":
        return mhttanDistance(stateInDirection,False) + curCost + t_cost(stateInDirection,curState) - mhttanDistance(curState,False)
    elif searchType == "astar-h3":
        return mhttanDistance(stateInDirection, True) + curCost + t_cost(stateInDirection,curState) - mhttanDistance(curState,True)
def mhttanDistance(curState,is_h3):
    curStateSplit = curState.pretty().replace("\n", "").split(" ")

    sum = 0
    counter = 0
    h3Sum = 0
    dict = {}
    dict["."] = [0,0]
    dict["1"] = [0,1]
    dict["2"] = [0,2]
    dict["3"] = [1,0]
    dict["4"] = [1,1]
    dict["5"] = [1,2]
    dict["6"] = [2,0]
    dict["7"] = [2,1]
    dict["8"] = [2,2]

    for i in range(3):
        for j in range(3):
            if(curStateSplit[counter]=="."):
                counter+=1
                continue
            mhatanDist = abs(dict[curStateSplit[counter]][0] - i) + abs(dict[curStateSplit[counter]][1] - j)
            sum += mhatanDist
            h3Sum += int(curStateSplit[counter])*int(curStateSplit[counter]) * mhatanDist

            counter +=1
    if is_h3:
        return h3Sum

    return sum

def misPlaced(curState):
    curStateSplit = curState.pretty().replace("\n", "").split(" ")
    goalState = GOAL_STATE.pretty().replace("\n", "").split(" ")
    result = 0
    for i in range(9):
        if curStateSplit[i] == ".":
            continue
        if goalState[i] != curStateSplit[i]:
            result+=1
    return result

def astar_h(searchType,curState):
    if searchType == "astar-h1":
        return misPlaced(curState) 
    elif searchType == "astar-h2":
        return mhttanDistance(curState,False) 
    elif searchType == "astar-h3":
        return mhttanDistance(curState,True) 
    
def bfs(state, results):
    # frontier count is the number added, explored count is the number popped
    explored = {} # object of states and predecessor state
    frontier = pdqpq.PriorityQueue() # creating queue
    frontier.add(state) # object with states
    results["frontier_count"] += 1
    c = state # used for backtracking once search is complete
    explored[c] = ("start",c) # start exploration
    flagDouble = False
    # check if state itself is goal
    
    while not frontier.empty(): # not equal goal 
        c = frontier.pop()
        # expanding curState
        results["expanded_count"] += 1 # marks each expansion
        for s in c.successors():
            sDir = c.successors()[s]
            #print(s)
            if c.pretty() == GOAL_STATE.pretty():
                flagDouble = True
                break
            if sDir not in frontier and sDir not in explored:

                explored[sDir] = (s,c)
                
                if sDir.pretty() == GOAL_STATE.pretty():
                    c = sDir
                    flagDouble = True
                    break
                else:
                    frontier.add(sDir)
                    results["frontier_count"] += 1
        #return 1
        if flagDouble:
            break
    if not flagDouble:
        print("failure")
        return results
    # after goal is found
    if c in explored:
        while (c != state):
            # Successor : [direction, curState]
            results["path"].insert(0, (explored[c][0],c))
            results["path_cost"] += t_cost(c, explored[c][1]) # change this to reflect the tile^2
            # print(c)
            c = explored[c][1] 
        results["path"].insert(0, (explored[c][0],c))
    return results

def ucs(state, results, searchType):
    # frontier count is the number added, explored count is the number popped
    explored = {} # object of states and predecessor state
    frontier = pdqpq.PriorityQueue() # creating queue
    frontier.add(state, 0) # object with states
    results["frontier_count"] += 1
    c = state # used for backtracking once search is complete
    b = False
    explored[c] = ("start",c) # start exploration
    # check if state itself is goal

    while not frontier.empty(): # not equal goal 
        curCost = frontier.peek(None)[1]
        c = frontier.pop()
        # expanding curState
        results["expanded_count"] += 1 # marks each expansion
        if c.pretty() == GOAL_STATE.pretty():
            b = True
            break
        for s in c.successors().values():
            if s not in frontier and s not in explored:
                explored[s] = (direction(c,s),c)
                frontier.add(s,priortyCalc(c,s,searchType,curCost))
                results["frontier_count"] += 1
            elif s in frontier and frontier.get(s) > priortyCalc(c,s,searchType,curCost):
                frontier.add(s,priortyCalc(c,s,searchType,curCost))
                explored[s] = (direction(c, s), c)
    # after goal is found
    if not b:
        print("failure")
        return results

    if c in explored:
        #results["path_cost"] = curCost 
        while (c != state):
            # Successor : [direction, curState]
            results["path"].insert(0, (explored[c][0],c))
            results["path_cost"] += t_cost(c, explored[c][1]) # change this to reflect the tile^2
            # print(c)
            c = explored[c][1] 
        results["path"].insert(0, (explored[c][0],c))
    return results


# have one function for all of these with different functions passed into it

def print_summary(results):
    if 'path' in results:
        print("found solution of length {}, cost {}".format(len(results['path']), 
                                                            results['path_cost']))
        for move, state in results['path']:
            print("  {:5} {}".format(move, state))
    else:
        print("no solution found")
    print("{} states placed on frontier, {} states expanded".format(results['frontier_count'], 
                                                                    results['expanded_count']))


############################################

if __name__ == '__main__':
    '''if(len(sys.argv[1]) != 9):
        print('invalid starting_state')
        exit()'''
    #print(mhttanDistance(puzz.EightPuzzleBoard("012345678"),))
    print(misPlaced(puzz.EightPuzzleBoard("142305678")))
    start = puzz.EightPuzzleBoard(sys.argv[1])
    method = sys.argv[2]
        
    print("solving puzzle {} -> {}".format(start, GOAL_STATE))
    results = solve_puzzle(start, method)
    print_summary(results)

    #python solver.py 802356174 bfs
    