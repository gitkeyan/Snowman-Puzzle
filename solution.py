#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Snowman Puzzle domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state, generate_coordinate_rect #for snowball specific classes and problems
from test_problems import PROBLEMS #20 test problems

#snowball HEURISTICS
def heur_simple(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''   
  return len(state.snowballs)

def heur_zero(state):
  return 0

def dist(coord1, coord2):
  '''return the manhattan distance between two points'''
  return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible snowball puzzle heuristic: manhattan distance'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between the snowballs and the destination for the Snowman is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    distance = 0
    for snowball in state.snowballs:
      distance += dist(snowball, state.destination)
    return distance

  
def heur_alternate(state): 
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
        
    big_s = None #the coordinate of the big snowball
    medium_s = None
    small_s = None
    m_b_s = None
    snowballs = state.snowballs
    destination = state.destination
    for snowball in snowballs:
      if snowballs[snowball] == 0:
        big_s = snowball
      if snowballs[snowball] == 1:
        medium_s = snowball
      if snowballs[snowball] == 2:
        small_s = snowball
      elif snowballs[snowball] == 3:
        m_b_s = snowball
        medium_s = snowball
        big_s = snowball
      elif snowballs[snowball] == 4:
        s_m_s = snowball
        small_s = snowball
        medium_s = snowball
      elif snowballs[snowball] == 5:
        s_b_s = snowball
        small_s = snowball
        big_s = snowball  
      elif snowballs[snowball] == 6:
        g_s = snowball
        small_s = snowball
        medium_s = snowball
        big_s = snowball


    if big_s == destination and medium_s != destination:#if big is at destination
      return  1.1 * (dist(destination,medium_s) + 0.99 * dist(destination,small_s))
    elif m_b_s == destination and small_s != destination:#if big and small are at destination
      return 1 * dist(destination,small_s)  
    else:
      return  1.2 * (dist(destination,big_s)+ 0.99 * dist(destination,medium_s) +  0.99 * dist(destination,small_s))   

    
def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SnowballState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    
    start_time = os.times()[0]
        
    se = SearchEngine('best_first', 'full')#use best_first
    se.init_search(initial_state, goal_fn=snowman_goal_state,heur_fn=heur_fn)
    solution = se.search(timebound)
    
    end_time = os.times()[0]
    elapsed_time = end_time - start_time
    
    while timebound > 0:
      timebound = timebound - elapsed_time #update timebound
      if solution == False:#if we haven't found a solution in the first attempt within the timebound
        return False
      else:
        start_time = os.times()[0]
        #prune based on gval, use solution.gval - 1 because we don't care about the solution with the same gval, only better ones
        better_solution = se.search(timebound, 
                                    costbound=(solution.gval - 1, float("inf"), float("inf")))
        end_time = os.times()[0]
        elapsed_time = end_time - start_time
        
        if better_solution:
          solution = better_solution
        else:#if there's no better solution, the the previous solution is the best one
          return solution
        
    return solution     

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
   
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
   
    start_time = os.times()[0]
        
    se = SearchEngine('custom', 'full') #using custom to compare the fval_function
    se.init_search(initial_state, goal_fn=snowman_goal_state,heur_fn=heur_fn,fval_function=wrapped_fval_function)
    solution = se.search(timebound)
    
    end_time = os.times()[0]
    elapsed_time = end_time - start_time
    while timebound > 0:
      timebound = timebound - elapsed_time
      if solution == False:
        return False
      else:
        start_time = os.times()[0]
        #prune based on gval + hval, use solution.gval - 1 because we don't care about the solution with the same gval+ hval, only better ones
        
        better_solution = se.search(timebound, 
                                    costbound=(float("inf"), float("inf"), solution.gval - 1))
        end_time = os.times()[0]
        elapsed_time = end_time - start_time
        
        if better_solution:
          solution = better_solution
        else:
          return solution
        
    return solution      

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 20 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=snowman_goal_state, heur_fn=heur_alternate)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10 
    final = anytime_weighted_astar(s0, heur_fn=heur_alternate, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 


