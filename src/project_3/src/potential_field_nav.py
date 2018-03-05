#!/usr/bin/env python

import numpy as np               # Linear algebra
import matplotlib.pyplot as plt  # Plotting
import math
# from scipy.spatial import distance # Euclidean distance

#-------------------------------------------------------------------------------
def GoToGoal(x_g, y_g):

  #-----------------------------------------------------------------------------
  # Initialization (you don't have to modify anything here)
  #-----------------------------------------------------------------------------

  # Define a map
  nX = 100
  nY = 100
  X, Y = np.meshgrid(np.linspace(0,nX,100), np.linspace(0,nY,100))

  # Define start position
  x_0 = 20
  y_0 = 20

  # Define Obstacles
  obsts = []
  obsts.append([50, 20])
  obsts.append([80, 35])

  #-----------------------------------------------------------------------------
  # Calculate potential field for each cell of the map
  #-----------------------------------------------------------------------------

  #Constants
  k_att = 1
  k_rep = 100000
  rho_0 = 10

  U = np.empty([len(X), len(Y)])
  V = np.empty([len(X), len(Y)])

  for xid in range(100):
    for yid in range(100):
      u_force, v_force = computeFoces([xid, yid], [x_g, y_g], obsts, k_att, k_rep, rho_0)

      U[xid, yid] = u_force
      V[xid, yid] = v_force

  #-----------------------------------------------------------------------------
  # Finding the robot path using the potential field
  #-----------------------------------------------------------------------------

  path = []
  path = computePath([x_0, y_0], [x_g, y_g], U, V)
  print "- - > COMPUTED PATH OF SIZE: ", len(path)
  #-----------------------------------------------------------------------------
  # Plot results (you don't have to modify anything here)
  #-----------------------------------------------------------------------------

  # Plot potential field. Every nth cell will be plotted
  nth = 1
  fig = plt.figure()
  Q = plt.quiver(Y[::nth, ::nth], X[::nth, ::nth], U[::nth, ::nth], V[::nth, ::nth],
        pivot='mid', units='width')


  plt.axis([-5, 105, -5, 105])
  plt.title('Robot path')
  plt.xlabel('X')
  plt.ylabel('Y')

  # Plot Path
  path_x = []
  path_y = []
  for i in range(len(path)-1):
    path_x.append(path[i][0])
    path_y.append(path[i][1])

  plt.plot(path_x, path_y, 'r', linewidth=4)

  # Plot Start and goal positions
  plt.plot([x_0], [y_0], 'bo', markersize=10)
  plt.plot([x_g], [y_g], 'go', markersize=10)

  obst1 = plt.Circle((50, 20), 5, color='#6699cc', alpha=0.4)
  obst2 = plt.Circle((80, 35), 5, color='#6699cc', alpha=0.4)
  ax = fig.add_subplot(111)
  ax.add_artist(obst1)
  ax.add_artist(obst2)
  # Show plot
  plt.show()

def computePath(startLoc, goalLoc, U, V):
  currLoc = startLoc;
  currDist2Goal = math.sqrt((goalLoc[1] - startLoc[1])**2 + (goalLoc[0] - startLoc[0])**2)
  finalPath = []
  finalPath.append(currLoc) # initial Location
  currSteps = 0
  maxSteps = 1000

  while currDist2Goal > 1:
    if currSteps > maxSteps:
      return finalPath

    curr_u_force = U[currLoc[0], currLoc[1]]
    curr_v_force = V[currLoc[0], currLoc[1]]

    neighLoc = []
    action_taken = -1;
    # action taken numbers
    # 1 - Right + x
    # 2 - Left - x
    # 2 - Up + y
    # 3 - Down - y

    if abs(curr_u_force) > abs(curr_v_force):
      if curr_u_force > 0:
        neighLoc = [currLoc[0]+1, currLoc[1]]
        action_taken = 1
      else:
        neighLoc = [currLoc[0]-1, currLoc[1]]
        action_taken = 2
    else:
      if curr_v_force > 0:
        neighLoc = [currLoc[0], currLoc[1]+1]
        action_taken = 3
      else:
        neighLoc = [currLoc[0], currLoc[1]-1]
        action_taken = 4

    # Checking if the state is in local minima    
    if len(finalPath) > 1:
      dist2PrevPoint = math.sqrt((finalPath[-2][1] - neighLoc[1])**2 + (finalPath[-2][0] - neighLoc[0])**2)
      if dist2PrevPoint < 0.1:

        # If the local minima is in x - axis take up or down action
        # depending on where the goal is
        if action_taken < 3:

          if (currLoc[1] - goalLoc[1]) < 0:
            neighLoc = [currLoc[0], currLoc[1]+1] # Up Action
            if neighLoc[1] >= 100:
              neighLoc = [currLoc[0], currLoc[1]-1] # Down Action
          else:
            neighLoc = [currLoc[0], currLoc[1]-1] # Down Action
            if neighLoc[1] < 0:
              neighLoc = [currLoc[0], currLoc[1]+1] # Up Action
        else:
          # If the local minima is in y - axis take right or left action
          # depending on where the goal is

          if (currLoc[0] - goalLoc[0]) < 0:
            neighLoc = [currLoc[0]+1, currLoc[1]] # Right Action
            if neighLoc[0] >= 100:
              neighLoc = [currLoc[0]-1, currLoc[1]] # Left Action
          else:
            neighLoc = [currLoc[0]-1, currLoc[1]] # Left Action
            if neighLoc[0] >= 100:
              neighLoc = [currLoc[0]+1, currLoc[1]] # Right Action
          
    if neighLoc[0] > 100:
      if (currLoc[0] - goalLoc[0]) < 0:
        neighLoc = [currLoc[0], currLoc[1]+1] # Up Action
      else:
        neighLoc = [currLoc[0], currLoc[1]-1] # Down Action

    if neighLoc[0] <= 0:
      if (currLoc[0] - goalLoc[0]) < 0:
        neighLoc = [currLoc[0], currLoc[1]+1] # Up Action
      else:
        neighLoc = [currLoc[0], currLoc[1]-1] # Down Action

    if neighLoc[1] > 100:
      if (currLoc[1] - goalLoc[1]) < 0:
        neighLoc = [currLoc[0]+1, currLoc[1]] # Right Action
      else:
        neighLoc = [currLoc[0]-1, currLoc[1]] # Left Action 

    if neighLoc[1] <= 0:
      if (currLoc[1] - goalLoc[1]) < 0:
        neighLoc = [currLoc[0]+1, currLoc[1]] # Right Action
      else:
        neighLoc = [currLoc[0]-1, currLoc[1]] # Left Action                         

    if len(neighLoc) > 0:    
      currLoc = neighLoc
    else:
      print "No Neighbor to Visit..!!!"

    currDist2Goal = math.sqrt((goalLoc[1] - currLoc[1])**2 + (goalLoc[0] - currLoc[0])**2)
    finalPath.append(currLoc)
    currSteps = currSteps + 1

  finalPath.append(goalLoc) # initial Location
  return finalPath

def computeFoces(currLoc, goalLoc, obsts, k_att, k_rep, rho_0):
  # print "k_att, k_rep, rho_0: ", k_att, k_rep, rho_0
  currAngle2Goal = math.atan2(goalLoc[1] - currLoc[1], goalLoc[0] - currLoc[0])
  currDist2Goal = math.sqrt((goalLoc[1] - currLoc[1])**2 + (goalLoc[0] - currLoc[0])**2)
  f_attrat_mag = k_att*currDist2Goal
  f_attraction_u = f_attrat_mag*math.cos(currAngle2Goal)
  f_attraction_v = f_attrat_mag*math.sin(currAngle2Goal)
  f_attraction = [f_attraction_u, f_attraction_v]                                                   

  f_repel_u = 0
  f_repel_v = 0

  for oid in range(len(obsts)):
    currAngle2Obst = math.atan2(obsts[oid][1] - currLoc[1], obsts[oid][0] - currLoc[0])
    currDist2Obst = math.sqrt((obsts[oid][1] - currLoc[1])**2 + (obsts[oid][0] - currLoc[0])**2)

    if (currDist2Obst == 0):
      f_rep1_x = 0;
      f_rep1_y = 0;
    elif (currDist2Obst < rho_0): 
      grad_x = (currLoc[0] - obsts[oid][0])/currDist2Obst
      grad_y = (currLoc[1] - obsts[oid][1])/currDist2Obst
      rho_ob1 = currDist2Obst
      f_rep1_x = k_rep * ( (1/rho_ob1) - ( 1/rho_0 ) ) * ( 1/ (rho_ob1**2) ) * grad_x;
      f_rep1_y = k_rep * ( (1/rho_ob1) - ( 1/rho_0 ) ) * ( 1/ (rho_ob1**2) ) * grad_y;
    else:
      f_rep1_x = 0;
      f_rep1_y = 0;

    # if f_rep1_x < 0 :
    #   print "Repel Force: ", oid, currLoc, currDist2Obst, f_rep1_x, f_repel_u

    f_repel_u = f_repel_u + f_rep1_x
    f_repel_v = f_repel_v + f_rep1_y

  f_total_u = f_attraction_u + f_repel_u
  f_total_v = f_attraction_v + f_repel_v

  # if f_repel_u < 0 :
  #   print "Total Force: ", currLoc, f_attraction_u, f_repel_u, f_total_u

  # if currLoc[0] < 40:
  #   if currLoc[1] > 60:
  #     print "Total Force: ", currLoc, f_attraction_u, f_attraction_v, f_total_u, f_total_v

  return f_total_u, f_total_v  
#-------------------------------------------------------------------------------
if __name__ == '__main__':
  #x_g = np.random.randint(99)
  #y_g = np.random.randint(99)
  x_g = 90
  y_g = 30
  GoToGoal(x_g, y_g)
