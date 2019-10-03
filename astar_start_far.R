astar_start_far <- function (roads, car, packages) {
  "
  Initialize variables that will be needed
  "
  h = 0  # Estimated cost of getting from current node to current goal (manhattan distance)
  offset = 1 # To find correct coordinates in 'packages'
  frontier = list(list(x = car$x, y = car$y, 
                       cost = 1000, h = h,
                       score = 1000, path = list()))  # Initialize frontier with dummy node
  expanded = list(x = car$x, y = car$y, cost = 0, h = h,
                  score = 0, path = list())  # Expanded node
  toGo = 0  # List of packages to fetch or specific package to deliver
  target = 0  # Specific index of the next package to pick up
  nextMove = 0  # Next move for the car
  x_goal = 0  # The x-coordinate the car currently wants to reach
  x_on = 1  # The x-coordinate the car is currently on
  x_check = 1  # The x-coordinate to check and put into the frontier
  y_goal = 0  # The y-coordinate the car currently wants to reach
  y_on = 1  # The y-coordinate the car is currently on
  y_check = 1  # The y-coordinate to check and put into the frontier
  h_distances = list(list(h = 1000, x = 1000, y = 1000))

  
  "
  Find optimal package to pick up based on manhattan distance
  "
  if (car$load == 0) {
    toGo = which(packages[, 5] == 0)
    if ((length(toGo) < 5) && (length(car$mem) == 0)) {
      closest_pack = 100
      for (i in 1:length(toGo)) {
        h = abs(car$x - packages[toGo[i], offset]) + abs(car$y - packages[toGo[i], 1+offset])
        if (h < closest_pack) {
          target = toGo[i]
          closest_pack = h
        }
      }
    }
    else {#if ((length(toGo) == 5) && (length(car$mem) == 0)) {
      closest_pack = -1
      # if?
      for (i in 1:length(toGo)) {
        h = abs(car$x - packages[toGo[i], offset]) + abs(car$y - packages[toGo[i], 1+offset])
        h_distances <- append(h_distances, list(list(h = h, x = packages[i, 1], y = packages[i, 2])))
      }
      #target_d = h_distances[[4]]
      #min_d = min(unlist(h_distances))
      #target_x = which(h_distances == packages[target, 2])
      #target_y = which(h_distances == packages[target, 3])
      #target_d = sort(unlist(h_distances))
      #target_d = 1000
      index = 0
      for (i in 1:2) {
        target_d = 1000
        for (k in 1:length(h_distances)) {
          if (target_d > h_distances[[k]]$h) {
            index = k
            target_d = h_distances[[k]]$h
          }
        }
        h_distances = h_distances[-index]
      }
      values = c(h_distances[[2]]$h, h_distances[[3]]$h, h_distances[[4]]$h)
      index = which(values == min(values))
      if (length(index) > 1) {
        index = index[1]
      }
      values_x = h_distances[[index+1]]$x
      #values_y = h_distances[[index]]$y
      target_x = which(packages[, 1] == values_x)
      if (length(target_x) > 1) {
        target_x = target_x[1]
      }
      #target_y = which(packages[, 2] == values_y)
        
      if ((length(toGo) == 5) && (length(car$mem) == 0)) {
        car$mem = list(1, packages[target_x, 1], packages[target_x, 2])
      }
    }
    # Scenario left is load=0 then (length(toGo) == 5) && (length(car$mem) != 0)
    if (length(car$mem) > 0) {
      x_goal = car$mem[[2]]
      y_goal = car$mem[[3]]
    }
    else { # if car$mem == 0 && length(toGo) != 5 going in
      closest_pack = 100
      toGo = which(packages[, 5] == 0)
      for (i in 1:length(toGo)) {
        h = abs(car$x - packages[toGo[i], offset]) + abs(car$y - packages[toGo[i], 1+offset])
        if (h < closest_pack) {
          target = toGo[i]
          closest_pack = h
        }
      }
      x_goal = packages[target, offset]
      y_goal = packages[target, 1 + offset]
    }
  }
  else {
    toGo = car$load  # Index of the package the car is carrying
    offset = 3
    x_goal = packages[toGo, offset]
    y_goal = packages[toGo, 1 + offset]
    car$mem = list()
  }
  
  # If already on goal do nothing until next turn
  if ((x_goal == car$x) && (y_goal == car$y)) {
    nextMove = 5
    car$nextMove = nextMove
    return(car)
  }
  
  "
  Loop through the algorithm until optimal path to goal is found
  "
  while ((expanded$x != x_goal) || (expanded$y != y_goal)) {
    "
    Need to check all neighbours and add them to the frontier
    "
    # Left, if there is a left
    if (expanded$x > 1) {
      x_on = expanded$x
      x_check = x_on - 1
      y_on = expanded$y
      h = abs(x_check - x_goal) + abs(y_on - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_check, y = y_on, cost = roads$hroads[x_on-1, y_on] + expanded$cost,
                  h = h, score = roads$hroads[x_on-1, y_on] + expanded$cost + h, 
                  path = node_path <- append(node_path, 4))  # Node to add to frontier
      is_in = 0  # To keep track if the frontier already has the coordinates we're checking
      for (i in 1:length(frontier)) {
        if ((frontier[[i]]$x == x_check) && (frontier[[i]]$y == y_on)) {
          is_in = i
        }
      }
      if (is_in == 0) {
        frontier <- append(frontier, list(node)) 
      }
      else if (frontier[[is_in]]$score > node$score) {
        frontier = frontier[-is_in]
        frontier <- append(frontier, list(node))
      }
    }
    
    # Right, if there is a right
    if (expanded$x < 10) {
      x_on = expanded$x
      x_check = x_on + 1
      y_on = expanded$y
      h = abs(x_check - x_goal) + abs(y_on - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_check, y = y_on, cost = roads$hroads[x_on, y_on] + expanded$cost,
                  h = h, score = roads$hroads[x_on, y_on] + expanded$cost + h,
                  path = node_path <- append(node_path, 6))  # Node to add to frontier
      is_in = 0  # To keep track if the frontier already has the coordinates we're checking
      for (i in 1:length(frontier)) {
        if ((frontier[[i]]$x == x_check) && (frontier[[i]]$y == y_on)) {
          is_in = i
        }
      }
      if (is_in == 0) {
        frontier <- append(frontier, list(node)) 
      }
      else if (frontier[[is_in]]$score > node$score) {
        frontier = frontier[-is_in]
        frontier <- append(frontier, list(node))
      }
    }
    
    # Up, if there is a up
    if (expanded$y < 10) {
      x_on = expanded$x
      y_on = expanded$y
      y_check = y_on + 1
      h = abs(x_on - x_goal) + abs(y_check - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_on, y = y_check, cost = roads$vroads[x_on, y_on] + expanded$cost,
                  h = h, score = roads$vroads[x_on, y_on] + expanded$cost + h,
                  path = node_path <- append(node_path, 8))  # Node to add to frontier
      is_in = 0  # To keep track if the frontier already has the coordinates we're checking
      for (i in 1:length(frontier)) {
        if ((frontier[[i]]$x == x_on) && (frontier[[i]]$y == y_check)) {
          is_in = i
        }
      }
      if (is_in == 0) {
        frontier <- append(frontier, list(node))
      }
      else if (frontier[[is_in]]$score > node$score) {
        frontier = frontier[-is_in]
        frontier <- append(frontier, list(node))
      }
    }
    
    # Down, if there is a down
    if (expanded$y > 1) {
      x_on = expanded$x
      y_on = expanded$y
      y_check = y_on - 1
      h = abs(x_on - x_goal) + abs(y_check - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_on, y = y_check, cost = roads$vroads[x_on, y_on-1] + expanded$cost,
                  h = h, score = roads$vroads[x_on, y_on-1] + expanded$cost + h,
                  path = node_path <- append(node_path, 2))  # Node to add to frontier
      is_in = 0  # To keep track if the frontier already has the coordinates we're checking
      for (i in 1:length(frontier)) {
        if ((frontier[[i]]$x == x_on) && (frontier[[i]]$y == y_check)) {
          is_in = i
        }
      }
      if (is_in == 0) {
        frontier <- append(frontier, list(node))
      }
      else if (frontier[[is_in]]$score > node$score) {
        frontier = frontier[-is_in]
        frontier <- append(frontier, list(node))
      }
    }
    
    "
    Need to expand the best node and move the car to it
    "
    best_index = list()  # To keep track of the index of the best node/s
    best_score = 1005  # To keep track of the best score found so far
    
    # Find the index/es with the best score
    k = 0
    for (i in 1:length(frontier)) {
      if (best_score > frontier[[i]]$score) {
        best_index <- append(best_index, i)
        if (k > 0) {
          best_index = best_index[-k]
        }
        else if (k == 0) {
          k = k + 1
        }
        best_score = frontier[[i]]$score
      }
      else if (best_score == frontier[[i]]$score) {
        best_index <- append(best_index, i)
        k = k + 1
      }
    }
    
    # Break potential ties and determine the final best index of frontier
    current = 0  # Variable simply to give easier syntax that works properly
    if (length(best_index) > 1) {
      new_index = 1
      for (i in 1:length(best_index)) {
        current = best_index[[i]] 
        if (frontier[[current]]$cost < frontier[[new_index]]$cost) {
          new_index = i
        }
      }
      best_index = best_index[[new_index]]
    }
    else {
      best_index = best_index[[1]]  # no ties so just grab
    }
    expanded = frontier[[best_index]]  # Reassign expanded to the new best node
    frontier = frontier[-best_index]  # Remove the new expanded from the frontier
  }
  
  # Assign new move to car
  nextMove = expanded$path[[1]]
  car$nextMove = nextMove
  return(car)
  
  # Notes
  "
  i. Might want to initialize frontier in a smarter way
  
  ii. 
  
  iii.
  
  iv.
  "
}