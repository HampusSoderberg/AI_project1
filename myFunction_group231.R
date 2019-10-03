myFunction <- function (roads, car, packages) {
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
  toGo = 0  # List of index for package/s left to pick up or index for package to deliver
  target = 0  # Specific index of the next package to pick up
  nextMove = 0  # Next move for the car
  x_goal = 0  # The x-coordinate the car currently wants to reach
  x_on = 1  # The x-coordinate the car is currently on
  x_check = 1  # The x-coordinate to check and put into the frontier
  y_goal = 0  # The y-coordinate the car currently wants to reach
  y_on = 1  # The y-coordinate the car is currently on
  y_check = 1  # The y-coordinate to check and put into the frontier
  
  "
  Define the goal location for the car. Either by which package is closest to pick up 
  measured with manhattan distance, if the car is not carrying a package. Or by the delivery
  location of the package the car is carrying, if it is carrying a package.
  "
  if (car$load == 0) {
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
  else {
    toGo = car$load  # Index of the package the car is carrying
    offset = 3
    x_goal = packages[toGo, offset]
    y_goal = packages[toGo, 1 + offset]
  }
  
  "
  If the car is already on the goal location, assign the next move as standing still in
  wait for the next turn and return the car.
  "
  if ((x_goal == car$x) && (y_goal == car$y)) {
    nextMove = 5
    car$nextMove = nextMove
    return(car)
  }
  
  "
  The actual A* algorithm needs to be looped through until the currently optimal path to
  the current goal is found
  "
  while ((expanded$x != x_goal) || (expanded$y != y_goal)) {
    "
    Need to check all neighbours and add them to the frontier
    "
    # Check the neighbour to the left, if one exist
    if (expanded$x > 1) {
      x_on = expanded$x
      x_check = x_on - 1
      y_on = expanded$y
      h = abs(x_check - x_goal) + abs(y_on - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_check, y = y_on, cost = roads$hroads[x_on-1, y_on] + expanded$cost,
                  h = h, score = roads$hroads[x_on-1, y_on] + expanded$cost + h, 
                  path = node_path <- append(node_path, 4))  # Node to add to frontier
      is_in = 0  # Keep track if, and where, the frontier has the coordinates we're checking
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
    
    # Check the neighbour to the right, if one exist
    if (expanded$x < 10) {
      x_on = expanded$x
      x_check = x_on + 1
      y_on = expanded$y
      h = abs(x_check - x_goal) + abs(y_on - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_check, y = y_on, cost = roads$hroads[x_on, y_on] + expanded$cost,
                  h = h, score = roads$hroads[x_on, y_on] + expanded$cost + h,
                  path = node_path <- append(node_path, 6))  # Node to add to frontier
      is_in = 0  # Keep track if, and where, the frontier has the coordinates we're checking
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
  
    # Check the neighbour above, if one exist
    if (expanded$y < 10) {
      x_on = expanded$x
      y_on = expanded$y
      y_check = y_on + 1
      h = abs(x_on - x_goal) + abs(y_check - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_on, y = y_check, cost = roads$vroads[x_on, y_on] + expanded$cost,
                  h = h, score = roads$vroads[x_on, y_on] + expanded$cost + h,
                  path = node_path <- append(node_path, 8))  # Node to add to frontier
      is_in = 0  # Keep track if, and where, the frontier has the coordinates we're checking
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
    
    # Check the neighbour to the below, if one exist
    if (expanded$y > 1) {
      x_on = expanded$x
      y_on = expanded$y
      y_check = y_on - 1
      h = abs(x_on - x_goal) + abs(y_check - y_goal)
      node_path = expanded$path  # To keep track of the path
      node = list(x = x_on, y = y_check, cost = roads$vroads[x_on, y_on-1] + expanded$cost,
                  h = h, score = roads$vroads[x_on, y_on-1] + expanded$cost + h,
                  path = node_path <- append(node_path, 2))  # Node to add to frontier
      is_in = 0  # Keep track if, and where, the frontier has the coordinates we're checking
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
    Need to find and expand the best node in the frontier
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
      best_index = best_index[[1]]  # No tie for best score so just grab the best one
    }
    expanded = frontier[[best_index]]  # Reassign expanded to the new best node in frontier
    frontier = frontier[-best_index]  # Remove the new expanded node from the frontier
  }
  
  "
  The expanded node now contains the best path to the current goal. We assign the cars next
  move as the first move in that path and then finally return car.
  "
  nextMove = expanded$path[[1]]
  car$nextMove = nextMove
  return(car)
  
}
