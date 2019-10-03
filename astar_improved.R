astar_improved <- function (roads, car, packages) {
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
  
  car_mem = 0  # Keep track if the car already has a goal
  pack_scores = list()  # To keep track of scores for fetching different packages
  
  "
  Find optimal package to pick up based on score
  "
  # Check if car already has a goal
  if ((length(car$mem) > 0)) {
    x_goal = car$mem[[1]]
    y_goal = car$mem[[2]]
    if ((x_goal == car$x) && (car$y == y_goal)) {
      car$mem = list()
    }
  } 
  else if (car$load == 0) {
    toGo = which(packages[, 5] == 0)
    for (i in 1:length(toGo)) {
      x_goal = packages[toGo[i], offset]
      y_goal = packages[toGo[i], 1 + offset]
      
      # Need to reset frontier and expanded
      frontier = list(list(x = car$x, y = car$y, 
                           cost = 1000, h = h,
                           score = 1000, path = list()))  # Initialize frontier with dummy node
      expanded = list(x = car$x, y = car$y, cost = 0, h = h,
                      score = 0, path = list())  # Expanded node
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
        Need to expand the best node and append it to pack_scores
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
      pack_scores <- append(pack_scores, expanded$score)
    }
    # Time to select the package with the best score
    target = 1
    for (i in 1:length(pack_scores)) {
      if (pack_scores[[target]] > pack_scores[[i]]) {
        target = i
      }
    }
    x_goal = packages[toGo[target], offset]
    y_goal = packages[toGo[target], 1 + offset]
    car$mem <- append(car$mem, x_goal)
    car$mem <- append(car$mem, y_goal)
  }
  else {
    toGo = car$load  # Index of the package the car is carrying
    offset = 3
    x_goal = packages[toGo, offset]
    y_goal = packages[toGo, 1 + offset]
  }
  
  # If already on goal do nothing until next turn
  if ((x_goal == car$x) && (y_goal == car$y)) {
    nextMove = 5
    car$nextMove = nextMove
    return(car)
  }
  
  # Need to reset frontier and expanded
  frontier = list(list(x = car$x, y = car$y, 
                       cost = 1000, h = h,
                       score = 1000, path = list()))  # Initialize frontier with dummy node
  expanded = list(x = car$x, y = car$y, cost = 0, h = h,
                  score = 0, path = list())  # Expanded node
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