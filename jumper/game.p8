pico-8 cartridge // http://www.pico-8.com
version 41
__lua__

function _init()
  -- runs up to 53 agents with a {3,5,3} network at 60fps, but keeping it lower
  -- to draw visualizations
  generation = 0
  robots = {}
  network_architecture = {3,1,3}
  population = 60
  dead = population
  best = 0
  max_speed = -1.9
  fitnesses = {}

  winners = {}
  max_generations = 5
  current_round = 0
  total_rounds = 0
end

function pprint(robot)
  local network = robot.brain
  for i = 1, #network do
    local layer = network[i]
    local weights = layer.weights
    for j = 1, #weights do
      local w_j = weights[j]
      local s = ""
      for k = 1, #w_j do
        s = s .. w_j[k] .. " "
      end
      print(s)
    end
  end
end

function evolve_generation()
  sort(robots, function(a, b)
         return a.fitness < b.fitness
  end)

  if robots[1].fitness > best then
    best = robots[1].fitness
  end
  fitnesses[#fitnesses + 1] = robots[1].fitness

  -- select winners
  local f = {}
  local sum = 0
  local new_gen = {}
  local p_2 = population / 2
  for i = 1, p_2 do
    local robot = make_robot(16, 104, i, network_architecture)
    robot.brain = robots[i].brain
    robot.age = robot.age + 1
    local fitness = robots[i].fitness
    f[i] = fitness
    sum = sum + fitness
    new_gen[i] = robot
  end

  -- normalize distance
  -- f = softmax(f)  -- pocketing an idea
  for i=1, p_2 do
    f[i] = f[i] / sum
  end

  for i = 1, p_2 do
    local brain = deepcopy(new_gen[i].brain)

    -- elitism
    local wheel = rnd(1)
    local total_wheel = 0
    local parent1 = 1
    for j = 1, p_2 do
      total_wheel += f[j]
      if total_wheel > wheel and j ~= i then
        parent1 = j
        break
      end
    end

    -- or a random parent
    -- local parent1
    -- repeat
    --   parent1 = flr(rnd(p_2))+1
    -- until parent1 ~= i

    local n = crossover(robots[i], robots[parent1])
    brain.weights = n.weights
    brain.biases = n.biases
    mutate(brain, 1) -- or mutate faster when robots are older: robots[i].age)
    local robot = make_robot(16, 104, i + p_2, network_architecture)
    robot.brain = brain
    new_gen[i + p_2] = robot
  end

  robots = new_gen

  generation = generation + 1
end

function _update60()
  cls()

  start_time = stat(1)

  -- championship
  if dead == population then
    total_rounds = total_rounds + 1

    if generation == 0 and current_round == 0 then
      -- create a new random generation
      robots = {}
      for i = 1, population do
        robots[i] = make_robot(16, 104, i, network_architecture)
      end
      generation = 1
      dead = 0
    elseif generation < max_generations then
      evolve_generation()
      dead = 0
    else
      -- I don't have to evolve a generation, but it also selects winners, which
      -- is convinient
      evolve_generation()

      -- select winners
      local next_round = current_round + 1
      winners[next_round] = winners[next_round] or {}
      for i = 1, population / 2 do
        winners[next_round][#winners[next_round] + 1] = deepcopy(robots[i])
      end

      if #winners[next_round] == population then
        robots = deepcopy(winners[next_round])
        -- reshuffle population so the selection criteria won't affected by the
        -- priority
        shuffle(robots)
        winners[next_round] = {}
        current_round = next_round
        generation = 1
        dead = 0
      else
        current_round = 0
        generation = 0
        dead = population
      end
    end

    -- reset world
    t = 0
    world_speed = -1.0

    obstacle = {
      x = 136, 
      my = 112, 
      y = 112 - 8,  -- y - h,
      w = 8,
      h = 8,
      coll = {x = 0, y = 0, w = 8, h = 8}
    }
  else
    think()

    -- update state of world objects
    for i = 1, population do
      local robot = robots[i]
      local output = robot.brain.output

      local action = 1
      for i = 2, #output do
        if output[i] > output[action] then action = i end
      end

      -- update robot's state
      local grounded = robot.y == robot.my  -- ground detection

      if (action == 2 or action == 3) and grounded then  -- jumping
        local jump_speed = 0
        if action == 2 then
          jump_speed = -1.4
        else
          jump_speed = -1.8
        end

        robot.dy = jump_speed
        grounded = false
      end

      if grounded == false then  -- acceleration
        robot.y = robot.y + robot.dy

        if robot.y > robot.my then  -- land
          robot.dy = 0
          robot.y = robot.my
        else
          robot.dy = robot.dy + 0.05  -- gravity
        end
      end

      -- collision detection
      if not robot.dead then
        local coll1, coll2 = robot.coll, obstacle.coll
        local x1, c_x1 = robot.x, coll1.x
        local y1, c_y1 = robot.y, coll1.y
        local w1, h1 = coll1.w, coll1.h
        local x2, c_x2 = obstacle.x, coll2.x
        local y2, c_y2 = obstacle.y, coll2.y
        local w2, h2 = coll2.w, coll2.h
        if (x1 + c_x1 < x2 + c_x2 + w2 and
            x2 + c_x2 < x1 + c_x1 + w1 and
            y1 + c_y1 < y2 + c_y2 + h2 and
            y2 + c_y2 < y1 + c_y1 + h1) then
          robot.dead = true
          dead = dead + 1
        end
        robot.fitness = t
      end
    end

    local x = obstacle.x
    if x < 0 - obstacle.w then
      local h = 1 + flr(rnd(3))
      h *= 8
      x = 128
      obstacle.y = obstacle.my - h

      obstacle.h = h
      obstacle.coll.h = h
    end
    obstacle.x = x + world_speed
  end

  t = t + 1

  if t % 60 == 0 then
    world_speed -= 0.01
    world_speed = max(world_speed, max_speed)
  end
end

function _draw()
  for i = 0, 16 do
    spr(9, i * 8, 112)
    spr(10, i * 8, 120)
  end

  local cols = 4
  local width = 16
  local p_c = population / cols
  for i = 1, cols do
    for j = 1, p_c do
      local robot = robots[(i - 1) * p_c + j]
      color(robot.dead == true and 8 or 11)
      print(flr(robot.fitness / 10), (i - 1) * width, (j - 1) * 7 + 1)
    end
  end

  line(70, 3, 70, 53, 5)
  line(70, 53, 120, 53, 5)
  local g_start = max(1, #fitnesses - 50)
  local g_end = min(g_start + 50, #fitnesses)
  local x
  for i = g_start, g_end do
    x = 71 + i - g_start
    line(x, 53 - fitnesses[i] / 150, x, 52, 7)
  end
  x = 72 + g_end - g_start
  line(x, 53 - t / 150, x, 52, 7)

  for i = 1, population do
    local robot = robots[i]
    if robot.dead == false then 
      spr(2, robot.x, robot.y)
      print(robot.name, robot.x + 2, robot.y + 2, 4)
    end
  end

  local x = obstacle.x + obstacle.coll.x
  local y = obstacle.y + obstacle.coll.y
  for i = 1, obstacle.h / 8 do
    spr(8, x, y + (i - 1) * 8)
  end

  local stats = {world_speed,
                 t,
                 total_rounds .. " " ..
                   current_round .. " " ..
                   generation .. "/" ..
                   max_generations,
                 best}
  for i = 1, #stats do
    print(stats[i], 90, 52 + i*6)
  end
end

function think()
  for i = 1, population do
    local robot = robots[i]
    if robot.dead == false then
      local network = robot.brain

      local input = {
        mid(0, (obstacle.x - robot.x) / 128, 1),
        mid(0, obstacle.h / 24, 1),
        mid(0, world_speed / max_speed, 1)
      }

      -- feed forward, all layers
      for j = 1, #network do
        local layer = network[j]
        local biases = layer.biases
        local weights = layer.weights
        local n = #input
        local output = {}
        for k = 1, #weights do
          -- matmul
          local sum = 0
          local w_k = weights[k]
          for p = 1, n do
            sum = sum + w_k[p] * input[p]
          end

          -- gelu activation
          local gelu_x = sum + biases[k]
          local tanh_x = 0.79788456 * gelu_x + 0.03567741 * gelu_x * gelu_x * gelu_x
          local tanh = 2 / (1 + 2.71828183 ^ (-2 * tanh_x))   -- faster than 0.13533528 ^ tanh_x
          local gelu = 0.5 * gelu_x * tanh

          output[k] = gelu
        end

        input = output
      end

      -- this naming is confusing, but `input' essentially is the output layer
      -- after the full feed forward pass
      network.output = input
    end
  end
end

function make_robot(x, y, name, layers)
  network = {}
  robot = {
    brain = network,

    -- stats
    fitness = 0,
    age = 0,
    mutation = 0,
    crossover = 0,

    -- state
    name = name,
    x = x,
    y = y,
    my = y,  -- ground y
    dy = 0,  -- current y delta
    coll = {
      x = 1,
      y = 0,
      w = 6,
      h = 7,
    },
    dead = false,
  }

  -- initialize brain
  for i = 2, #layers do
    local layer = {
      -- neural network
      weights = {},
      biases = {},
      output = {},
    }

    local prev_layer = layers[i - 1]
    local current_layer = layers[i]
    for j = 1, current_layer do
      layer.biases[j] = rnd(2)-1

      local weights = {}
      for k = 1, prev_layer do
        weights[k] = rnd(2)-1
      end
      layer.weights[j] = weights
    end
    network[i - 1] = layer
  end

  return robot
end

function sort(a, cmp)
  for i=1, #a do
    local j = i
    while j > 1 and cmp(a[j-1], a[j]) do
      a[j], a[j-1] = a[j-1], a[j]
      j = j - 1
    end
  end
end

function shuffle(t)
  -- do a fisher-yates shuffle
  for i = #t, 1, -1 do
    local j = flr(rnd(i)) + 1
    t[i], t[j] = t[j], t[i]
  end
end

function softmax(x)
  local max_x = -999999
  for i = 1, #x do
    if x[i] > max_x then
      max_x = x[i]  -- find the maximum value in x for numerical stability
    end
   end
  
  local sum_exp = 0
  for i = 1, #x do
    x[i] = exp(x[i] - max_x)  -- subtract max_x for numerical stability
    sum_exp = sum_exp + x[i]
  end

  for i = 1, #x do
     x[i] = x[i] / sum_exp
  end

  return x
end

function deepcopy(orig, copies)
  copies = copies or {}
  local orig_type = type(orig)
  local copy
  if orig_type == 'table' then
    if copies[orig] then
      copy = copies[orig]
    else
      copy = {}
      copies[orig] = copy
      for orig_key, orig_value in next, orig, nil do
        copy[deepcopy(orig_key, copies)] = deepcopy(orig_value, copies)
      end
      setmetatable(copy, deepcopy(getmetatable(orig), copies))
    end
  else -- number, string, boolean, etc
    copy = orig
  end
  return copy
end

function crossover(parent_1, parent_2)
  local child = {}
  for i = 1, #parent_1.brain do
    -- Crossover for weights
    local weights_1 = parent_1.brain[i].weights
    local weights_2 = parent_2.brain[i].weights
    local weights = {}
    for j = 1, #weights_1 do
      weights[j] = {}
      local w1_j = weights_1[j]
      local w2_j = weights_2[j]
      for k = 1, #w1_j do
        weights[j][k] = (rnd() < 0.5) and w1_j[k] or w2_j[k]
      end
    end
    
    -- Crossover for biases
    local biases_1 = parent_1.brain[i].biases
    local biases_2 = parent_2.brain[i].biases
    local biases = {}
    for j = 1, #biases_1 do
      biases[j] = (rnd() < 0.5) and biases_1[j] or biases_2[j]
    end
    
    child[i] = {weights = weights, biases = biases}
  end

  return child
end

function mutate(network, age)
  local mutation_rate = 1
  if age > 10 then
    mutation_rate = age - 10
  end

  for i = 1, #network do
    -- Mutate weights
    local weights = network[i].weights
    for j = 1, #weights do
      local w_j = weights[j]
      for k = 1, #w_j do
        w = w_j[k]
        r = rnd()

        -- depending on a random num we choose a mutation
        if r <= 0.02 * mutation_rate then
          -- random gene
          w = rnd(2)-1
        elseif r <= 0.04 * mutation_rate then
          -- enhance/degrade a gene
          -- w = w + rnd(2) * 0.1 - 0.1
          factor = rnd(2)
          w *= factor
          w = mid(-1, w, 1)
        elseif r <= 0.06 * mutation_rate then
          -- opposite gene
          w = -w
        end

        if w_j[k] ~= w then
          w_j[k] = w
        end
      end
    end

    -- Mutate biases
    local biases = network[i].biases
      
    for j = 1, #biases do
      w = biases[j]
      local r = rnd()
      if r <= 0.02 * mutation_rate then
        factor = rnd(2)
        w = w * factor
        w = mid(-1, w, 1)
      elseif r <= 0.04 * mutation_rate then
        w = rnd(2) - 1
        -- w = w + rnd(2) * 0.1 - 0.1
      end

      if biases[j] ~= w then
        biases[j] = w 
      end
    end
  end
end

__gfx__
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000099999900999999009999990099999900000000000000000000000000aaaaaa003333330033333300000000000000000000000000000000000000000
00700700099999900999999009999990099999900000000000000000000000000aaaaaa003333330033333300000000000000000000000000000000000000000
00077000099999900999999009999990099999900000000000000000000000000aaaaaa003333330033333300000000000000000000000000000000000000000
00077000099999900999999009999990099999900000000000000000000000000aaaaaa003333330033333300000000000000000000000000000000000000000
00700700099999900999999009999990099999900000000000000000000000000aaaaaa003333330033333300000000000000000000000000000000000000000
00000000099999900999999009999990099999900000000000000000000000000aaaaaa003333330033333300000000000000000000000000000000000000000
