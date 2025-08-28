-- Pocket Computer Mini-Map Client with Correct Pathfinding

-- ===== CONFIG =====
local PORT = 63989
local mapWidth, mapHeight = 20, 10
local scale = 0.2
local refreshRate = 0.1

local lastCoords = nil
local facing = "Unknown"
local selectedBuilding = nil
local selectedBuildingName = nil
local pathPoints = {}
local pathComputed = false
local lastPlayerPos = nil

local colorsMap = {
    unknown = colors.white,
    road = colors.gray,
    building = colors.yellow,
    player = colors.red,
    path = colors.blue
}

-- ===== DATA =====
local roads = {}
local buildings = {}
local timerID = nil

-- ===== MODEM =====
local modem = peripheral.find("modem")
if not modem then 
    error("No modem attached") 
end
modem.open(PORT)

-- ===== UTILS =====
local function nodeKey(n) 
    return n.x..","..n.z 
end

local function checkIfIn(coords, area)
    if not area or #area < 4 then return false end
    local minX = math.min(area[1][1], area[2][1], area[3][1], area[4][1])
    local maxX = math.max(area[1][1], area[2][1], area[3][1], area[4][1])
    local minZ = math.min(area[1][3], area[2][3], area[3][3], area[4][3])
    local maxZ = math.max(area[1][3], area[2][3], area[3][3], area[4][3])
    return coords[1] >= minX and coords[1] <= maxX and coords[3] >= minZ and coords[3] <= maxZ
end

local function getCurrentArea(coords)
    for name, area in pairs(roads) do
        if checkIfIn(coords, area) then 
            return name 
        end
    end
    return "Unknown"
end

local function updateFacing(coords)
    if lastCoords then
        local dx = coords[1] - lastCoords[1]
        local dz = coords[3] - lastCoords[3]
        if dx ~= 0 or dz ~= 0 then
            if math.abs(dx) > math.abs(dz) then
                facing = dx > 0 and "East" or "West"
            else
                facing = dz > 0 and "South" or "North"
            end
        end
    end
    lastCoords = {coords[1], coords[2], coords[3]}
end

local function clear()
    term.setBackgroundColor(colors.black)
    term.clear()
    term.setCursorPos(1, 1)
end

-- ===== PATHFINDING =====

local function heuristic(a, b)
    return math.abs(a.x - b.x) + math.abs(a.z - b.z)
end

local function getNeighbors(node, nodes)
    local neighbors = {}
    local offsets = {{1,0}, {-1,0}, {0,1}, {0,-1}}
    for _, off in ipairs(offsets) do
        local nx, nz = node.x + off[1], node.z + off[2]
        local k = nx..","..nz
        if nodes[k] then 
            table.insert(neighbors, nodes[k]) 
        end
    end
    return neighbors
end

local function findNearestRoadNode(x, z, nodes)
    local nearest, dist = nil, math.huge
    for _, n in pairs(nodes) do
        local d = math.abs(n.x - x) + math.abs(n.z - z)
        if d < dist then 
            nearest, dist = n, d 
        end
    end
    return nearest
end

local DEBUG = true

local function log(msg)
    if DEBUG then
        local currentPos = {term.getCursorPos()}
        term.setCursorPos(1, mapHeight + 7)
        term.setTextColor(colors.white)
        term.setBackgroundColor(colors.black)
        term.clearLine()
        write(msg)
        if currentPos[1] and currentPos[2] then
            term.setCursorPos(currentPos[1], currentPos[2])
        end
    end
end

local function getRoadNodes()
    local nodes = {}
    local total = 0
    for _, area in pairs(roads) do
        if type(area) == "table" and #area >= 4 then
            local minX = math.min(area[1][1], area[2][1], area[3][1], area[4][1])
            local maxX = math.max(area[1][1], area[2][1], area[3][1], area[4][1])
            local minZ = math.min(area[1][3], area[2][3], area[3][3], area[4][3])
            local maxZ = math.max(area[1][3], area[2][3], area[3][3], area[4][3])
            for x = minX, maxX do
                for z = minZ, maxZ do
                    nodes[x..","..z] = {x = x, z = z}
                    total = total + 1
                end
            end
        end
    end
    log("Road nodes: "..total)
    return nodes
end

local function computePath(startCoords, building)
    pathPoints = {}
    pathComputed = false
    
    if not building or not building[1] or not building[3] then 
        log("Invalid building data")
        return 
    end

    log("Computing path...")
    local nodes = getRoadNodes()
    if not next(nodes) then
        log("No road nodes!")
        return
    end

    local startNode = findNearestRoadNode(startCoords[1], startCoords[3], nodes)
    local bx = (building[1][1] + building[3][1]) / 2
    local bz = (building[1][3] + building[3][3]) / 2
    local goalNode = findNearestRoadNode(bx, bz, nodes)

    if not startNode or not goalNode then
        log("No start or goal node!")
        return
    end

    log(string.format("Start: %d,%d Goal: %d,%d", startNode.x, startNode.z, goalNode.x, goalNode.z))

    -- A* pathfinding
    local openSet = {startNode}
    local openSetHash = {[nodeKey(startNode)] = true}
    local cameFrom = {}
    local gScore = {[nodeKey(startNode)] = 0}
    local fScore = {[nodeKey(startNode)] = heuristic(startNode, goalNode)}

    while #openSet > 0 do
        -- Find node with lowest fScore
        local currentIndex = 1
        local lowestF = fScore[nodeKey(openSet[1])] or math.huge
        for i = 2, #openSet do
            local f = fScore[nodeKey(openSet[i])] or math.huge
            if f < lowestF then
                lowestF = f
                currentIndex = i
            end
        end
        
        local current = table.remove(openSet, currentIndex)
        local currentKey = nodeKey(current)
        openSetHash[currentKey] = nil

        -- Goal reached
        if current.x == goalNode.x and current.z == goalNode.z then
            local key = currentKey
            while key do
                local n = nodes[key]
                table.insert(pathPoints, 1, {x = n.x, z = n.z})
                key = cameFrom[key]
            end
            pathComputed = true
            log("Path found! Length: "..#pathPoints)
            return
        end

        -- Check neighbors
        for _, neighbor in ipairs(getNeighbors(current, nodes)) do
            local nk = nodeKey(neighbor)
            local tentativeG = (gScore[currentKey] or math.huge) + 1
            
            if tentativeG < (gScore[nk] or math.huge) then
                cameFrom[nk] = currentKey
                gScore[nk] = tentativeG
                fScore[nk] = tentativeG + heuristic(neighbor, goalNode)
                
                if not openSetHash[nk] then
                    table.insert(openSet, neighbor)
                    openSetHash[nk] = true
                end
            end
        end
    end

    log("No path found!")
end

-- ===== DRAW =====
local function drawUI(coords)
    term.setCursorPos(1, 1)
    term.setBackgroundColor(colors.black)
    term.setTextColor(colors.white)
    
    term.clearLine()
    print(string.format("X=%d Y=%d Z=%d", coords[1], coords[2], coords[3]))
    term.clearLine()
    print("Area: "..getCurrentArea(coords))
    term.clearLine()
    print("Facing: "..facing)
    term.clearLine()
    
    local roadCount = 0
    local buildingCount = 0
    for _ in pairs(roads) do roadCount = roadCount + 1 end
    for _ in pairs(buildings) do buildingCount = buildingCount + 1 end
    
    print(string.format("Roads: %d  Buildings: %d", roadCount, buildingCount))
    term.clearLine()
    
    if selectedBuildingName then
        print("Target: "..selectedBuildingName)
        term.clearLine()
        if pathComputed and #pathPoints > 0 then
            print("Path: "..#pathPoints.." nodes")
        elseif selectedBuilding then
            print("Computing path...")
        end
    else
        print("Press 'I' to select building")
    end
end

local function drawMap(coords)
    local halfW, halfH = math.floor(mapWidth / 2), math.floor(mapHeight / 2)
    local px, pz = coords[1], coords[3]

    -- Prepare blank map
    local map = {}
    for y = 1, mapHeight do
        map[y] = {}
        for x = 1, mapWidth do
            map[y][x] = {char = " ", fg = colors.black, bg = colors.white}
        end
    end

    -- Draw roads
    for _, area in pairs(roads) do
        if type(area) == "table" and #area >= 4 then
            local minX = math.min(area[1][1], area[2][1], area[3][1], area[4][1])
            local maxX = math.max(area[1][1], area[2][1], area[3][1], area[4][1])
            local minZ = math.min(area[1][3], area[2][3], area[3][3], area[4][3])
            local maxZ = math.max(area[1][3], area[2][3], area[3][3], area[4][3])
            
            for x = minX, maxX do
                for z = minZ, maxZ do
                    local mx = math.floor((x - px) * scale) + halfW + 1
                    local mz = math.floor((z - pz) * scale) + halfH + 1
                    if mx >= 1 and mx <= mapWidth and mz >= 1 and mz <= mapHeight then
                        map[mz][mx].bg = colors.gray
                    end
                end
            end
        end
    end

    -- Draw buildings
    for _, b in pairs(buildings) do
        if type(b) == "table" and #b >= 4 then
            local minX = math.min(b[1][1], b[2][1], b[3][1], b[4][1])
            local maxX = math.max(b[1][1], b[2][1], b[3][1], b[4][1])
            local minZ = math.min(b[1][3], b[2][3], b[3][3], b[4][3])
            local maxZ = math.max(b[1][3], b[2][3], b[3][3], b[4][3])
            
            for x = minX, maxX do
                for z = minZ, maxZ do
                    local mx = math.floor((x - px) * scale) + halfW + 1
                    local mz = math.floor((z - pz) * scale) + halfH + 1
                    if mx >= 1 and mx <= mapWidth and mz >= 1 and mz <= mapHeight then
                        map[mz][mx].bg = colors.yellow
                    end
                end
            end
        end
    end

    -- Draw path (draw BEFORE player so path is visible under player)
    if pathPoints and #pathPoints > 0 then
        for _, node in ipairs(pathPoints) do
            local mx = math.floor((node.x - px) * scale) + halfW + 1
            local mz = math.floor((node.z - pz) * scale) + halfH + 1
            if mx >= 1 and mx <= mapWidth and mz >= 1 and mz <= mapHeight then
                map[mz][mx].bg = colors.blue
                map[mz][mx].char = "*"
            end
        end
    end

    -- Player in center (draw AFTER path so player is on top)
    if halfH + 1 >= 1 and halfH + 1 <= mapHeight and halfW + 1 >= 1 and halfW + 1 <= mapWidth then
        map[halfH + 1][halfW + 1].bg = colors.red
        map[halfH + 1][halfW + 1].char = "@"
    end

    -- Render the map
    term.setCursorPos(1, 6)
    for y = 1, mapHeight do
        for x = 1, mapWidth do
            term.setBackgroundColor(map[y][x].bg)
            term.setTextColor(map[y][x].fg)
            write(map[y][x].char)
        end
        if y < mapHeight then
            print()
        end
    end
    term.setBackgroundColor(colors.black)
    term.setTextColor(colors.white)
end

-- ===== BUILDING SELECTOR =====
local function buildingSelector()
    local list = {}
    for name, b in pairs(buildings) do 
        table.insert(list, {name = name, data = b}) 
    end
    
    if #list == 0 then
        clear()
        print("No buildings available!")
        print("Press any key to continue...")
        os.pullEvent("key")
        return nil, nil
    end
    
    local index = 1
    while true do
        clear()
        print("Select Building (Up/Down arrows, Enter, Q to cancel):")
        print()
        
        for i, b in ipairs(list) do
            if i == index then
                term.setBackgroundColor(colors.yellow)
                term.setTextColor(colors.black)
                print("> "..b.name)
                term.setBackgroundColor(colors.black)
                term.setTextColor(colors.white)
            else
                print("  "..b.name)
            end
        end
        
        local event, key = os.pullEvent("key")
        if key == keys.up and index > 1 then 
            index = index - 1
        elseif key == keys.down and index < #list then 
            index = index + 1
        elseif key == keys.enter then 
            return list[index].data, list[index].name
        elseif key == keys.q then 
            return nil, nil
        end
    end
end

-- ===== WAIT FOR SERVER DATA =====
clear()
print("Waiting for server data...")
print("Port: "..PORT)

while true do
    local event, side, senderChannel, replyChannel, message, distance = os.pullEvent("modem_message")
    if senderChannel == PORT and type(message) == "table" and message.ROADS and message.BUILDINGS then
        roads = message.ROADS
        buildings = message.BUILDINGS
        print("Data received!")
        sleep(1)
        break
    end
end

-- ===== MAIN LOOP =====
timerID = os.startTimer(refreshRate)
while true do
    local event, p1, p2 = os.pullEvent()
    
    if event == "timer" and p1 == timerID then
        local x, y, z = gps.locate()
        if x and y and z then
            local coords = {math.floor(x), math.floor(y) - 1, math.floor(z)}
            updateFacing(coords)
            
            -- Only compute path when building is first selected or player moved significantly
            if selectedBuilding and not pathComputed then
                computePath(coords, selectedBuilding)
                lastPlayerPos = {coords[1], coords[3]}
            elseif selectedBuilding and pathComputed and lastPlayerPos then
                -- Recompute if player moved more than 5 blocks
                local dx = math.abs(coords[1] - lastPlayerPos[1])
                local dz = math.abs(coords[3] - lastPlayerPos[2])
                if dx > 5 or dz > 5 then
                    computePath(coords, selectedBuilding)
                    lastPlayerPos = {coords[1], coords[3]}
                end
            end
            
            clear()
            drawUI(coords)
            drawMap(coords)
        else
            clear()
            print("GPS signal lost!")
            print("Retrying...")
        end
        timerID = os.startTimer(refreshRate)
        
    elseif event == "key" then
        if p1 == keys.i then
            local b, name = buildingSelector()
            if b and name then 
                selectedBuilding = b
                selectedBuildingName = name
                pathComputed = false  -- Reset path computation
                pathPoints = {}       -- Clear old path
                lastPlayerPos = nil   -- Reset position tracking
                clear()
                print("Building selected: "..name)
                print("Computing path...")
                sleep(0.5)
            end
            timerID = os.startTimer(refreshRate)
        elseif p1 == keys.c then
            -- Clear current path/target
            selectedBuilding = nil
            selectedBuildingName = nil
            pathComputed = false
            pathPoints = {}
            lastPlayerPos = nil
            clear()
            print("Path cleared!")
            sleep(0.5)
            timerID = os.startTimer(refreshRate)
        elseif p1 == keys.q then
            clear()
            print("Exiting...")
            break
        end
    end
end