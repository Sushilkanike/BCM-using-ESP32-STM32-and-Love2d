-- This is main.lua
-- This program now relies on UDP packets from an ESP32.

-- REQUIRES: LuaSocket. Place the 'socket'folders
-- in the same directory as this file.

HEIGHT = 850
WIDTH  = 1070

-- *** IMPORTANT ***
-- Make sure this IP is correct! Check your ESP32's Serial Monitor.
ESP_IP = "10.232.11.234"
HOST = "127.0.0.1"
LOCAL_IP = "10.232.11.65" --localhost was not working. had to use WLAN IP
PORT = 8888

-- Global variable for connection status
Connection_Status = "Initializing..."

--[[
    Helper function to map a value from one range to another.
--]]
function map(value, in_min, in_max, out_min, out_max)
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
end

-- --- Custom Drawing Functions for Icons ---

--[[
    Draws a filled polygon arrow for turn indicators.
--]]
function drawArrowIcon(x, y, size, direction, color)
    love.graphics.setColor(color)
    local points
    if direction == "left" then
        points = { --left
            x, y,
            x + size, y - size * 0.7,
            x + size, y - size * 0.3,
            x + size * 0.6, y - size * 0.3,
            x + size * 0.6, y + size * 0.3,
            x + size, y + size * 0.3,
            x + size, y + size * 0.7
        }
    else -- right
        points = {
            x, y,
            x - size, y - size * 0.7,
            x - size, y - size * 0.3,
            x - size * 0.6, y - size * 0.3,
            x - size * 0.6, y + size * 0.3,
            x - size, y + size * 0.3,
            x - size, y + size * 0.7
        }
    end
    love.graphics.polygon("fill", points)
end

--[[
    Draws the high beam (headlight) icon.
--]]
function drawHighBeamIcon(x, y, color)
    love.graphics.setColor(color)
    love.graphics.setLineWidth(3)
    love.graphics.arc("line", "open", x, y, 15, -math.pi/2.5, math.pi/2.5) -- "D" shape
    love.graphics.line(x, y - 13, x, y + 13)
    love.graphics.setLineWidth(2)
    love.graphics.line(x + 18, y - 12, x + 28, y - 12)
    love.graphics.line(x + 18, y - 4, x + 28, y - 4)
    love.graphics.line(x + 18, y + 4, x + 28, y + 4)
    love.graphics.line(x + 18, y + 12, x + 28, y + 12)
    love.graphics.setLineWidth(1)
end

--[[
    Draws the check engine warning icon.
--]]
function drawCheckEngineIcon(x, y, color)
    love.graphics.setColor(color)
    love.graphics.setLineWidth(2)
    love.graphics.rectangle("line", x - 20, y - 15, 40, 30, 5)
    love.graphics.line(x - 15, y - 15, x - 15, y - 20)
    love.graphics.line(x, y - 15, x, y - 20)
    love.graphics.line(x + 15, y - 15, x + 15, y - 20)
    love.graphics.line(x - 15, y + 15, x - 10, y + 20)
    love.graphics.line(x + 15, y + 15, x + 10, y + 20)
    love.graphics.setLineWidth(1)
end

--[[
    Draws the battery warning icon.
--]]
function drawBatteryIcon(x, y, color)
    love.graphics.setColor(color)
    love.graphics.setLineWidth(2)
    love.graphics.rectangle("line", x - 25, y - 10, 50, 20, 3)
    love.graphics.rectangle("fill", x + 20, y - 5, 10, 10) -- Positive terminal
    love.graphics.setLineWidth(1)
end

--[[
    Draws the oil pressure warning icon.
--]]
function drawOilPressureIcon(x, y, color)
    love.graphics.setColor(color)
    love.graphics.setLineWidth(2)
    love.graphics.circle("line", x, y, 10) -- Pot/can part
    love.graphics.line(x, y - 10, x, y - 20) -- Spout
    love.graphics.line(x - 5, y - 15, x + 5, y - 15) -- Drip
    love.graphics.setLineWidth(1)
end

--[[
    Helper function to parse boolean strings
    "true" -> true
    "false" -> false
--]]
function parseBool(str)
    if str == nil then return false end
    return str == "true"
end

-- --- Main LÖVE Callbacks ---

--[[
    Called once at the beginning of the program.
--]]
function love.load()
    love.window.setMode(WIDTH, HEIGHT)
    love.graphics.setBackgroundColor(0.08, 0.08, 0.12)

    -- Require the luasocket library
    socket = require("socket")

    -- Main table to hold all dashboard state data
    dashboardData = {
        speed = 0,
        fuel = 0,
        gear = "N",
        odometer = 0,
        indicatorLeft = false,
        indicatorRight = false,
        highBeams = false,
        checkEngine = false,
        batteryWarning = false,
        oilWarning = false,
        IMU = { ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0 }
    }
    
    -- Table to hold debugging information
    debugInfo = {
        lastPacketTime = 0
    }
    
    -- Load images
    dashboardFrameImage = love.graphics.newImage("dashboard_frame.png")
    speedometerFaceImage = love.graphics.newImage("speedometer_face.png")

    -- Load fonts
    hugeFont = love.graphics.newFont(100)
    largeFont = love.graphics.newFont(45)
    mediumFont = love.graphics.newFont(25)
    smallFont = love.graphics.newFont(16)
    tinyFont = love.graphics.newFont(12)
    
    -- Timers
    blinkerTimer = 0
    blinkerOn = false
    logTimer = 0
    logInterval = 0.5 
    maxLogEntries = 6 
    consoleMessages = {}
    
    -- --- Network State ---
    isConnected = false   -- Tracks if we've received a packet
    pingTimer = 0         -- Timer for re-sending the ping
    pingInterval = 1.0    -- Ping every 1.0 second

    -- --- Network Setup ---
    --# Note: Windows. echo "SPEED=65,..." | ncat -u 127.0.0.1 8888
    udp = socket.udp()
    if udp then
        -- Bind to all interfaces ("*") on port 8888
        local ok, err = udp:setsockname(LOCAL_IP, PORT)
        if ok then
            -- Set to non-blocking. This is CRITICAL for a game loop.
            udp:settimeout(0)
            Connection_Status = "Ready to ping ESP32..."
        else
            Connection_Status = "Error: Failed to bind port 8888."..err
        end
    else
        Connection_Status = "Error: Failed to open UDP socket."..err
    end
end

--[[
    Called repeatedly on every frame.
    Updates all game logic, physics, and state variables.
    @param dt: "Delta time" - the time in seconds since the last frame.
--]]
function love.update(dt)
    -- Increment timers
    blinkerTimer = blinkerTimer + dt
    logTimer = logTimer + dt
    
    -- If we are not connected, keep pinging
    if not isConnected then
        pingTimer = pingTimer + dt
        --Connection_Status = "Pinging again"
        if pingTimer >= pingInterval then
            pingTimer = pingTimer - pingInterval -- Reset timer
            
            local ok, err = udp:sendto("PING", ESP_IP, PORT)
            
            if ok then
                Connection_Status = "Pinging " .. ESP_IP .. "..."
                table.insert(consoleMessages, 1, "Sent PING to " .. ESP_IP)
            else
                Connection_Status = "Error sending PING: " .. tostring(err)
            end
        end
    end

    -- --- Network Receive Logic ---
    local data, ip, port
    if udp then
        -- Try to receive data. Returns nil if no data (due to settimeout(0))
        data, ip, port = udp:receivefrom()
    end
        
    -- Check if we actually received a packet
    if data then
        -- === PACKET RECEIVED! ===
        isConnected = true
        pingTimer = 0 -- Stop the ping timer
        
        if Connection_Status:find("lost") or Connection_Status:find("Pinging") or Connection_Status:find("Ready") then
             Connection_Status = "Connected to ESP32 @ " .. ip .. ":" .. port
        end
        debugInfo.lastPacketTime = love.timer.getTime()

        -- --- PARSE THE PACKET STRING ---
        -- Note: The first "PING" reply from the ESP32 ("Connection OK...")
        -- will fail to parse here

        local speedStr = string.match(data, "SPEED=([%d%.]+)")
        local fuelStr = string.match(data, "FUEL=([%d%.]+)")
        local gearStr = string.match(data, "GEAR=([^,]+)")
        local odoStr = string.match(data, "ODOMETER=([%d%.]+)")
        local blinkStr = string.match(data, "BLINK=([^,]+)")
        local beamStr = string.match(data, "HI_BEAM=([^,]+)")
        local engineStr = string.match(data, "CHK_ENG=([^,]+)")
        local oilStr = string.match(data, "OIL=([^,]+)")
        local battStr = string.match(data, "BATT=([^,]+)")
        
        -- Update dashboardData with new values
        -- Only update if the string was found (not nil)
        dashboardData.speed = tonumber(speedStr) or dashboardData.speed
        dashboardData.fuel = tonumber(fuelStr) or dashboardData.fuel
        dashboardData.gear = gearStr or dashboardData.gear
        dashboardData.odometer = tonumber(odoStr) or dashboardData.odometer
        
        if blinkStr then
            dashboardData.indicatorLeft = (blinkStr == "LEFT")
            dashboardData.indicatorRight = (blinkStr == "RIGHT")
        else
            -- Only parse if speed is 0. Prevents flicker when packet is "Connection OK"
            if (tonumber(speedStr) or dashboardData.speed) == 0 then
                dashboardData.indicatorLeft = false
                dashboardData.indicatorRight = false
            end
        end
        
        dashboardData.highBeams = parseBool(beamStr)
        dashboardData.checkEngine = parseBool(engineStr)
        dashboardData.oilWarning = parseBool(oilStr)
        dashboardData.batteryWarning = parseBool(battStr)

        -- Log this packet to the console
        if logTimer >= logInterval then
            logTimer = logTimer - logInterval
            local packetString = string.format("[%.2f] RECV: %s", debugInfo.lastPacketTime, data)
            table.insert(consoleMessages, 1, packetString)
            if #consoleMessages > maxLogEntries then
                table.remove(consoleMessages, maxLogEntries + 1)
            end
        end
        
    end -- end of "if data then"
    
    
    -- --- CHECK FOR DISCONNECTION ---
    -- If it's been > 2 seconds since the last packet, assume connection is lost.
    if love.timer.getTime() - debugInfo.lastPacketTime > 2.0 and debugInfo.lastPacketTime > 0 then
        Connection_Status = "Connection lost. Reconnecting..."
        debugInfo.lastPacketTime = 0 -- Reset timer to prevent this from firing repeatedly
        
        isConnected = false
        
        -- Reset all data to a safe "off" state
        dashboardData.speed = 0
        dashboardData.fuel = 0 -- Set to 0, or could leave it at last known value
        dashboardData.gear = "N"
        dashboardData.indicatorLeft = false
        dashboardData.indicatorRight = false
        dashboardData.highBeams = false
        dashboardData.checkEngine = false
        dashboardData.batteryWarning = false
        dashboardData.oilWarning = false
    end
    

    -- --- ANIMATION (Not simulation) ---
    if blinkerTimer > 0.4 then
        blinkerTimer = 0
        blinkerOn = not blinkerOn
    end
end

--[[
    Called repeatedly on every frame after love.update().
    Handles all drawing operations to the screen.
--]]
function love.draw()
    -- Dashboard Display Area Configuration
    local dashboardAreaX, dashboardAreaY = 10, 10
    local dashboardAreaW, dashboardAreaH = 1050, 650
    
    -- Draw the pre-rendered dashboard frame image
    love.graphics.setColor(1, 1, 1)
    love.graphics.draw(dashboardFrameImage, dashboardAreaX, dashboardAreaY, 0, dashboardAreaW / dashboardFrameImage:getWidth(), dashboardAreaH / dashboardFrameImage:getHeight())

-- --- 1. Speedometer (Central) ---
    local speedometerX, speedometerY = dashboardAreaX + dashboardAreaW / 2, dashboardAreaY + dashboardAreaH / 2 + 45
    local speedometerRadius = 170
    local minAngle = -3.77 -- 0 KPH
    local maxAngle = 0.63  -- 220 KPH
    
    love.graphics.draw(speedometerFaceImage, speedometerX - speedometerFaceImage:getWidth()/2, speedometerY - speedometerFaceImage:getHeight()/2 + 10)

    love.graphics.setFont(mediumFont)
    love.graphics.setColor(1, 1, 1)
    love.graphics.printf(math.floor(dashboardData.speed), speedometerX - 150, speedometerY + 110, 300, "center")
    
    love.graphics.setFont(smallFont)
    love.graphics.setColor(0.7, 0.7, 0.7)
    love.graphics.printf("KPH", speedometerX - 150, speedometerY + 140, 300, "center")
    
-- --- 2. Fuel Gauge (Right Side) ---
    local fuelX, fuelY = dashboardAreaX-70 + dashboardAreaW - 120, speedometerY - 80
    local fuelWidth, fuelHeight = 25, 180
    local fuelRadius = 8
    
    love.graphics.setColor(0.05, 0.05, 0.08, 0.8)
    love.graphics.rectangle("fill", fuelX, fuelY, fuelWidth, fuelHeight, fuelRadius)
    
    local currentFuelHeight = map(dashboardData.fuel, 0, 100, 0, fuelHeight)
    if dashboardData.fuel < 15 then
        love.graphics.setColor(1, 0.1, 0.1) -- Red when low
    else
        love.graphics.setColor(0.2, 0.8, 0.2) -- Green
    end
    love.graphics.rectangle("fill", fuelX, fuelY + (fuelHeight - currentFuelHeight), fuelWidth, currentFuelHeight, fuelRadius)
    
    love.graphics.setFont(smallFont)
    love.graphics.setColor(0.9, 0.9, 0.9)
    love.graphics.printf("F", fuelX + fuelWidth + 10, fuelY - 5, 20, "left")
    love.graphics.printf("E", fuelX + fuelWidth + 10, fuelY + fuelHeight - 15, 20, "left")
    
-- --- 3. Gear Indicator (Left Side) ---
    local gearX, gearY = dashboardAreaX + 270, speedometerY+20
    
    love.graphics.setColor(0.05, 0.05, 0.08, 0.8)
    love.graphics.rectangle("fill", gearX - 40, gearY - 50, 80, 100, 10)
    
    love.graphics.setFont(largeFont)
    love.graphics.setColor(0.2, 0.8, 0.2) -- Green
    love.graphics.printf(dashboardData.gear, gearX - 40, gearY - 30, 80, "center")
    
    love.graphics.setFont(smallFont)
    love.graphics.setColor(0.7, 0.7, 0.7)
    love.graphics.printf("GEAR", gearX - 40, gearY + 20, 80, "center")

-- --- 4. Odometer (Bottom of Speedometer) ---
    local odoX, odoY = speedometerX+245, speedometerY + speedometerRadius - 100
    
    love.graphics.setColor(0.05, 0.05, 0.08, 0.8)
    love.graphics.rectangle("fill", odoX - 80, odoY - 10, 160, 40, 20)

    love.graphics.setFont(mediumFont)
    love.graphics.setColor(0.8, 0.8, 0.9)
    love.graphics.printf(string.format("%.1f KM", dashboardData.odometer), odoX - 80, odoY - 5, 160, "center") 

-- --- 5. Indicator Lights (Top Row) ---
    local topIndY = dashboardAreaY + 280 
    local indColorOff = {0.1, 0.3, 0.1, 0.5}
    local indColorOn = {0, 1, 0, 1}
    local warnColorOff = {0.3, 0.2, 0.1, 0.5}
    local warnColorOn = {1, 0.5, 0, 1}
    
    -- Left Turn Indicator
    local leftIndColor = (dashboardData.indicatorLeft and blinkerOn) and indColorOn or indColorOff
    drawArrowIcon(dashboardAreaX + 250, topIndY, 20, "left", leftIndColor)
    
    -- Right Turn Indicator
    local rightIndColor = (dashboardData.indicatorRight and blinkerOn) and indColorOn or indColorOff
    drawArrowIcon(dashboardAreaX + dashboardAreaW - 250, topIndY, 20, "right", rightIndColor)
    
    -- High Beams
    local highBeamColor = dashboardData.highBeams and {0.2, 0.5, 1, 1} or {0.1, 0.2, 0.3, 0.5}
    drawHighBeamIcon(speedometerX+250, topIndY+60, highBeamColor)
    
-- --- 6. Warning Lights (Bottom Row) ---
    local bottomWarnY = dashboardAreaY + dashboardAreaH - 60 
    
    -- Check Engine
    local checkEngineColor = dashboardData.checkEngine and warnColorOn or warnColorOff
    drawCheckEngineIcon(speedometerX - 360, bottomWarnY-250, checkEngineColor)

    -- Battery Warning
    local batteryColor = dashboardData.batteryWarning and {1, 0.8, 0, 1} or warnColorOff
    drawBatteryIcon(speedometerX - 360, bottomWarnY-200, batteryColor)

    -- Oil Pressure Warning
    local oilColor = dashboardData.oilWarning and {1, 0.1, 0.1, 1} or warnColorOff
    drawOilPressureIcon(speedometerX-360, bottomWarnY-150, oilColor)


-- --- Draw Needle (Last, so it's on top) ---
    local needleAngle = map(dashboardData.speed, 0, 220, minAngle, maxAngle)
    local hubRadius = 15
    local needleLength = speedometerRadius - 40
    
    local hubX = speedometerX - 0.8
    local hubY = speedometerY + 31
    local startX = hubX + math.cos(needleAngle) * hubRadius
    local startY = hubY + math.sin(needleAngle) * hubRadius
    local endX = hubX + math.cos(needleAngle) * needleLength
    local endY = hubY + math.sin(needleAngle) * needleLength

    love.graphics.setColor(1, 0.1, 0.1, 0.9) -- Bright Red needle
    love.graphics.setLineWidth(6)
    love.graphics.line(startX, startY, endX, endY)
    
    love.graphics.setColor(0, 0, 0, 0.4)
    love.graphics.line(startX + 2, startY + 2, endX + 2, endY + 2)
    love.graphics.setLineWidth(1)
    
    -- Draw 3D needle hub
    love.graphics.setColor(0.2, 0.2, 0.25)
    love.graphics.circle("fill", hubX, hubY, 15)
    love.graphics.setColor(0.8, 0.8, 0.8)
    love.graphics.circle("fill", hubX, hubY, 12)
    love.graphics.setColor(0.6, 0.6, 0.6)
    love.graphics.circle("fill", hubX, hubY, 8)
    love.graphics.setColor(0.1, 0.1, 0.1)
    love.graphics.circle("fill", hubX, hubY, 3)
    
-- --- Output Console ---
    local Stats_x = 10
    local Stats_y = 670
    local Stats_width = 1050
    local Stats_height = 170

    love.graphics.setColor(0, 0.4, 0.4)
    love.graphics.rectangle("fill", Stats_x, Stats_y, Stats_width, Stats_height)

    love.graphics.setFont(smallFont)
    love.graphics.setColor(1, 1, 1)
    love.graphics.printf("Output Console > ", Stats_x + 10, Stats_y + 8, 500, "left")

    love.graphics.setColor(1, 0.8, 0.2)
    love.graphics.printf(Connection_Status, Stats_x + 170, Stats_y + 8, Stats_width - 410, "left")
    
    love.graphics.setFont(smallFont)
    love.graphics.setColor(0.7, 0.7, 1)
    local lastPacketString
    if debugInfo.lastPacketTime == 0 then
        if Connection_Status:find("Pinging") or Connection_Status:find("Ready") then
            lastPacketString = "Last Packet: Waiting..."
        else
            lastPacketString = "Last Packet: N/A"
        end
    else
        lastPacketString = string.format("Last Packet: %.2fs ago", love.timer.getTime() - debugInfo.lastPacketTime)
    end
    love.graphics.printf(lastPacketString, Stats_x + Stats_width - 210, Stats_y + 8, 200, "right")

    -- Draw console log messages
    love.graphics.setFont(tinyFont)
    love.graphics.setColor(0.8, 1, 0.8)
    
    local logStartY = Stats_y + 40
    local lineHeight = 18

    for i, msg in ipairs(consoleMessages) do
        local currentY = logStartY + (i - 1) * lineHeight
        if currentY < (Stats_y + Stats_height - 10) then
            love.graphics.printf(msg, Stats_x + 10, currentY, Stats_width - 20, "left")
        end
    end
end

--[[
    Called whenever a key on the keyboard is pressed.
--]]
function love.keypressed(key)
   if key == "escape" then
      love.event.quit()
   end

end
