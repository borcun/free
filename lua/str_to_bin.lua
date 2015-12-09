-- @brief function that converts decimal number string to binary number string
-- @param str - decimal number string
-- @return binary number string

function toBinary( str )
  local num = tonumber( str )
  local ret = ""
  
  while num ~= 0 do
    ret = tostring( num % 2 ) .. ret
    -- ceil to convert float to decimal
    num = math.ceil( num / 2 - 0.5 )
  end

  return ret    
end

local x = "921"
local y = toBinary( x )

print( "y = " .. y )