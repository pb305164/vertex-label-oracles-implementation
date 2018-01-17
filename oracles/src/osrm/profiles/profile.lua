-- Car profile

api_version = 2

Set = require('lib/set')
Sequence = require('lib/sequence')
Handlers = require("lib/way_handlers")
find_access_tag = require("lib/access").find_access_tag
limit = require("lib/maxspeed").limit

function setup()
  local use_left_hand_driving = false
  return {
    properties = {
      max_speed_for_map_matching      = 250/3.6, -- 180kmph -> m/s
      left_hand_driving               = use_left_hand_driving,
      -- For routing based on duration, but weighted for preferring certain roads
      -- weight_name                     = 'routability',
      -- For shortest duration without penalties for accessibility
      weight_name                     = 'duration',
      -- For shortest distance without penalties for accessibility
      -- weight_name                     = 'distance',
      process_call_tagless_node      = false,
      u_turn_penalty                 = 0,
      continue_straight_at_waypoint  = false,
      use_turn_restrictions          = false,
      traffic_light_penalty          = 0,
    },

    default_mode              = mode.driving,
    default_speed             = 5,
    oneway_handling           = false,
    side_road_multiplier      = 1,
    turn_penalty              = 0,
    speed_reduction           = 0,

    -- Note: this biases right-side driving.
    -- Should be inverted for left-driving countries.
    -- turn_bias   = use_left_hand_driving and 1/1.075 or 1.075,

    -- a list of suffixes to suppress in name change instructions
    suffix_list = {
      'N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'North', 'South', 'West', 'East'
    },

    barrier_whitelist = Set {
      'cattle_grid',
      'border_control',
      'toll_booth',
      'sally_port',
      'gate',
      'lift_gate',
      'no',
      'entrance'
    },

    access_tag_whitelist = Set {
      'yes',
      'motorcar',
      'motor_vehicle',
      'vehicle',
      'permissive',
      'designated',
      'hov'
    },

    access_tag_blacklist = Set {
      -- 'no',
      -- 'agricultural',
      -- 'forestry',
      -- 'emergency',
      -- 'psv',
      -- 'customers',
      -- 'private',
      -- 'delivery',
      -- 'destination'
    },

    restricted_access_tag_list = Set {
      -- 'private',
      -- 'delivery',
      -- 'destination',
      -- 'customers',
    },

    access_tags_hierarchy = Sequence {
    },

    service_tag_forbidden = Set {
      -- 'emergency_access'
    },

    restrictions = Sequence {
    },

    classes = Sequence {
        'toll', 'motorway', 'ferry', 'restricted'
    },

    -- classes to support for exclude flags
    excludable = Sequence {
    },

    avoid = Set {
    },

    speeds = Sequence {
      highway = {
        motorway        = 90,
        motorway_link   = 45,
        trunk           = 85,
        trunk_link      = 40,
        primary         = 65,
        primary_link    = 30,
        secondary       = 55,
        secondary_link  = 25,
        tertiary        = 40,
        tertiary_link   = 20,
        unclassified    = 25,
        residential     = 25,
        living_street   = 10,
        service         = 15
      }
    },

    service_penalties = {
      alley             = 0.5,
      parking           = 0.5,
      parking_aisle     = 0.5,
      driveway          = 0.5,
      ["drive-through"] = 0.5,
      ["drive-thru"] = 0.5
    },

    restricted_highway_whitelist = Set {
      'motorway',
      'motorway_link',
      'trunk',
      'trunk_link',
      'primary',
      'primary_link',
      'secondary',
      'secondary_link',
      'tertiary',
      'tertiary_link',
      'residential',
      'living_street',
    },

    construction_whitelist = Set {
      'no',
      'widening',
      'minor',
    },

    route_speeds = {
      ferry = 5,
      shuttle_train = 10
    },

    bridge_speeds = {
      movable = 5
    },

    -- surface/trackype/smoothness
    -- values were estimated from looking at the photos at the relevant wiki pages

    -- max speed for surfaces
    surface_speeds = {
      asphalt = nil,    -- nil mean no limit. removing the line has the same effect
      concrete = nil,
      ["concrete:plates"] = nil,
      ["concrete:lanes"] = nil,
      paved = nil,

      cement = 80,
      compacted = 80,
      fine_gravel = 80,

      paving_stones = 60,
      metal = 60,
      bricks = 60,

      grass = 40,
      wood = 40,
      sett = 40,
      grass_paver = 40,
      gravel = 40,
      unpaved = 40,
      ground = 40,
      dirt = 40,
      pebblestone = 40,
      tartan = 40,

      cobblestone = 30,
      clay = 30,

      earth = 20,
      stone = 20,
      rocky = 20,
      sand = 20,

      mud = 10
    },

    -- max speed for tracktypes
    tracktype_speeds = {
      grade1 =  60,
      grade2 =  40,
      grade3 =  30,
      grade4 =  25,
      grade5 =  20
    },

    -- max speed for smoothnesses
    smoothness_speeds = {
      intermediate    =  80,
      bad             =  40,
      very_bad        =  20,
      horrible        =  10,
      very_horrible   =  5,
      impassable      =  0
    },

    -- http://wiki.openstreetmap.org/wiki/Speed_limits
    maxspeed_table_default = {
      urban = 50,
      rural = 90,
      trunk = 110,
      motorway = 130
    },

    -- List only exceptions
    maxspeed_table = {
      ["ch:rural"] = 80,
      ["ch:trunk"] = 100,
      ["ch:motorway"] = 120,
      ["de:living_street"] = 7,
      ["dk:rural"] = 80,
      ["ru:living_street"] = 20,
      ["ru:urban"] = 60,
      ["ua:urban"] = 60,
      ["at:rural"] = 100,
      ["de:rural"] = 100,
      ["at:trunk"] = 100,
      ["cz:trunk"] = 0,
      ["ro:trunk"] = 100,
      ["cz:motorway"] = 0,
      ["de:motorway"] = 0,
      ["ru:motorway"] = 110,
      ["gb:nsl_single"] = (60*1609)/1000,
      ["gb:nsl_dual"] = (70*1609)/1000,
      ["gb:motorway"] = (70*1609)/1000,
      ["uk:nsl_single"] = (60*1609)/1000,
      ["uk:nsl_dual"] = (70*1609)/1000,
      ["uk:motorway"] = (70*1609)/1000,
      ["nl:rural"] = 80,
      ["nl:trunk"] = 100,
      ["none"] = 140
    }
  }
end

function process_node(profile, node, result)

end

function process_way(profile, way, result)
local highway = way:get_value_by_key("highway")
  local toll = way:get_value_by_key("toll")
  local name = way:get_value_by_key("name")
  local oneway = way:get_value_by_key("oneway")
  local route = way:get_value_by_key("route")
  local duration = way:get_value_by_key("duration")
  local maxspeed = tonumber(way:get_value_by_key ( "maxspeed"))
  local maxspeed_forward = tonumber(way:get_value_by_key( "maxspeed:forward"))
  local maxspeed_backward = tonumber(way:get_value_by_key( "maxspeed:backward"))
  local junction = way:get_value_by_key("junction")

  if name then
    result.name = name
  end

  result.forward_mode = mode.driving
  result.backward_mode = mode.driving


  local speed_forw = profile.speeds[highway] or profile.default_speed
  local speed_back = speed_forw

  if highway == "river" then
    local temp_speed = speed_forw
    result.forward_mode = mode.river_down
    result.backward_mode = mode.river_up
    speed_forw = temp_speed*1.5
    speed_back = temp_speed/1.5
  elseif highway == "steps" then
    result.forward_mode = mode.steps_down
    result.backward_mode = mode.steps_up
  end

  if maxspeed_forward ~= nil and maxspeed_forward > 0 then
    speed_forw = maxspeed_forward
  else
    if maxspeed ~= nil and maxspeed > 0 and speed_forw > maxspeed then
      speed_forw = maxspeed
    end
  end

  if maxspeed_backward ~= nil and maxspeed_backward > 0 then
    speed_back = maxspeed_backward
  else
    if maxspeed ~=nil and maxspeed > 0 and speed_back > maxspeed then
      speed_back = maxspeed
    end
  end

  result.forward_speed = speed_forw
  result.backward_speed = speed_back
end

function process_turn(profile, turn)

end

return {
  setup = setup,
  process_way = process_way,
  process_node = process_node,
  process_turn = process_turn
}
