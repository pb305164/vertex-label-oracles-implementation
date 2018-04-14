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
      max_speed_for_map_matching      = 200/3.6, -- 200kmph -> m/s
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

    construction_whitelist = Set {
      'no',
      'widening',
      'minor',
    },

    -- surface/trackype/smoothness
    -- values were estimated from looking at the photos at the relevant wiki pages

  }
end

function process_node(profile, node, result)

end

function process_way(profile, way, result)
local highway = way:get_value_by_key("highway")
  local toll = way:get_value_by_key("toll")
  local maxspeed = tonumber(way:get_value_by_key ( "maxspeed"))

  result.forward_mode = mode.driving
  result.backward_mode = mode.driving


  speed_forw = maxspeed

  speed_back = maxspeed

  result.forward_speed = speed_forw
  result.backward_speed = speed_back
end

function process_turn(profile, turn)
  turn.duration = 0.
end

return {
  setup = setup,
  process_way = process_way,
  process_node = process_node,
  process_turn = process_turn
}
