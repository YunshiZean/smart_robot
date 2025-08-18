import osmnx as ox
from osmnx import projection
import networkx as nx

import sys
sys.path.append('..')

from pyadroute.ad_route_adaptor import ADRouteAdaptor


osm_filename = r"/home/ros/ros_ws/smart_robot_ws/src/autonomous_driving/map/way2.osm"

route_adaptor = ADRouteAdaptor(osm_filename)
print(route_adaptor.G)
print(route_adaptor.G.graph)
