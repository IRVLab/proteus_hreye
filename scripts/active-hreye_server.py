#!/usr/bin/python3

import rospy
from rosnode import get_node_names
from rospy import service

import xml.etree.ElementTree as ET
from proteus.luceme import Luceme, LNode, LNodeStatic, LNodeBlink, LNodePulse, LNodeFill
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity

rospy.init_node('active_hreye_server', argv=None, anonymous=True)

hreye_params = None
symbols = None
lucemes = None

# Farms out the execution of the luceme to the appropriate function
def service_cb(req, luceme):
    rospy.logdebug('Service callback for luceme %s'%(luceme.id))
    if luceme.call_type == 'trigger':
        return execute_trigger(req, luceme)
    elif luceme.call_type == 'directional':
        return execute_directional(req, luceme)
    elif luceme.call_type == 'target':
        return execute_target(req, luceme)
    elif luceme.call_type == 'quantity':
        return execute_quantity(req, luceme)
    else:
        return False

if __name__ == '__main__':
    rospy.loginfo('Initializing the HREye server')

    hreye_params = rospy.get_param('vectors/out/ActiveHREye/')

    #Check if PROTEUS language server is up
    rospy.loginfo('Checking PROTEUS language server...')
    lang_server_active = False
    nodes = get_node_names()
    rospy.logdebug(nodes)
    for n in nodes:
        if n.split('/')[-1] == 'proteus_language_server':
            lang_server_active = True
            break
    if not lang_server_active:
        rospy.logerr("This HREye implementation requires the PROTEUS language server to be active.")
        sys.exit(1)
    else:
        rospy.loginfo('PROTEUS language server OK!')


     # Find luceme language definition file
    rospy.loginfo("Loading RCVM vector information...")
    hreye_info = rospy.get_param('vectors/out/ActiveHREye')
    hreye_def_file = hreye_info['definition_file']

    # Find symbol definitions
    rospy.loginfo("Loading symbol information...")
    symbols = rospy.get_param('symbols/out')
    
    # Process luceme definition file into luceme objects
    rospy.loginfo("Loading luceme definitions from luceme definition file.")
    lucemes = dict()

    #Load XML file
    tree = ET.parse(hreye_def_file)
    root = tree.getroot()


    for ldef in root:
        l = Luceme()
        l.parse_from_xml(ldef)
        lucemes[l.id] = l

    # Check for symbol matchup.
    for s in symbols:
        for key in lucemes:
            k = lucemes[key]
            if s == l.id:
                rospy.loginfo("Found match beteween symbol %s and luceme %s, associating data."%(s, k.id))
                rospy.logdebug("Call type: %s"%(symbols.get(s).get('call_type')))
                l.set_call_type(symbols.get(s).get('call_type'))
                break

    print(lucemes)
    
    # Setup service calls
    for key, luceme in lucemes.items():
        service_class = None
        if luceme.call_type == 'trigger':
            service_class = SymbolTrigger
        elif luceme.call_type == 'directional':
            service_class = SymbolDirectional
        elif luceme.call_type == 'target':
            service_class = SymbolTarget
        elif luceme.call_type == 'quantity':
            service_class = SymbolQuantity
        else:
            rospy.logwarn("Unexpected luceme call type %s"%(luceme.call_type))

        service_name = 'hreye/active/'+ luceme.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for luceme %s at service endpoint: %s'%(luceme.id, service_name))
        rospy.Service(service_name, service_class, lambda req, luceme=luceme: service_cb(req, luceme))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass