#!/usr/bin/python3

from operator import length_hint
import rospy
from rosnode import get_node_names
from tf.transformations import euler_from_quaternion
import sys
from time import sleep

import xml.etree.ElementTree as ET
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity
from proteus.color import Color
from proteus.luceme import Luceme, LNode, LNodeStatic, LNodeBlink, LNodePulse, LNodeFill
from proteus.hreye import HREyeConfig

from std_msgs.msg import Header, ColorRGBA
from proteus_hreye.msg import HREyeState

rospy.init_node('active_hreye_server', argv=None, anonymous=True)

hreye_params = None
hreye_config = None
symbols = None
lucemes = None

state_publisher = None
state_queue = []

# Farms out the execution of the luceme to the appropriate function
def service_cb(req, luceme):
    global state_queue, hreye_config
    rospy.logdebug('Service callback for luceme %s'%(luceme.id))
    if luceme.call_type == 'trigger':
        return execute_trigger(req, luceme, state_queue, hreye_config)
    elif luceme.call_type == 'directional':
        return execute_directional(req, luceme, state_queue, hreye_config)
    elif luceme.call_type == 'target':
        return execute_target(req, luceme, state_queue, hreye_config)
    elif luceme.call_type == 'quantity':
        return execute_quantity(req, luceme, state_queue, hreye_config)
    else:
        return False

def execute_trigger(req, luceme, state_queue, hreye_config):
    step_durations = luceme.get_luceme_duration()
    states_per_second = hreye_config.rate
    # state_vector = [ColorRGBA()] * 40

    # This sets up a list of lists. Each list is the states for that step of the luceme.
    stepwise_states = [[] for x in range(len(step_durations))]
    total_duration = 0
    for step, duration in enumerate(step_durations):
        total_duration += duration
        stepwise_states[step] = [[ColorRGBA() for x in range(40)] for x in range(int(states_per_second * duration))]

    for l in luceme.lnodes:
        step = l.step # Relevent step for everything we're going to do.
        duration = l.get_duration_seconds()
        indexes = hreye_config.sectors[l.sector_id].get_indexes() # These are the relevent indexes for everything we're going to be doing.

        if type(l) is LNodeStatic:
            n_states = int(duration * states_per_second)
            ill = list(l.illuminations.items())[0][1]
            color_values = hreye_config.colors[ill.color_id]

            for state in stepwise_states[step][0:n_states + 1]:    
                for i in indexes:
                    state[i].r = color_values.r
                    state[i].g = color_values.g
                    state[i].b = color_values.b
                    state[i].a = ill.brightness

        elif type(l) is LNodeBlink:
            on_ill = l.illuminations['on_state']
            off_ill = l.illuminations['off_state']
            on_color = hreye_config.colors[on_ill.color_id]

            if off_ill.color_id == "none":
                off_color = Color()
                off_color.r = 0
                off_color.g = 0
                off_color.b = 0
            else:
                off_color = hreye_config.colors[off_ill.color_id]

            flash_on = True

            for iter in range(l.blink.iterations):
                # Make sure that we consider the the iterations in our calculation of states.
                n_states = int((duration * states_per_second)/l.blink.iterations)
                start_state = iter * n_states

                if flash_on:
                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = on_color.r
                            state[i].g = on_color.g
                            state[i].b = on_color.b
                            state[i].a = on_ill.brightness
                    flash_on = False
                        
                    continue
                else:
                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = off_color.r
                            state[i].g = off_color.g
                            state[i].b = off_color.b
                            state[i].a = off_ill.brightness
                    flash_on = True
                    continue
                
        elif type(l) is LNodePulse:
            low_ill = l.illuminations['lo_state']
            high_ill = l.illuminations['hi_state']
            pulse = l.pulse

            if l.pulse.vary == "alpha":
                low_a = low_ill.brightness
                current_a = low_a
                high_a = high_ill.brightness
                static_color = hreye_config.colors[low_ill.color_id]
                
                for cycle in range(l.pulse.cycles):
                    n_states = int((duration * states_per_second)/l.pulse.cycles)
                    start_state = cycle * n_states

                    difference = float(high_a - low_a)
                    step_size = float(difference * 2.0)/float(n_states)
                    going_up = True

                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            # print(current_a)
                            state[i].r = static_color.r
                            state[i].g = static_color.g
                            state[i].b = static_color.b
                            state[i].a = current_a

                        if going_up:
                            current_a += step_size
                        else:
                            current_a -= step_size

                        if going_up and current_a >= high_a:
                            going_up = False
                        elif not going_up and current_a <= low_a:
                            going_up = True
            
            elif l.pulse.vary == "color":
                static_a = low_ill.brightness
                low_color = eval(low_ill.color_id) #Note that in this case, it's not an actual ID, it's a color string.
                high_color =eval(high_ill.color_id)

                current_color = list(low_color)

                for cycle in range(l.pulse.cycles):
                    n_states = int((duration * states_per_second)/l.pulse.cycles)
                    start_state = cycle * n_states

                    differences = (high_color[0]-low_color[0], high_color[1]-low_color[1], high_color[2]-low_color[2])
                    step_sizes = ((differences[0] * 2.0)/float(n_states), (differences[1] * 2.0)/float(n_states), (differences[2] * 2.0)/float(n_states))
                    going_up = True

                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = current_color[0]
                            state[i].g = current_color[1]
                            state[i].b = current_color[2]
                            state[i].a = static_a

                        if going_up:
                            current_color[0] += step_sizes[0]
                            current_color[1] += step_sizes[1]
                            current_color[2] += step_sizes[2]
                        else:
                            current_color[0] -= step_sizes[0]
                            current_color[1] -= step_sizes[1]
                            current_color[2] -= step_sizes[2]

                        if going_up and current_color[0] >= high_color[0] and current_color[1] >= high_color[1] and current_color[2] >= high_color[2]:
                            going_up = False
                        elif not going_up and current_color[0] <= low_color[0] and current_color[1] <= low_color[1] and current_color[2] <= low_color[2]:
                            going_up = True

                pass

            else:
                raise NotImplementedError("This type of pulse LNodes({}) have not yet been implemented.".format(pulse.vary))
            
        elif type(l) is LNodeFill:
            #This is gonna get messy. 
            ill = l.illuminations['fill']
            color_values = hreye_config.colors[ill.color_id]
            n_states = int(duration * states_per_second)

            if l.fill.type == 'segment-move':
                #TODO Modify this so that we aren't assuming the sector is one segment only.
                sector_seg = hreye_config.sectors[l.sector_id].segments[0]
                start_index = sector_seg.resolve_index_formula(l.fill.start)

                fill_start = start_index
                max_index = indexes[-1]
                min_index = indexes[0]
                print(fill_start, l.fill.length, max_index)
                
                for state in stepwise_states[step][0:n_states + 1]:

                    diff = max_index-fill_start

                    #FIXME Doesn't work for counterclockwise.
                    
                    if diff > l.fill.length:
                        segment_indexes = [k for k in range(fill_start, fill_start + l.fill.length + 1)]    

                    else:
                        segment_indexes = [k for k in range(fill_start, max_index+1)]
                        segment_indexes.extend([k  for k in range(0, (l.fill.length - (max_index-fill_start) + 1))])

                    for i in segment_indexes:
                        state[i].r = color_values.r
                        state[i].g = color_values.g
                        state[i].b = color_values.b
                        state[i].a = ill.brightness
                
                                    
                    if l.fill.direction == 'clockwise':
                        if fill_start < max_index:
                            fill_start += 1
                        else:
                            fill_start = 0
                    elif l.fill.direction == 'counter-clockwise':
                        if fill_start < min_index:
                            fill_start -= 1
                        else:
                            fill_start = max_index
            else:
                raise NotImplementedError("Fill type {} not implemented.".format(l.fill.type))

        else:
            raise NotImplementedError("That type of LNode ({}) not implemented for Trigger lucemes.".format(type(l)))

    luceme_state = [state for statelist in stepwise_states for state in statelist]
    state_queue.extend(luceme_state)

    # With the states dispatched to the queue, block until the luceme is completed.
    sleep(total_duration)

    return True

def execute_directional(req, luceme, state_queue, hreye_config):
    euler_angles = euler_from_quaternion([req.transform.rotation.x, req.transform.rotation.y, req.transform.rotation.z, req.transform.rotation.w])

    step_durations = luceme.get_luceme_duration()
    states_per_second = hreye_config.rate
    # state_vector = [ColorRGBA()] * 40

    # This sets up a list of lists. Each list is the states for that step of the luceme.
    stepwise_states = [[] for x in range(len(step_durations))]
    total_duration = 0
    for step, duration in enumerate(step_durations):
        total_duration += duration
        stepwise_states[step] = [[ColorRGBA() for x in range(40)] for x in range(int(states_per_second * duration))]

    for l in luceme.lnodes:
        step = l.step # Relevent step for everything we're going to do.
        duration = l.get_duration_seconds()
        indexes = hreye_config.sectors[l.sector_id].get_indexes(euler_angles) # These are the relevent indexes for everything we're going to be doing.

        if type(l) is LNodeStatic:
            n_states = int(duration * states_per_second)
            ill = list(l.illuminations.items())[0][1]
            color_values = hreye_config.colors[ill.color_id]

            for state in stepwise_states[step][0:n_states + 1]:    
                for i in indexes:
                    state[i].r = color_values.r
                    state[i].g = color_values.g
                    state[i].b = color_values.b
                    state[i].a = ill.brightness

        elif type(l) is LNodeBlink:
            on_ill = l.illuminations['on_state']
            off_ill = l.illuminations['off_state']
            on_color = hreye_config.colors[on_ill.color_id]

            if off_ill.color_id == "none":
                off_color = Color()
                off_color.r = 0
                off_color.g = 0
                off_color.b = 0
            else:
                off_color = hreye_config.colors[off_ill.color_id]

            flash_on = True

            for iter in range(l.blink.iterations):
                # Make sure that we consider the the iterations in our calculation of states.
                n_states = int((duration * states_per_second)/l.blink.iterations)
                start_state = iter * n_states

                if flash_on:
                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = on_color.r
                            state[i].g = on_color.g
                            state[i].b = on_color.b
                            state[i].a = on_ill.brightness
                    flash_on = False
                        
                    continue
                else:
                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = off_color.r
                            state[i].g = off_color.g
                            state[i].b = off_color.b
                            state[i].a = off_ill.brightness
                    flash_on = True
                    continue
    
        elif type(l) is LNodePulse:
            low_ill = l.illuminations['lo_state']
            high_ill = l.illuminations['hi_state']
            pulse = l.pulse

            if l.pulse.vary == "alpha":
                low_a = low_ill.brightness
                current_a = low_a
                high_a = high_ill.brightness
                static_color = hreye_config.colors[low_ill.color_id]
                
                for cycle in range(l.pulse.cycles):
                    n_states = int((duration * states_per_second)/l.pulse.cycles)
                    start_state = cycle * n_states

                    difference = float(high_a - low_a)
                    step_size = float(difference * 2.0)/float(n_states)
                    going_up = True

                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            # print(current_a)
                            state[i].r = static_color.r
                            state[i].g = static_color.g
                            state[i].b = static_color.b
                            state[i].a = current_a

                        if going_up:
                            current_a += step_size
                        else:
                            current_a -= step_size

                        if going_up and current_a >= high_a:
                            going_up = False
                        elif not going_up and current_a <= low_a:
                            going_up = True
            
            elif l.pulse.vary == "color":
                static_a = low_ill.brightness
                low_color = eval(low_ill.color_id) #Note that in this case, it's not an actual ID, it's a color string.
                high_color =eval(high_ill.color_id)

                current_color = list(low_color)

                for cycle in range(l.pulse.cycles):
                    n_states = int((duration * states_per_second)/l.pulse.cycles)
                    start_state = cycle * n_states

                    differences = (high_color[0]-low_color[0], high_color[1]-low_color[1], high_color[2]-low_color[2])
                    step_sizes = ((differences[0] * 2.0)/float(n_states), (differences[1] * 2.0)/float(n_states), (differences[2] * 2.0)/float(n_states))
                    going_up = True

                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = current_color[0]
                            state[i].g = current_color[1]
                            state[i].b = current_color[2]
                            state[i].a = static_a

                        if going_up:
                            current_color[0] += step_sizes[0]
                            current_color[1] += step_sizes[1]
                            current_color[2] += step_sizes[2]
                        else:
                            current_color[0] -= step_sizes[0]
                            current_color[1] -= step_sizes[1]
                            current_color[2] -= step_sizes[2]

                        if going_up and current_color[0] >= high_color[0] and current_color[1] >= high_color[1] and current_color[2] >= high_color[2]:
                            going_up = False
                        elif not going_up and current_color[0] <= low_color[0] and current_color[1] <= low_color[1] and current_color[2] <= low_color[2]:
                            going_up = True

                pass

            else:
                raise NotImplementedError("This type of pulse LNodes({}) have not yet been implemented.".format(pulse.vary))
            
        else:
            raise NotImplementedError("That type of LNode ({}) not implemented for Directional lucemes.".format(type(l)))



    luceme_state = [state for statelist in stepwise_states for state in statelist]
    state_queue.extend(luceme_state)

    # With the states dispatched to the queue, block until the luceme is completed.
    sleep(total_duration)

    return True
    

def execute_target(req, luceme, state_queue, hreye_config):
    pass

def execute_quantity(req, luceme, state_queue, hreye_config):
    value = int(req.quantity * 100)

    step_durations = luceme.get_luceme_duration()
    states_per_second = hreye_config.rate
    # state_vector = [ColorRGBA()] * 40

    # This sets up a list of lists. Each list is the states for that step of the luceme.
    stepwise_states = [[] for x in range(len(step_durations))]
    total_duration = 0
    for step, duration in enumerate(step_durations):
        total_duration += duration
        stepwise_states[step] = [[ColorRGBA() for x in range(40)] for x in range(int(states_per_second * duration))]


    for l in luceme.lnodes:
        step = l.step # Relevent step for everything we're going to do.
        duration = l.get_duration_seconds()
        indexes = hreye_config.sectors[l.sector_id].get_indexes() # These are the relevent indexes for everything we're going to be doing.

        if type(l) is LNodeStatic:
            n_states = int(duration * states_per_second)
            ill = list(l.illuminations.items())[0][1]
            color_values = hreye_config.colors[ill.color_id]

            for state in stepwise_states[step][0:n_states + 1]:    
                for i in indexes:
                    state[i].r = color_values.r
                    state[i].g = color_values.g
                    state[i].b = color_values.b
                    state[i].a = ill.brightness

        elif type(l) is LNodeBlink:
            on_ill = l.illuminations['on_state']
            off_ill = l.illuminations['off_state']
            on_color = hreye_config.colors[on_ill.color_id]

            if off_ill.color_id == "none":
                off_color = Color()
                off_color.r = 0
                off_color.g = 0
                off_color.b = 0
            else:
                off_color = hreye_config.colors[off_ill.color_id]

            flash_on = True

            for iter in range(l.blink.iterations):
                # Make sure that we consider the the iterations in our calculation of states.
                n_states = int((duration * states_per_second)/l.blink.iterations)
                start_state = iter * n_states

                if flash_on:
                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = on_color.r
                            state[i].g = on_color.g
                            state[i].b = on_color.b
                            state[i].a = on_ill.brightness
                    flash_on = False
                        
                    continue
                else:
                    for state in stepwise_states[step][start_state: (start_state + n_states) + 1]:
                        for i in indexes:
                            state[i].r = off_color.r
                            state[i].g = off_color.g
                            state[i].b = off_color.b
                            state[i].a = off_ill.brightness
                    flash_on = True
                    continue
    
        elif type(l) is LNodeFill:
            color_map = l.color_map
            range = l.fill.range

            if l.fill.type == 'expand-value':
                
                current_color_vals = hreye_config.colors[l.illuminations[color_map.mapping[0]].color_id]
                current_brightness = l.illuminations[color_map.mapping[0]].brightness

            elif l.fill.type == 'contract-value':
                current_color_vals = hreye_config.colors[l.illuminations[color_map.mapping[value]].color_id]
                current_brightness = l.illuminations[color_map.mapping[0]].brightness

        else:
            raise NotImplementedError("That type of LNode ({}) not implemented for Directional lucemes.".format(type(l)))



    luceme_state = [state for statelist in stepwise_states for state in statelist]
    state_queue.extend(luceme_state)

    # With the states dispatched to the queue, block until the luceme is completed.
    sleep(total_duration)

    return True

    

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

    for item in root:
        if item.tag == 'lucemes':
            for ldef in item:
                l = Luceme()
                l.parse_from_xml(ldef)
                lucemes[l.id] = l
        elif item.tag == 'hreye-config':
            #Special case for parsing the meta information.
            hreye_config = HREyeConfig()
            hreye_config.parse_from_xml(item)
            hreye_config.associate_rings()
            hreye_config.resolve_sector_indexes() # This is necessary to allow for rings to be undefined until after sectors are read in. 
                                                  # Once the entire config is read in, however, we have all the required information.

    print(hreye_config)

    # Check for symbol matchup.
    for s in symbols:
        for key in lucemes:
            l = lucemes[key]
            if s == l.id:
                rospy.loginfo("Found match beteween symbol %s and luceme %s, associating data."%(s, l.id))
                rospy.logdebug("Call type: %s"%(symbols.get(s).get('call_type')))
                l.set_call_type(symbols.get(s).get('call_type'))
                break

    state_publisher = rospy.Publisher("/hreye_state", HREyeState)
    
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
            rospy.logwarn("Unexpected luceme call type {} for luceme {}".format(luceme.call_type, luceme.id))

        service_name = 'hreye/active/'+ luceme.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for luceme %s at service endpoint: %s'%(luceme.id, service_name))
        rospy.Service(service_name, service_class, lambda req, luceme=luceme: service_cb(req, luceme))

    rate = rospy.Rate(hreye_config.rate)
    while not rospy.is_shutdown():
        if len(state_queue) > 0:
            if hreye_config.mode == 'mirror':
                state = state_queue.pop(0)
                msgs = []
                for k in range(hreye_config.number_of_eyes):
                    msg = HREyeState()
                    msg.header = Header()
                    msg.hreye_index = k
                    msg.state = state 

                    # print(state)
                    state_publisher.publish(msg)
                
            else:
                raise NotImplementedError("Not sure how to handle this HREye mode {}".format(hreye_config.mode))
                
        else:
            pass # This is where we publish the default state.

        
        rate.sleep()

else:
    pass