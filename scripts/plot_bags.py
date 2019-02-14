import plotly
import rosbag
from std_msgs.msg import Float64
import numpy as np

from plotly import offline
import plotly.graph_objs as go
offline.init_notebook_mode()

# %%

#date = "circle-0814"
#num = "80"

#bag = rosbag.Bag('/home/blandry/Code/catkin_ws/src/asl_flight/scripts/'+date+'-'+num+'.bag')

bag = rosbag.Bag('/home/sumeet/catkin_ws/src/asl_flight/scripts/sim.bag')

# debug_channels = ['/debug1','/debug2','/debug3','/debug4','/debug5','/debug6']
# debug_channels = ['/debug8','/debug9','/debug10']
# debug_channels = ['/debug7']
# debug_channels = ['/debug11','/debug13','/debug12']


debug_data = {}
for c in debug_channels:
    debug_data[c] = []

for topic,msg,t in bag.read_messages(topics=debug_channels):
    debug_data[topic].append(msg.data)

# XY plot
layout = go.Layout(height=1200,width=1200,)
# offline.plot({'data':[ {'x': debug_data['/debug1'], 'y':debug_data['/debug3']},{'x': debug_data['/debug2'], 'y':debug_data['/debug4']}], 'layout': layout})
# plot channels
offline.plot({'data':[ {'y': debug_data[k]} for k in debug_channels]})

# %%

# offline.plot({'data':[ {'x': debug_data['/debug1'], 'y':debug_data['/debug2']}]})

# %%

start = 3410
end = 16000

err_x = np.array(debug_data['/debug1'][start:end]) - np.array(debug_data['/debug2'][start:end])
err_y = np.array(debug_data['/debug3'][start:end]) - np.array(debug_data['/debug4'][start:end])
err_z = np.array(debug_data['/debug5'][start:end]) - np.array(debug_data['/debug6'][start:end])

err_net = np.sqrt(err_x**2.0 + err_y ** 2.0 + err_z ** 2.0)

print ("x: {} ({}), y: {} ({}), z: {} ({}), net: {} ({})".format(np.mean(err_x), np.amax(np.abs(err_x)),
                                                   np.mean(err_y), np.amax(np.abs(err_y)),
                                                   np.mean(err_z), np.amax(np.abs(err_z)),
                                                   np.mean(err_net), np.amax(err_net)))
