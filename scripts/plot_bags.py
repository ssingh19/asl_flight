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

debug_channels = ['/debug1','/debug2','/debug3','/debug4','/debug5','/debug6']
#debug_channels = ['/debug4','/debug5','/debug6']
#debug_channels = ['/debug8','/debug9','/debug10']
#debug_channels = ['/debug7']
#debug_channels = ['/debug11','/debug12']


debug_data = {}
for c in debug_channels:
    debug_data[c] = []

for topic,msg,t in bag.read_messages(topics=debug_channels):
    debug_data[topic].append(msg.data)

# %%

#offline.plot({'data':[ {'x': debug_data['/debug1'], 'y':debug_data['/debug3']},{'x': debug_data['/debug2'], 'y':debug_data['/debug4']}]})# for k in debug_channels], 'layout': layout})

# %%

#debug_channels = ['/debug10','/debug11','/debug12']
offline.plot({'data':[ {'y': debug_data[k]} for k in debug_channels]})


# %%

#offline.plot({'data':[ {'x': debug_data['/debug1'], 'y':debug_data['/debug2']}]})

# %%

#start = 7500
#end = 14000
#print np.mean(np.array(debug_data['/debug8'][start:end]))
#print np.mean(np.array(debug_data['/debug10'][start:end]))
