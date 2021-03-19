#!/usr/bin/env python

import sys, os, rospy, nav_msgs.msg
import numpy as np, matplotlib, matplotlib.pyplot as plt, scipy.misc
import pdb

def map_callback(msg):
    print('got map {}x{}'.format(msg.info.width, msg.info.height))
    #img = np.flipud(np.array(msg.data).reshape((msg.info.width, msg.info.height))) # this was backwards....
    img = np.flipud(np.array(msg.data).reshape((msg.info.height, msg.info.width)))
    #pdb.set_trace()
    #plt.imshow(img) #Needs to be in row,col order
    #plt.show()
    #matplotlib.image.imsave(img_path, img)
    scipy.misc.imsave(img_path, img)
    yaml_path = os.path.splitext(img_path)[0]+'.yaml'
    with open(yaml_path, 'w') as f:
        f.write('''image: {}
resolution: {:.3f}
origin: [{}, {}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
'''.format(os.path.basename(img_path), msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y))
    
if __name__ == '__main__':
    topic = '/costmap_node/costmap/costmap'
    img_path = sys.argv[1] if len(sys.argv)>1 else './tmp/costmap.png'
    rospy.init_node('save_costmap')
    rospy.Subscriber(topic, nav_msgs.msg.OccupancyGrid, map_callback, queue_size=1)
    rospy.spin()
