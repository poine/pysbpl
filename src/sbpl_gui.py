#!/usr/bin/env python
import logging, sys, os, math, numpy as np, cv2, gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, GLib, GObject
import matplotlib
from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas
import matplotlib.pyplot as plt
import rospy, geometry_msgs.msg, nav_msgs.msg, tf, threading, subprocess
import pdb
import pysbpl, guidance

# map_resolution = 0.005
# #map_origin = [0., -0.3]
# map_origin = [0., 0.]
# map_w, map_h = 370, 500

# def world_to_map(prl): return [1, -1]*(prl-map_origin)/map_resolution+[0, map_h]


class Node:
    def __init__(self, nav_goal_handler=None, **kwargs):
        self.nav_goal_handler = nav_goal_handler
        self.pub_path = rospy.Publisher('sbpl/path', nav_msgs.msg.Path, queue_size=1)
        self.pub_start = rospy.Publisher('/sbpl/start', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
        self.pub_goal = rospy.Publisher('/sbpl/goal', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
        self.path = None
        self.start = None
        self.goal = None
        self.nav_goal_sub = rospy.Subscriber('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, self.on_nav_goal)
        self._request_costmap = True
        
    def publish_path(self):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="map"
        for l, y in zip(self.path.points, self.path.headings):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            o = pose.pose.orientation
            o.x, o.y, o.z, o.w = tf.transformations.quaternion_from_euler(*[0, 0, y])
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)


    def publish_markers(self):
        msg = geometry_msgs.msg.PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = self.start[0]
        msg.pose.pose.position.y = self.start[1]
        msg.pose.pose.position.z = 0
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = tf.transformations.quaternion_from_euler(*[0, 0, self.start[2]])
        self.pub_start.publish(msg)
        msg.pose.pose.position.x = self.goal[0]
        msg.pose.pose.position.y = self.goal[1]
        o.x, o.y, o.z, o.w = tf.transformations.quaternion_from_euler(*[0, 0, self.goal[2]])
        self.pub_goal.publish(msg)
         
    def set_path(self, _path):
        self.path = _path

    def set_start(self, x, y, yaw):
        self.start = (x, y, yaw)

    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)
        
    def run(self):
        rate = rospy.Rate(5.)
        try:
            while not rospy.is_shutdown():
                self.periodic()
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

    def periodic(self):
        if self.path is not None: self.publish_path()
        if self.start is not None and self.goal is not None:
            self.publish_markers()
        if self._request_costmap:
            self.fetch_costmap()
            self._request_costmap = False
            
    def on_nav_goal(self, msg):
        o = msg.pose.orientation; yaw = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        GLib.idle_add(self.nav_goal_handler, msg.pose.position.x, msg.pose.position.y, yaw)


    def fetch_costmap(self):
        msg = rospy.wait_for_message('/costmap_node/costmap/costmap', nav_msgs.msg.OccupancyGrid)
        #print ('got_costmap', msg)
        # header: 
        #   seq: 0
        #   stamp: 
        #     secs: 1511352708
        #     nsecs: 336167472
        #   frame_id: map
        # info: 
        #   map_load_time: 
        #     secs: 0
        #     nsecs:         0
        #   resolution: 0.00499999988824
        #   width: 500
        #   height: 240
        #   origin: 
        #     position: 
        #       x: 0.0
        #       y: 0.0
        #       z: 0.0
        #     orientation: 
        #       x: 0.0
        #       y: 0.0
        #       z: 0.0
        #       w: 1.0
        # data:
        o = msg.info.origin.position
        kwargs = {'img': np.flipud(np.array(msg.data).reshape((msg.info.height, msg.info.width)))/255.,
                  'resolution': msg.info.resolution,
                  'origin': [o.x, o.y, o.z]
        }
        self.costmap = guidance.Map(**kwargs)
        print 'node got costmap'

    def request_costmap(self): self._request_costmap = True
        
    def register_nav_goal_handler(self, _handler):   
        self.nav_goal_handler = _handler

        
            
class Model:
    def __init__(self, **kwargs):
        self.node = Node()
        self.map_path = '/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/enac_bench/track_test2.yaml'
        self.params = {
            'map': guidance.Map(yaml_path='/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/enac_bench/track_test2_costmap.yaml'),
            'perimeter':[[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]],
            'start':[1.21, 0.18, 0.0],
            'goal': [1.20, 0.18, 0.], # one loop
            'goal_tol':[0.05, 0.05, 0.1],
            'vel':0.1, 'time_45_deg':10,
            'mprim_path':'/home/poine/work/oscar.git/oscar/oscar_navigation/params/sbpl/oscar_3.mprim',
            'obs_thresh': 100,# 254, #165 # 0.65*255
            'inscribed_thresh': 90,
            'possibly_circumscribed_thresh': 80#165
        }
        self.run_map_server()
        self.run_costmap()

    def run_map_server(self):
        cmd = 'rosrun map_server map_server {} __name:=map_server'.format(self.map_path)
        self.map_server = subprocess.Popen(cmd, shell=True, close_fds=True)

    def run_costmap(self):
        costmap_node_name = 'costmap_node'
        if 0:
            for pn, p in [('plugins', '[]'),
                          ('publish_frequency', '1')]:
                subprocess.call( 'rosparam set /{}/{} {}'.format(costmap_node_name, pn, p), shell=True)
        else:
            with open('/tmp/tmp_costmap_cfg.yaml', 'w') as f:
                f.write('''
global_frame: /map
robot_base_frame: /base_footprint
transform_tolerance: 0.1
footprint: [[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]]
plugins: 
    - {name: static_map,       type: "costmap_2d::StaticLayer"}
    - {name: inflation, type: "costmap_2d::InflationLayer"}

inflation:
  enabled:              true
  cost_scaling_factor:  10.0  # exponential rate at which the obstacle cost drops off (default: 10)
  inflation_radius:     0.06 #0.065 # max. distance from an obstacle at which costs are incurred for planning paths.

static:
  enabled:              true
  lethal_cost_threshold: 100
  map_topic:            "/map"
''')
            
            subprocess.call( 'rosparam load -v /tmp/tmp_costmap_cfg.yaml costmap_node/costmap', shell=True)    
        
        cmd = 'rosrun costmap_2d costmap_2d_node __name:={}'.format(costmap_node_name)
        self.costmap_node = subprocess.Popen(cmd, shell=True, close_fds=True)
        cmd = 'rosrun tf static_transform_publisher {} __name:=map_to_base_footprint_transform'.format('0 0 0 0 0 0 /base_footprint /map 100')
        self.static_tf_node = subprocess.Popen(cmd, shell=True, close_fds=True)
        
    def quit(self):
        print 'terminate map server'
        self.map_server.terminate()
        subprocess.call('rosnode kill /map_server', shell=True)
        self.costmap_node.terminate()
        subprocess.call('rosnode kill /costmap_node', shell=True)
        self.static_tf_node.terminate()
        subprocess.call('rosnode kill /map_to_base_footprint_transform', shell=True)

    def run_sbpl(self):
        self.env = pysbpl.EnvironmentNAVXYTHETALAT()
        self.params['map'] = self.node.costmap
        #pdb.set_trace()
        self.start_id, self.goal_id = self.env.InitializeEnv(**self.params)
        self.planner = pysbpl.ARAPlanner(self.env)
        self.planner.initialize(self.start_id, self.goal_id)

        points, headings = self.planner.run()
        points += self.params['map'].origin[:2]
        self.path = guidance.path.Path(points=points, headings=headings)
        self.node.path = self.path

    def save_path(self, file_path):
         self.path.save(file_path)

    def load_path(self, file_path):
        self.path = guidance.path.Path(load=file_path)
        self.node.path = self.path
        p0 = [self.path.points[0,0], self.path.points[0,1],self.path.headings[0]]
        self.node.set_start(*p0)
        p1 = [self.path.points[-1, 0], self.path.points[-1, 1], self.path.headings[-1]]
        self.node.set_goal(*p1)
         
    def register_nav_goal_handler(self, _handler):
        self.node.register_nav_goal_handler(_handler)

    def set_goal(self, _p):
        self.params['goal'] = _p
        self.node.set_goal(*_p)

    def set_start(self, _p):
        self.params['start'] = _p
        self.node.set_start(*_p)


    def get_goal(self): return self.node.goal
    def get_start(self): return self.node.start

        
# class Window:

#     def __init__(self):
#         pass

#     def display_map(self, m):
#         plt.imshow(m.img)
#         ticks = matplotlib.ticker.FuncFormatter(lambda _x, pos: '{0:g}'.format(_x*m.resolution))
#         plt.gca().xaxis.set_major_formatter(ticks)
#         plt.gca().set_xlabel('dimensions in meters')
#         plt.gca().yaxis.set_major_formatter(ticks)
#         plt.gca().set_ylabel('dimensions in meters')

#     def display_path(self, points):
#         map_points = world_to_map(points)
#         plt.plot(map_points[:,0], map_points[:,1], marker='.', markersize=10, color='r')

        
#     def show(self):
#         plt.show()


class GUI:
    def __init__(self):
        self.b = Gtk.Builder()
        gui_xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sbpl_gui.xml')
        self.b.add_from_file(gui_xml_path)
        self.window = self.b.get_object("window")
        self.window.set_title('SBPL')
        self.start_entries = [self.b.get_object("entry_start_"+axis) for axis in ['x', 'y', 'yaw']]
        self.goal_entries = [self.b.get_object("entry_goal_"+axis) for axis in ['x', 'y', 'yaw']]
        self.last_dir = os.getcwd()
        self.window.show_all()

    def display_start(self, _p):
        for i in range(3):
            self.start_entries[i].set_text('{}'.format(_p[i]))
        
    def display_goal(self, _p):
        for i in range(3):
            self.goal_entries[i].set_text('{}'.format(_p[i]))

    def request_path(self, action):
        dialog = Gtk.FileChooserDialog("Please choose a file", self.window, action,
                                       (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                        Gtk.STOCK_OPEN, Gtk.ResponseType.OK))
        dialog.set_current_folder(self.last_dir)
        ret = dialog.run()
        file_path = dialog.get_filename() if ret == Gtk.ResponseType.OK else None
        dialog.destroy()
        if file_path is not None:
            self.last_dir = os.path.dirname(file_path)
        return file_path



        
        
class App:
    def __init__(self):
        self.gui = GUI()
        self.model = Model()
        self.register_gui()
        self.model.register_nav_goal_handler(self.on_nav_goal)
        self.gui.display_start(self.model.params['start'])
        self.gui.display_goal(self.model.params['goal'])
         
    def register_gui(self):
        self.gui.window.connect("delete-event", self.quit)
        self.gui.b.get_object("button_update_costmap").connect("clicked", self.on_update_costmap_clicked)
        self.gui.b.get_object("button_run").connect("clicked", self.on_run_clicked)
        self.gui.b.get_object("button_save_path").connect("clicked", self.on_save_path_clicked)
        self.gui.b.get_object("button_load_path").connect("clicked", self.on_load_path_clicked) 
        for i in range(3):
            self.gui.start_entries[i].connect("activate", self.on_start_callback)
        for i in range(3):
            self.gui.goal_entries[i].connect("activate", self.on_goal_callback)

        #self.gui.b.get_object("togglebutton_rec_start").connect("toggled", self.on_rec_start_toggled)

    def on_update_costmap_clicked(self, b):
        self.model.node.request_costmap()
        
    def on_run_clicked(self, button):
        self.model.run_sbpl()

    def on_save_path_clicked(self, button):
        filename = self.gui.request_path(Gtk.FileChooserAction.SAVE)
        if filename is not None:
            self.model.save_path(filename)

    def on_load_path_clicked(self, button):
        filename = self.gui.request_path(Gtk.FileChooserAction.OPEN)
        if filename is not None:
            self.model.load_path(filename)
            self.gui.display_start(self.model.get_start())
            self.gui.display_goal(self.model.get_goal())
            
    def on_start_callback(self, entry):
        _p = list(self.model.params['start'])
        for i in range(3):
            try:
                _p[i] = float(self.gui.start_entries[i].get_text())
            except ValueError:
                pass
        self.set_start(_p)
        
    def on_goal_callback(self, entry):
        _p = list(self.model.params['goal'])
        for i in range(3):
            try:
                _p[i] = float(self.gui.goal_entries[i].get_text())
            except ValueError:
                pass
        self.set_goal(_p)
        
    #def on_rec_start_toggled(self, button):
    #    print 'hello'
        

    def on_nav_goal( self, x, y, yaw):
        if self.gui.b.get_object("togglebutton_rec_start").get_active():
            self.set_start([x, y, yaw])
        if self.gui.b.get_object("togglebutton_rec_goal").get_active():
            self.set_goal([x, y, yaw])   
        
    def set_goal(self, _p):
        self.model.set_goal(_p)
        self.gui.display_goal(_p)
        

    def set_start(self, _p):
        self.model.set_start(_p)
        self.gui.display_start(_p)

    def run(self):
        self.ros_thread = threading.Thread(target=self.model.node.run)
        self.ros_thread.start()
        Gtk.main()

    def quit(self, a, b):
        rospy.signal_shutdown("just because")
        self.ros_thread.join()
        print 'ros thread ended'
        self.model.quit()
        print 'all dead'
        Gtk.main_quit() 


if __name__ == '__main__':
    rospy.init_node('sbpl_gui')
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=3, linewidth=300)
    App().run()
