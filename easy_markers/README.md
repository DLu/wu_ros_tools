## Standard Markers Usage



```python
#!/usr/bin/python

import roslib; roslib.load_manifest('easy_markers')
import rospy
from easy_markers.generator import *

if __name__=='__main__':
    rospy.init_node('some_markers')
    pub = rospy.Publisher('/visualization_marker', Marker)
    gen = MarkerGenerator()
    gen.ns = '/awesome_markers'
    gen.type = Marker.SPHERE_LIST
    gen.scale = [.3]*3
    gen.frame_id = '/base_link'
   
    while not rospy.is_shutdown():
        gen.counter = 0
        t = rospy.get_time()

        gen.color = [1,0,0,1]
        m = gen.marker(points= [(0, i, (i+t)%5.0) for i in range(10)])
        pub.publish(m)
        gen.color = [0,1,0,1]
        m = gen.marker(points= [(0, i, (i-t)%5.0) for i in range(10)])
        pub.publish(m)
        rospy.sleep(.1)
```



![screenshot](doc/Screenshot-RViz.png)

## Interactive Usage



```python
#!/usr/bin/python

import roslib; roslib.load_manifest('easy_markers')
import rospy
from easy_markers.interactive import InteractiveGenerator

def callback(feedback):
    print feedback

if __name__=='__main__':
    rospy.init_node('itest')

    ig = InteractiveGenerator()
    ig.makeMarker(controls=["move_x", "rotate_x"])
    ig.makeMarker(controls=["move_y", "rotate_y"], pose=[1,0,0], description="X")
    rospy.spin()
```



![screenshot](doc/Screenshot-RViz-1.png)
