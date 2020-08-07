import numpy as np
import matplotlib.pyplot as plt
import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import thread
from std_msgs.msg import String
import time 

# Parameters
KP = 10.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m] #should be 30.0
OSCILLATIONS_DETECTION_LENGTH = 3
show_animation = True
pub = rospy.Publisher ('/path_planning', String , queue_size = 10)

def calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy):
    minx = min(min(ox), sx, gx) - AREA_WIDTH / 2.0
    miny = min(min(oy), sy, gy) - AREA_WIDTH / 2.0
    maxx = max(max(ox), sx, gx) + AREA_WIDTH / 2.0
    maxy = max(max(oy), sy, gy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    for ix in range(xw):
        x = ix * reso + minx
        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf
    return pmap, minx, miny

def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)

def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i
    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])
    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1
        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0

def Callback_loc(data):
    global sx
    global sy
    sx = 0 
    sy = 0
    sx = data.pose.pose.position.x
    sy = data.pose.pose.position.y

def Callback_obs(data):
    global gx 
    global gy 
    global ox 
    global oy
    gx=data.x  #add the goal x 
    gy=data.y  #add the goal y
    ox = []  # obstacle x position list [m]
    oy = []  # obstacle y position list [m]

    tx = gx+0.5
    ty=gy
    while tx<5:
        ox.append(tx)
        oy.append(ty)
        ox.append(tx)
        oy.append(ty-0.1)
        tx+=0.2
    tx = gx - 0.5
    ty = gy
    while tx > 0:
        ox.append(tx)
        oy.append(ty)
        ox.append(tx)
        oy.append(ty - 0.1)
        tx -= 0.2 


def get_motion_model():
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]
    return motion

def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy)
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)
    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    #previous_ids = deque()

    while d >= 0:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)
    print("Goal!!")
    return rx, ry
def logic():
    while not rospy.is_shutdown():
        grid_size = 0.2  # potential grid size [m]
        robot_radius = 0.4  # robot radius [m]
        lx, ly = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
        path_string=""
        for x,y in zip(lx,ly):
            path_string += "{},{},3;".format(x,y)
        pub.publish (path_string)
        rospy.sleep(0.08)
                

def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def main():
    print("potential_field_planning start")
    rospy.init_node ('publish_next_point', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber('/mavros/local_position/odom' , Odometry , Callback_loc)
    rospy.Subscriber('/framegoal', Point, Callback_obs) #select subscriber which gives goal 
    thread.start_new_thread(logic,())
    rospy.spin()
           
    
if __name__ == '__main__':
    print(__file__ + " start!!")
    try : 
        main()
    except rospy.ROSInterruptException:
        pass
    print(__file__ + " Done!!")
