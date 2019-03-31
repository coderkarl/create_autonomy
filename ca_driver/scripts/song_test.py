#!/usr/bin/env python

import rospy, math
import numpy as np

from std_msgs.msg import Int16
from ca_msgs.msg import DefineSong, PlaySong

import time
import sys

class SongTest():
    def __init__(self):
        
        rospy.init_node('song_test')
        #rospy.Subscriber('raw_cone_pose', PoseStamped, self.raw_cone_callback, queue_size=2)
        
        self.define_song_pub = rospy.Publisher('define_song', DefineSong, queue_size=1)
        self.play_song_pub = rospy.Publisher('play_song', PlaySong, queue_size=1)
        time.sleep(1.0)
        
        # Victory in Jesus
        # Christ the Solid Rock
        print('Initializing Create2 Song Test.')
        tune = DefineSong()
        tune.song = 3
        tune.length = 16
        #                 Eb,  Eb,   Bb,  Eb,  F,   Bb,  p,     G,   p,   Eb, F,  F,  G,   Ab,   Eb,  p
        tune.notes =     [51,  51,  46,  51,   53,  46,  0,     55,  0,   51, 53, 53, 55,  56,  51,  0]
        tune.durations = [0.6, 0.6, 0.6, 0.3,  0.6, 0.6, 0.6,   0.4, 0.3, 0.3,0.3,0.3,0.3, 0.6, 0.3, 0.6]
        #self.define_song_pub.publish(tune)
        
        tune.song = 0
        #tune.length = 9
        #tune.notes = [60,62,64,65,67,69,71,72,0]
        #tune.durations = [0.1]*9
        tune.length = 14
        #             C,  G, A, G, P, GGAG, P, FEDC
        tune.notes = [60, 55,57,55,0, 55,55,57,55,0, 53,52,50,48]
        tune.durations = [0.6, 0.3,0.4,0.4,0.4, 0.2,0.2,0.4,0.4,0.4, 0.4,0.7,0.7,0.8] 
        self.define_song_pub.publish(tune)
        
        tune.song = 1
        tune.length = 9
        #                  G,      B,       C            C#,   D
        tune.notes =     [55,  0,  59,  0,  60,  60, 0,  61,  62,  0]
        tune.durations = [0.4,0.2, 0.4,0.2, 0.2,0.1,0.2, 0.6, 0.4, 0.5]
        #self.define_song_pub.publish(tune)
        
        time.sleep(1.0)
        s = PlaySong()
        s.song = 0
        #self.play_song_pub.publish(s)
            
    #def raw_cone_callback(self, data):
        #print "Raw Cone Callback"

if __name__ == '__main__':
    try:
        song_test = SongTest()
        print("Starting Song Test")

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
