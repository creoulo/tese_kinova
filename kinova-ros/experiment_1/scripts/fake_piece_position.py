#! /usr/bin/env python
""""""

import rospy
import experiment_1_msgs.srv

def initServices():
    #Instanciate all the services
    rospy.Service(name="experiment_1_msgs/PosPiece", service_class=PosPiece,
                                                    handler=pos_piece_handler)

#define all the handlers
def pos_piece_handler(req):

   return

if __name__ == '__main__':
  try:
    #--------------------------------------------------------------------------
    rospy.init_node('detect_piece')


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
