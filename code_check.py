data: "#! /usr/bin/env python3\nfrom geometry_msgs.msg import Vector3Stamped, TwistStamped,\
  \ PoseStamped, PointStamped\nimport math\nfrom mtc_wrapper.manipulation_interface\
  \ import ManipulationInterface\nimport rospy\nfrom moveit_msgs.msg import GripperTranslation\n\
  from lrp_server.msg import PoseInGripperStamped\n\ndef main():\n    #rospy.init_node('live_robot_programming',\
  \ anonymous=True)\n    manipulation_interface = ManipulationInterface(\"panda_arm\"\
  , \"hand\", \"panda_link8\")\n    manipulation_interface.create_task(\"Pick and\
  \ place\")\n\n    move_to_pre_wipe_pose = PoseStamped()\n    move_to_pre_wipe_pose.header.frame_id\
  \ = \"panda_link0\"\n    move_to_pre_wipe_pose.pose.position.x = 0.4635782241821289\n\
  \    move_to_pre_wipe_pose.pose.position.y = 0.042791008949279\n    move_to_pre_wipe_pose.pose.position.z\
  \    move_to_pre_wipe_pose.pose.orientation.y = 78.953910103140\n    move_to_pre_wipe_pose.pose.orientation.z\
  \ = 159.6060328469644\n    manipulation_interface.move_to_pose(move_to_pre_wipe_pose)\n\
  \n    wipe_motion = Vector3Stamped()\n    wipe_motion.header.frame_id = \"panda_link0\"\
  \n    wipe_motion.vector.x = 0.13\n    wipe_motion.vector.y = -0.13\n    manipulation_interface.translate(wipe_motion)\n\
  \n    manipulation_interface.plan()  \n    ###########################################################################\n\
  \    manipulation_interface.cleanup()\n \nif __name__ == '__main__':\n    main()\n\
  \n\n"
---
data: "#! /usr/bin/env python3\nfrom geometry_msgs.msg import Vector3Stamped, TwistStamped,\
  \ PoseStamped, PointStamped\nimport math\nfrom mtc_wrapper.manipulation_interface\
  \ import ManipulationInterface\nimport rospy\nfrom moveit_msgs.msg import GripperTranslation\n\
  from lrp_server.msg import PoseInGripperStamped\n\ndef main():\n    #rospy.init_node('live_robot_programming',\
  \ anonymous=True)\n    manipulation_interface = ManipulationInterface(\"panda_arm\"\
  , \"hand\", \"panda_link8\")\n    manipulation_interface.create_task(\"Pick and\
  \ place\")\n\n    move_to_pre_wipe_pose = PoseStamped()\n    move_to_pre_wipe_pose.header.frame_id\
  \ = \"panda_link0\"\n    move_to_pre_wipe_pose.pose.position.x = 0.4635782241821289\n\
  \    move_to_pre_wipe_pose.pose.position.y = 0.042791008949279\n    move_to_pre_wipe_pose.pose.position.z\
  \    move_to_pre_wipe_pose.pose.orientation.y = 78.953910103140\n    move_to_pre_wipe_pose.pose.orientation.z\
  \ = 159.6060328469644\n    manipulation_interface.move_to_pose(move_to_pre_wipe_pose)\n\
  \n    wipe_motion = Vector3Stamped()\n    wipe_motion.header.frame_id = \"panda_link0\"\
  \n    wipe_motion.vector.x = 0.13\n    wipe_motion.vector.y = -0.13\n    manipulation_interface.translate(wipe_motion)\n\
  \n    manipulation_interface.plan()  \n    ###########################################################################\n\
  \    manipulation_interface.cleanup()\n \nif __name__ == '__main__':\n    main()\n\
  \n\n"
  \ = 0.536597013475\n    move_to_pre_wipe_pose.pose.orientation.x = 44.66101495\n\
  \ = 0.53659701347\n    move_to_pre_wipe_pose.pose.orientation.x = 44.66101495\n\
