#this robot is used to collect eggs in scattered duck houses. It is able to use slam to navigate itself to the designatted places, and search for eggs in the near area.
#slam algorithm is cartographer, object detection algorithm is yolov5, the arm uses MoveIt! for planning, there isn't much original algorithm. Before running, you'll need to install the requirements first.
#this robot still has a lot of drawbacks, and will not be maintained by the author anymore, If any one wants to push through, contact the author or publish issues
#main fold of the egg collector, only essential packs.
#use mapping.launch to start mapping, you may use wheeltec_robot_rc to remotely control the robot to explore around and stcript_name.sh to save the map to /map, robot arm will also be activated, it should be set to retracted before you start mapping.
#use navigation.launch to launch use the saved map as navigation map file.
#use yolov5ros yolonoed.launch to start the collecting and search node.
