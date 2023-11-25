from maps.map_intermediate_01 import MyMapIntermediate01
from maps.map_medium_02 import MyMapMedium02

from spg_overlay.gui_map.gui_sr import GuiSR

from solutions.my_drone_eval import MyDroneEval
from solutions.my_drone_random import MyDroneRandom
from solutions.my_drone_lidar_communication import MyDroneLidarCommunication
from examples.example_semantic_sensor import MyDroneSemantic


def main():
    #mymap = MyMapMedium02()
    mymap = MyMapIntermediate01()
    pg = mymap.construct_playground(drone_type=MyDroneRandom)

    gui = GuiSR(playground=pg, the_map=mymap)
    gui.run()

if __name__ == "__main__":
    main()
