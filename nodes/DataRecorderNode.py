#!/usr/bin/python
# Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org>

import rospy
from enum import unique, Enum
import typing as typ

from data_recorder.data_recorder import DataRecorder
from std_srvs.srv import SetBool as BoolService
from std_srvs.srv import SetBoolRequest as BoolReq
from std_srvs.srv import SetBoolResponse as BoolRes


@unique
class RecorderAction(Enum):
    STOP = -1
    NOTHING = 0
    START = 1
    pass  # enum RecorderAction


class DataRecorderNode(object):

    def __init__(self):
        # ROS parameters
        self.record_script_file = rospy.get_param("~record_script", "")
        self.store_script_file = rospy.get_param("~store_script", "")
        self.record_script_cmd = rospy.get_param("~record_cmd", "")
        self.store_script_cmd = rospy.get_param("~store_cmd", "")
        self.__b_verbose = rospy.get_param("~verbose", True)

        # debug
        if self.__b_verbose:
            rospy.loginfo("activated verbose output")
            pass

        # declare ROS services
        self.record_srv = rospy.Service(
            '/data_recorder/record', BoolService, self.__handle_record_service)

        # setup flags
        self.__f_is_recording = False                   # type: bool

        # create recorder
        self.__recorder = DataRecorder(
            record_script_file=self.record_script_file,
            record_command=self.record_script_cmd,
            data_storage_script_file=self.store_script_file,
            storage_command=self.store_script_cmd,
            verbose=self.__b_verbose,
        )
        pass  # def __init__()

    def __handle_record_service(self,
                                req,                    # type: BoolReq
                                ):
        # type: (...) -> BoolRes
        """service callback for 'record' service"""
        rospy.logdebug("received record start/stop request")

        # setup response and action variables
        res_value = False                               # type: bool
        action = RecorderAction.NOTHING                 # type: RecorderAction

        # check whether record start or stop was requested
        if req.data:
            # start requested, check if not started otherwise start

            if self.__f_is_recording:
                # started & start (do nothing)
                rospy.logwarn("already recording -- doing nothing")
            else:
                # stopped & start (start recording)
                # debug output
                if self.__b_verbose:
                    rospy.loginfo("starting recording")
                    pass

                # set action to start observation
                action = RecorderAction.START
                pass
            pass
        else:
            # stop requested, check if started and stop
            if self.__f_is_recording:
                # started & stop (stop recording)
                # debug output
                if self.__b_verbose:
                    rospy.loginfo("stopping recording")
                    pass

                # set action to stop recording
                action = RecorderAction.STOP
                pass
            else:
                # stopped & stop (do nothing)
                rospy.logwarn("already stopped -- doing nothing")
                pass
            pass

        # check for action and perform action
        if action == RecorderAction.START:
            res_value = self.__recorder.start_recording()
            self.__f_is_recording = True
            res_value = True
            pass
        elif action == RecorderAction.STOP:
            res_value = self.__recorder.stop_recording()
            self.__f_is_recording = False
            res_value = True  # TODO(scm): hardcoded but not checked by autonomy
            pass
        else:
            # doing nothing
            res_value = True
            pass

        # setup response message
        res_msg = BoolRes()
        res_msg.success = res_value
        res_msg.message = self.__recorder.get_status_msg()

        # return response
        return res_msg
        pass  # def __handle_record_service(...)

    pass  # class DataRecorderNode


if __name__ == "__main__":
    # rospy.init_node("data_recorder", log_level=rospy.DEBUG)
    rospy.init_node("data_recorder", log_level=rospy.INFO)

    # Go to class functions that do all the heavy lifting.
    try:
        node = DataRecorderNode()
        # node.run()

    except rospy.ROSInterruptException:
        pass

    # Allow ROS to go to all callbacks.
    rospy.spin()
    pass  # if __name__ == '__main__'
