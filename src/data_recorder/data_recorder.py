# Copyright (C) 2022 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org>

import subprocess
import os, signal
import typing as typ


class DataRecorder(object):

    def __init__(self,
                 record_script_file,                            # type: str
                 record_command,                                # type: str
                 record_stop_file,                              # type: str
                 data_storage_script_file,                      # type: str
                 storage_command,                               # type: str
                 verbose=False,                                 # type: bool
                 ):

        # set parameters
        self.__rec_script_file = record_script_file             # type: str
        self.__store_script_file = data_storage_script_file     # type: str
        self.__record_stop_file = record_stop_file              # type: str
        self.__record_cmd = record_command                      # type: str
        self.__store_cmd = storage_command                      # type: str
        self.__f_script_valid = self.__check_script_path()      # type: bool
        self.__b_verbose = verbose                              # type: bool

        # setup process parameters
        self.__proc_record = None                               # type: typ.Optional[int]

        # setup status variables (used for debugging)
        self.__is_recording = False                             # type: bool
        self.__status_msg = "STOP: record not started"          # type: str

        # debug
        if self.__b_verbose:
            print("[RECORD] finished setting up recorder")
            pass
        pass  # def __init__()

    def start_recording(self):
        # type: (...) -> bool

        if not self.__f_script_valid:
            # script could not be found, return false
            print("[RECORD] record script file not valid (%s)" % self.__rec_script_file)
            return False
            pass

        # TODO(scm): start recording with subservice here
        # add a savepath with cmd 'cwd='
        # do not start this with 'shell=True' if not necessary, otherwise you have to close also all subprocesses
        # see here for more information:
        #       https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        # self.__proc_record = subprocess.Popen([self.__rec_script_file, self.__record_cmd])
        self.__proc_record = subprocess.Popen([self.__rec_script_file, self.__record_cmd], preexec_fn=os.setsid)
        print("[RECORD] started recording with PID %d" % self.__proc_record.pid)

        # started successfully: set status to recording
        self.__status_msg = "REC:  recording data"
        self.__is_recording = True
        return True  # successfully started recording
        pass

    def stop_recording(self):
        # type: (...) -> bool

        # check if process was started
        if self.__proc_record is None:
            print("[RECORD] ERROR = record process not started")
            return False  # TODO(scm): should we actually return true here, since technically process is stopped
            pass

        # otherwise send SIGINT or stop it with another method
        # TODO(scm): stop process
        # self.__proc_record.terminate()
        # kill process group rather than just the parent process
        pgid = os.getpgid(self.__proc_record.pid)
        print("[RECORD] terminating group %d" % pgid)

        os.killpg(pgid, signal.SIGTERM)
        # # needs to be send twice for GNU Parallel
        os.killpg(pgid, signal.SIGTERM)

        # make sure records are killed
        if self.__b_verbose:
            print("[RECORD] stopping remaining recording nodes")
            pass
        kill_record_proc = subprocess.Popen([self.__record_stop_file], shell=True)
        kill_record_proc.wait()

        # debug
        if self.__b_verbose:
            print("[RECORD] terminated recording")
            pass

        # perform data merging to storage location
        proc_store = subprocess.Popen([self.__store_script_file, self.__store_cmd])
        print("[RECORD] started data storing with  PID %d" % proc_store.pid)
        try:
            proc_store.wait()  # TODO(scm): set timeout here
        except subprocess.TimeoutExpired:
            print("[RECORD] could not finish data merging within timeout")
            # INFO(scm): we are not changing return value here
            proc_store.terminate()
            pass

        # stopped successfully: set status to stopped
        self.__status_msg = "STOP: stopped recording"
        self.__is_recording = False
        return True  # successfully stopped recording
        pass

    def __check_script_path(self):
        # type: (...) -> bool
        """checks if the record script file exists"""
        if os.path.isfile(self.__rec_script_file):
            self.__status_msg = "OK:   record file %s found" % self.__rec_script_file
            return True
        else:
            self.__status_msg = "ERR:  record file %s NOT found" % self.__rec_script_file
            return False
        pass  # def __check_script_path()

    ####################
    # GETTER
    ####################

    def get_status_msg(self):
        # type: (...) -> str
        return self.__status_msg
        pass  # def get_status_msg()

    pass  # class DataRecorder
