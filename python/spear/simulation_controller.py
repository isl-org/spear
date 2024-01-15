#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import msgpackrpc
import os
import psutil
import spear
from subprocess import Popen
import sys
import time

class SimulationController():
    def __init__(self, config):
        
        self._config = config
        self.rpc_client: msgpackrpc.Client

        self._request_launch_unreal_instance()
        self._initialize_rpc_client()
        self._initialize_unreal_instance()

    def close(self):
        # Note that in the constructor, we launch the Unreal instance first and then initialize the RPC client. Normally,
        # we would do things in the reverse order here. But if we close the client first, then we can't send a command to
        # the Unreal instance to close it. So we close the Unreal instance first and then close the client.
        self._request_close_unreal_instance()
        self._close_rpc_client()

    def n_ticks(self, num_of_ticks=1):
        for _ in range(num_of_ticks):
            self.begin_tick()
            self.tick()
            self.end_tick()

    def begin_tick(self):
        self.rpc_client.call("begin_tick")

    def tick(self):
        self.rpc_client.call("tick")

    def end_tick(self):
        self.rpc_client.call("end_tick")

    def get_byte_order(self):
        unreal_instance_byte_order = self.rpc_client.call("get_byte_order")
        rpc_client_byte_order = sys.byteorder
        if unreal_instance_byte_order == rpc_client_byte_order:
            return None
        elif unreal_instance_byte_order == "little":
            return "<"
        elif unreal_instance_byte_order == "big":
            return ">"
        else:
            assert False

    def _request_launch_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            spear.log('SPEAR.LAUNCH_MODE == "running_instance" so we assume that the Unreal instance has already launched...')
            return

        spear.log("Launching Unreal instance...")

        # write temp file
        temp_dir = os.path.realpath(os.path.join(self._config.SPEAR.TEMP_DIR))
        temp_config_file = os.path.realpath(os.path.join(temp_dir, "config.yaml"))

        spear.log("Writing temp config file: " + temp_config_file)

        os.makedirs(temp_dir, exist_ok=True)
        with open(temp_config_file, "w") as output:
            self._config.dump(stream=output, default_flow_style=False)

        # create a symlink to SPEAR.PAKS_DIR
        if self._config.SPEAR.LAUNCH_MODE == "standalone_executable" and self._config.SPEAR.PAKS_DIR != "":

            assert os.path.exists(self._config.SPEAR.STANDALONE_EXECUTABLE)
            assert os.path.exists(self._config.SPEAR.PAKS_DIR)

            if sys.platform == "win32":
                paks_dir = \
                    os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(self._config.SPEAR.STANDALONE_EXECUTABLE)), "..", "..", "Content", "Paks"))
            elif sys.platform == "darwin":
                paks_dir = \
                    os.path.realpath(os.path.join(self._config.SPEAR.STANDALONE_EXECUTABLE, "Contents", "UE", "SpearSim", "Content", "Paks"))
            elif sys.platform == "linux":
                paks_dir = \
                    os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(self._config.SPEAR.STANDALONE_EXECUTABLE)), "SpearSim", "Content", "Paks"))
            else:
                assert False

            assert os.path.exists(paks_dir)

            # we don't use os.path.realpath here because we don't want to resolve the symlink
            spear_paks_dir = os.path.join(paks_dir, "SpearPaks")

            if spear.path_exists(spear_paks_dir):
                spear.log(f"File or directory or symlink exists, removing: {spear_paks_dir}")
                spear.remove_path(spear_paks_dir)

            spear.log(f"Creating symlink: {spear_paks_dir} -> {self._config.SPEAR.PAKS_DIR}")
            os.symlink(self._config.SPEAR.PAKS_DIR, spear_paks_dir)

        # provide additional control over which Vulkan devices are recognized by Unreal
        if len(self._config.SPEAR.VK_ICD_FILENAMES) > 0:
            spear.log("Setting VK_ICD_FILENAMES environment variable: " + self._config.SPEAR.VK_ICD_FILENAMES)
            os.environ["VK_ICD_FILENAMES"] = self._config.SPEAR.VK_ICD_FILENAMES

        # set up launch executable and command-line arguments
        launch_args = []

        if self._config.SPEAR.LAUNCH_MODE == "uproject":
            launch_executable = self._config.SPEAR.UNREAL_EDITOR_EXECUTABLE
            launch_args.append(self._config.SPEAR.UPROJECT)
            launch_args.append("-game") # launch the game using uncooked content
        elif self._config.SPEAR.LAUNCH_MODE == "standalone_executable":
            launch_executable = self._config.SPEAR.STANDALONE_EXECUTABLE
        else:
            assert False

        assert os.path.exists(launch_executable)

        launch_executable_name, launch_executable_ext = os.path.splitext(launch_executable)

        if sys.platform == "win32":
            assert launch_executable_name[-4:] == "-Cmd"
            assert launch_executable_ext == ".exe"
            launch_executable_internal = launch_executable
        elif sys.platform == "darwin":
            assert launch_executable_ext == ".app"
            launch_executable_internal = os.path.realpath(os.path.join(launch_executable, "Contents", "MacOS", os.path.basename(launch_executable_name)))
        elif sys.platform == "linux":
            assert launch_executable_ext == ".sh"
            launch_executable_internal = launch_executable
        else:
            assert False

        assert os.path.exists(launch_executable_internal)

        launch_args.append("-windowed")
        launch_args.append("-resx={}".format(self._config.SPEAR.WINDOW_RESOLUTION_X))
        launch_args.append("-resy={}".format(self._config.SPEAR.WINDOW_RESOLUTION_Y))
        launch_args.append("-graphicsadapter={}".format(self._config.SPEAR.GPU_ID))
        launch_args.append("-nosound")
        # launch_args.append("-fileopenlog")         # generate a log of which files are opened in which order
        launch_args.append("-stdout")              # ensure log output is written to the terminal 
        launch_args.append("-fullstdoutlogoutput") # ensure log output is written to the terminal
        launch_args.append("-nologtimes")          # don't print timestamps next to log messages twice

        if self._config.SPEAR.RENDER_OFFSCREEN:
            launch_args.append("-renderoffscreen")

        if self._config.SPEAR.UNREAL_INTERNAL_LOG_FILE != "":
            launch_args.append("-log={}".format(self._config.SPEAR.UNREAL_INTERNAL_LOG_FILE))
       
        launch_args.append("-config_file={}".format(temp_config_file))

        for a in self._config.SPEAR.CUSTOM_COMMAND_LINE_ARGUMENTS:
            launch_args.append("{}".format(a))

        cmd = [launch_executable_internal] + launch_args

        spear.log("Launching executable with the following command-line arguments:")
        spear.log_no_prefix(" ".join(cmd))

        spear.log("Launching executable with the following config values:")
        spear.log_no_prefix(self._config)

        popen = Popen(cmd)
        self._process = psutil.Process(popen.pid)

        # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
        status = self._process.status()
        if status not in ["running", "sleeping", "disk-sleep"]:
            spear.log("ERROR: Unrecognized process status: " + status)
            spear.log("ERROR: Killing process " + str(self._process.pid) + "...")
            self._force_kill_unreal_instance()
            self._close_rpc_client()
            assert False

    def _request_close_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            spear.log('SPEAR.LAUNCH_MODE == "running_instance" so we assume that the Unreal instance should remain open...')
            return

        spear.log("Closing Unreal instance...")

        self._request_close()
        self._wait_until_unreal_instance_is_closed()

        spear.log("Finished closing Unreal instance.")

    def _initialize_rpc_client(self):

        spear.log("Initializing RPC client...")
        
        # if we're connecting to a running instance, then we assume that the RPC server is already running and only try to connect once
        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            connected = False
            try:
                self.rpc_client = msgpackrpc.Client(
                    msgpackrpc.Address(self._config.SIMULATION_CONTROLLER.IP, self._config.SIMULATION_CONTROLLER.PORT),
                    timeout=self._config.SPEAR.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS,
                    reconnect_limit=self._config.SPEAR.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                self._ping()
                connected = True
            except:
                # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14 for more details.
                self._close_rpc_client()

        # otherwise try to connect repeatedly, since the RPC server might not have started yet
        else:
            connected = False
            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while not connected and elapsed_time_seconds < self._config.SPEAR.RPC_CLIENT_INITIALIZE_CONNECTION_MAX_TIME_SECONDS:
                # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
                status = self._process.status()
                if status not in ["disk-sleep", "running", "sleeping", "stopped"]:
                    spear.log("ERROR: Unrecognized process status: " + status)
                    spear.log("ERROR: Killing process " + str(self._process.pid) + "...")
                    self._force_kill_unreal_instance()
                    self._close_rpc_client()
                    assert False
                try:
                    self.rpc_client = msgpackrpc.Client(
                        msgpackrpc.Address(self._config.SIMULATION_CONTROLLER.IP, self._config.SIMULATION_CONTROLLER.PORT), 
                        timeout=self._config.SPEAR.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.SPEAR.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    self._ping()
                    connected = True
                except:
                    # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                    # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14 for more details.
                    self._close_rpc_client()
                time.sleep(self._config.SPEAR.RPC_CLIENT_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        if not connected:
            if self._config.SPEAR.LAUNCH_MODE != "running_instance":
                spear.log("ERROR: Couldn't connect, killing process " + str(self._process.pid) + "...")
                self._force_kill_unreal_instance()
                self._close_rpc_client()
            assert False

        spear.log("Finished initializing RPC client.")

    def _initialize_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            spear.log('SPEAR.LAUNCH_MODE == "running_instance" so we assume that the Unreal instance is already initialized...')
            return

        spear.log("Initializing Unreal instance, warming up for " + str(1 + self._config.SPEAR.NUM_EXTRA_WARMUP_TICKS) + " ticks...")

        # Do at least one complete tick to guarantee that we can receive valid observations. If we don't
        # do this, it is possible that Unreal will return an initial visual observation of all zeros. We
        # generally also want to do more than one tick to warm up various caches and rendering features
        # that leverage temporal coherence between frames.
        for i in range(1 + self._config.SPEAR.NUM_EXTRA_WARMUP_TICKS):
            self.begin_tick()
            self.tick()
            self.end_tick()

        spear.log("Finished initializing Unreal instance.")

    def _wait_until_unreal_instance_is_closed(self):
        try:
            status = self._process.status()
        except psutil.NoSuchProcess:
            pass
        else:
            while status in ["running", "sleeping", "disk-sleep"]:
                time.sleep(1.0)
                try:
                    status = self._process.status()
                except psutil.NoSuchProcess:
                    break

    def _force_kill_unreal_instance(self):
        spear.log("Forcefully killing Unreal instance...")
        self._process.terminate()
        self._process.kill()

    def _close_rpc_client(self):
        spear.log("Closing RPC client...")
        self.rpc_client.close()
        self.rpc_client._loop._ioloop.close()

    def _ping(self):
        return self.rpc_client.call("ping")

    def _request_close(self):
        self.rpc_client.call("request_close")
