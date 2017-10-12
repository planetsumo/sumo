# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2017-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    __init__.py
# @author Leonhard Luecken
# @date   2017-04-09
# @version $Id$

"""
simpla - A simple platooning plugin for TraCI

simpla is a configurable, simple platooning plugin for TraCI.
A platooning configuration has to be created before using.
Its possible elements are given in the example configuration file
'simpla_example.cfg.xml'

Information about vType mappings between original and
platooning vTypes has to be supplied. This can be done directly
in the configuration xml-file by using 'vTypeMapLeader', 'vTypeMapFollower' and 'vTypeMapCatchup'
elements or by reference to seperate files which define the mappings as
'originalVType : mappedVType'

All specified vTypes should be available within the simulation, the "default" type
is optional and used whenever information is missing for some original type
if no default is specified, the original type remains unchanged within the platoon.

For the definition of platooning vTypes for existing basic vTypes,
and generating vTypeMapping-files see the script generateModifiedVTypes.py.

Usage:
1) import simpla into your traci script.
2) After establishing a connection to SUMO with traci, call simpla.load(<configuration_filename>)
3) Only applies to SUMO version < 0.30: After starting simpla, call simpla.update() after each call to traci.simulationStep()

Notes:
1) simpla changes the vehicle types, speedfactors, and lane changemodes of all connected vehicles.
   If your application does so as well, this might have unintended consequences.
2) Currently, steps of lengths other than DeltaT are not supported (i.e. if traci.simulationStep()
   is called with argument when simpla is running this may yield undesired behaviour).
3) simpla adds subscriptions to VAR_ROAD_ID, VAR_LANE_INDEX (and currently VAR_LANE_ID) and removes them when stopped
"""


import traci

class SimplaException(Exception):
    '''
    Simple exception raised by simpla.
    '''
    def __init__(self,*args,**kwargs):
        super(SimplaException, self).__init__(*args,**kwargs)
        


import simpla._config
import simpla._reporting as rp
import simpla._platoonmanager

warn = rp.Warner("simpla")
_mgr = None
_useStepListener = 'addStepListener' in dir(traci)
_emergencyDecelImplemented = 'VAR_EMERGENCY_DECEL' in dir(traci.constants)

if not _emergencyDecelImplemented:
    # Old traci version. No emergency decel present.
    if rp.VERBOSITY >= 1:
        warn("Using old traci version assuming emergency decel == decel", True)
    # Set emergency decel to decel
    traci.constants.VAR_EMERGENCY_DECEL = 0x7b
    traci.vehicletype.getEmergencyDecel = traci.vehicletype.getDecel


def load(config_filename):
    '''
    Load the config from file and create a Platoon Manager
    '''
    global _mgr
    _config.load(config_filename)
    _mgr = _platoonmanager.PlatoonManager()
    if _useStepListener:
        # For SUMO version >= 0.30
        traci.addStepListener(_mgr)


def stop():
    '''
    Stop the PlatoonManager
    '''
    global _mgr
    if _mgr is not None:
        _mgr.stop()
        traci.removeStepListener(_mgr)
    _mgr = None


def update():
    '''
    Function called each simulation step. Only to be used for SUMO version < 1.0
    '''
    global _mgr, warn
    if _mgr is not None:
        _mgr.step()
    else:
        if rp.VERBOSITY >= 1:
            warn("call simpla.init(<config_file>) before simpla.update()!")
