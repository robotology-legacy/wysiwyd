#!/usr/bin/env python

# Current version: 20160315
# Written by Tobias Fischer
# t.fischer@imperial.ac.uk
#
# Description:
# This script can be used to update all yarp/icub/wysiwyd related projects at once
# For each project defined in the `projects` variable, it does a `git pull --rebase`,
# `make` and optionally `make install` or `sudo make install`.
# 
# The script therefore needs to be adapted to your policies (whether you do `(sudo) make install`)
# and the specific machine you use (as different modules will be build depending on the machine).
#
# The script can handle shared sources, as the `build` directory must be specified (e.g. `build-pc104` and `build-ikart`)
#
# Usage: 
# 1) Copy this file to /usr/local/src/robot or whereever your projects are located.
# 2) Change the variables use_sudo and use_make_install according to your setup.
# 2.1) if use_make_install=True, the script does a `make install` for each project
# 2.2) if use_sudo=True, the script does a `sudo make install` rather than `make install`
# 3) Specify the list of projects to be build in the `projects` variable. It must point to the build directory of the project.
# 4) If you share sources, make two copies of the file and change accordingly, e.g. `update-software-pc104.py` and `update-software-ikart.py`
# 5) Run `./update-software.py`.
# 6) For pc104, you can choose to set `projects_pullonly=['icub-firmware-build']` as there is no `make` necessary for this project.
#
# Warnings: 
# The projects must be in a clean state because of the `git pull --rebase`.
# I.e. there must not be any changes in any files in any of the projects.
# If you need to use the script in a non-clean state, proceed as follows (advanced!):
# 1) `cd unclean-project`
# 2) `git stash`
# 3) `cd ..`
# 4) `./update-software.py`
# 5) `cd unclean-project`
# 6) `git stash pop`
# 7) `make`; optionally: `(sudo) make install`
#
# Machines using this script:
# [iCubLyon01] All machines
# [iCubLondon01] All machines
# Let me know if you need help adapting for your machine

import subprocess
from contextlib import contextmanager
import os
import sys, getopt

def main(argv):
    use_sudo=False
    use_make_install=False

    if(use_sudo):
        subprocess.call('sudo test', shell=True)

    @contextmanager
    def cd(newdir):
        prevdir = os.getcwd()
        os.chdir(os.path.expanduser(newdir))
        try:
            yield
        finally:
            os.chdir(prevdir)

    projects=['yarp/build', 'icub-main/build', 'icub-contrib-common/build', 'kinect-wrapper/build', 'wysiwyd/main/build', \
              'icub-contrib-iit/stereo-vision/build', 'icub-contrib-iit/segmentation/build', 'icub-contrib-iit/iol/build']
    projects_pullonly=[]

    for project in projects:
        print('Build %s' % project)
        build_folder = './'+project
        with cd(build_folder):
            subprocess.check_call('git pull --rebase', shell=True)
            subprocess.check_call('make -j6', shell=True)
            if(use_sudo and use_make_install):
                subprocess.check_call('sudo make install', shell=True)
            elif(use_make_install):
                subprocess.check_call('make install', shell=True)

    for project in projects_pullonly:
        pull_folder = './'+project
        with cd(pull_folder):
            subprocess.check_call('git pull --rebase', shell=True)

if __name__ == "__main__":
   main(sys.argv[1:])
