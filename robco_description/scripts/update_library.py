#!/usr/bin/env python

# update_library.py - Update robco_description package's URDF and meshes from
# RobCo module library
# 
# Author: Bernhard Vorhofer
#
# For internal use at RobCo only - requires access to module library.
#

import sys
import os
import re
import shutil
import json
import xml.dom.minidom
import numpy as np
import traceback

BASE_MODULE_TYPE = 'Base'
LINK_MODULE_TYPE = 'Link'
DRIVE_MODULE_TYPE = 'Drive'
TOOL_MODULE_TYPE = 'Tool'

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

# ======================
# === FILE UTILITIES ===
# ======================

# Returns true if the given path conforms to the module directory naming scheme,
# false otherwise.
def isModuleDir(path):
    dirname = os.path.basename(path)
    return re.compile('^\d\d\d\d_.*').search(dirname) != None

# Returns true if the given path refers to an STL file, false otherwise.
def isStlFile(path):
    filename = os.path.basename(path)
    return filename.endswith('.stl')

# ==========================
# === JSON/XML UTILITIES ===
# ==========================

# Returns the rpy euler angles (extrinsic) for the given rotation matrix (as numpy
# array).
def rotationMatrixToRPY(matrix):
    r11, r12, r13 = matrix[0]
    r21, r22, r23 = matrix[1]
    r31, r32, r33 = matrix[2]

    pitch = -np.arcsin(r31)

    if r31 == 1: # catch gimbal lock
        roll = 0
        yaw = np.arctan2(-r12, -r13)
    elif r31 == -1: # catch gimbal lock
        yaw = 0
        roll = np.arctan2(r12, r13)
    else:
        roll = np.arctan2(r32, r33)
        yaw = np.arctan2(r21, r11)

    return roll, pitch, yaw

# Returns the tf_rpy and tf_xyz strings (as a tuple, in that order) for the given
# transformation matrix (in dictionary form, from JSON).
def getUrdfTransformFromJsonMatrix(matrixDict):
    x = matrixDict[0][3]
    y = matrixDict[1][3]
    z = matrixDict[2][3]

    tf_xyz = '{} {} {}'.format(x, y, z)

    mat = np.array(matrixDict)
    r, p, y = rotationMatrixToRPY(mat[0:3,0:3])

    tf_rpy = '{} {} {}'.format(r, p, y)

    return tf_rpy, tf_xyz

# ====================================
# === MODULE JSON TO XML FUNCTIONS ===
# ====================================

# Returns an XML/xacro string for the module based on the given dictionary
# representation of the module's JSON file.
def getModuleXacro(moduleDir, jsonDict):
    xmlStr = ''
    moduleType = jsonDict['module-type']
    if moduleType == DRIVE_MODULE_TYPE:
        xmlStr += getDriveModuleXacro(moduleDir, jsonDict)
    elif moduleType == LINK_MODULE_TYPE or moduleType == BASE_MODULE_TYPE:
        xmlStr += getLinkModuleXacro(moduleDir, jsonDict) # link and base modules are effectively the same
    elif moduleType == TOOL_MODULE_TYPE:
        xmlStr += getToolModuleXacro(moduleDir, jsonDict)

    return xmlStr

def getLinkModuleXacro(moduleDir, jsonDict):
    xmlStr = ''
    moduleType = jsonDict['module-type']

    typeId = '{:04d}'.format(jsonDict['type-id'])
    xmlStr += '<xacro:macro name="robco_module_{}" params="name next">'.format(typeId)

    if moduleType == BASE_MODULE_TYPE:
        xmlStr += '<xacro:robco_base_module name="${name}"'
    elif moduleType == LINK_MODULE_TYPE:
        xmlStr += '<xacro:robco_link_module name="${name}"'
    else:
        raise Exception('Invalid module type in getLinkModuleXacro')

    meshFilename = jsonDict['collisions_visuals']['proximal_visual_mesh']
    meshPath = os.path.join(moduleDir, meshFilename)

    tf_rpy, tf_xyz = getUrdfTransformFromJsonMatrix(jsonDict['kinematics']['proximal_transformation'])

    xmlStr += ' mesh_filename="{}"'.format(meshPath)
    xmlStr += ' tf_rpy="{}"'.format(tf_rpy)
    xmlStr += ' tf_xyz="{}"'.format(tf_xyz)
    xmlStr += ' next="${next}"/>'

    xmlStr += '</xacro:macro>'

    return xmlStr

def getDriveModuleXacro(moduleDir, jsonDict):
    xmlStr = ''
    moduleType = jsonDict['module-type']

    typeId = '{:04d}'.format(jsonDict['type-id'])
    xmlStr += '<xacro:macro name="robco_module_{}" params="name next">'.format(typeId)

    xmlStr += '<xacro:robco_drive_module name="${name}"'

    proximalMeshFilename = jsonDict['collisions_visuals']['proximal_visual_mesh']
    distalMeshFilename = jsonDict['collisions_visuals']['distal_visual_mesh']
    proximalMeshPath = os.path.join(moduleDir, proximalMeshFilename)
    distalMeshPath = os.path.join(moduleDir, distalMeshFilename)

    proximal_tf_rpy, proximal_tf_xyz = getUrdfTransformFromJsonMatrix(jsonDict['kinematics']['proximal_transformation'])
    distal_tf_rpy, distal_tf_xyz = getUrdfTransformFromJsonMatrix(jsonDict['kinematics']['distal_transformation'])

    xmlStr += ' proximal_mesh_filename="{}"'.format(proximalMeshPath)
    xmlStr += ' distal_mesh_filename="{}"'.format(distalMeshPath)
    xmlStr += ' proximal_tf_rpy="{}"'.format(proximal_tf_rpy)
    xmlStr += ' proximal_tf_xyz="{}"'.format(proximal_tf_xyz)
    xmlStr += ' distal_tf_rpy="{}"'.format(distal_tf_rpy)
    xmlStr += ' distal_tf_xyz="{}"'.format(distal_tf_xyz)
    xmlStr += ' next="${next}"/>'

    xmlStr += '</xacro:macro>'

    return xmlStr

def getToolModuleXacro(moduleDir, jsonDict):
    xmlStr = ''
    moduleType = jsonDict['module-type']

    typeId = '{:04d}'.format(jsonDict['type-id'])
    xmlStr += '<xacro:macro name="robco_module_{}" params="name">'.format(typeId)

    xmlStr += '<xacro:robco_end_effector name="${name}"'

    meshFilename = jsonDict['collisions_visuals']['proximal_visual_mesh']
    meshPath = os.path.join(moduleDir, meshFilename)

    tf_rpy, tf_xyz = getUrdfTransformFromJsonMatrix(jsonDict['kinematics']['proximal_transformation'])

    xmlStr += ' mesh_filename="{}"'.format(meshPath)
    xmlStr += ' tf_rpy="{}"'.format(tf_rpy)
    xmlStr += ' tf_xyz="{}"/>'.format(tf_xyz)

    xmlStr += '</xacro:macro>'

    return xmlStr

# ====================
# === MAIN PROGRAM ===
# ====================

def main():
    if (len(sys.argv) < 3):
        print("usage: update_library.py <module_lib_dir> <out_dir>")
        print("Internal use at RobCo only, for updating the package from the module library.")
        sys.exit(-1)

    inDir = sys.argv[1]
    outDir = sys.argv[2]
    print("Module database source is {}".format(inDir))
    print("Target package directory is at {}".format(outDir))

    print("")
    print("Generating URDF from module descriptions...")
    
    errorOccurred = False
    xmlStr = '<?xml version="1.0" ?>'
    xmlStr += '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
    xmlStr += '<xacro:include filename="$(find robco_description)/urdf/robco_modules_base.xacro"/>'
    moduleDirs = [(x[0], x[2]) for x in os.walk(inDir) if isModuleDir(x[0])]
    for dirPath, dirFiles in moduleDirs:
        dirName = os.path.basename(dirPath)
        sys.stdout.write("  {}...".format(dirName))
        sys.stdout.flush()

        # Create directory in target directory
        newDir = os.path.join(outDir, 'meshes', dirName)
        try:
            os.makedirs(newDir)
        except OSError:
            pass

        # Copy all immediate children STL files to target directory
        for f in dirFiles:
            if not isStlFile(f):
                continue

            srcPath = os.path.join(dirPath, f)
            dstPath = os.path.join(newDir, f)
            shutil.copy(srcPath, dstPath)

        # Parse module JSON
        jsonFilePath = os.path.join(dirPath, dirName + '.json')
        try:
            with open(jsonFilePath) as jsonFile:
                jsonStr = jsonFile.read()
                jsonDict = json.loads(jsonStr)

                xmlStr += getModuleXacro(dirName, jsonDict)

        except Exception as err:
            errorOccurred = True
            print(bcolors.FAIL + "FAIL" + bcolors.ENDC)
            traceback.print_exc()
            continue

        print(bcolors.OKGREEN + " OK" + bcolors.ENDC)

    xmlStr += '</robot>'

    dom = xml.dom.minidom.parseString(xmlStr)
    prettyXml = dom.toprettyxml()

    xacroDir = os.path.join(outDir, 'urdf')

    try:
        os.makedirs(xacroDir)
    except OSError:
        pass

    xacroPath = os.path.join(outDir, 'urdf', 'robco_modules.xacro')
    with open(xacroPath, 'w') as f:
        f.write(prettyXml)


if __name__ == "__main__":
    main()
