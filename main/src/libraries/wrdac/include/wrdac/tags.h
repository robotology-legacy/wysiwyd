/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ugo Pattacini, Stéphane Lallée
 * email:   ugo.pattacini@iit.it stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @defgroup wrdac Helper Routines
 *
 * @ingroup wysiwyd_libraries
 *
 * A container library containing defines and structures to formalize
 * knowledge within an objectPropertiesCollector module.
 *
 * @author Ugo Pattacini & Stéphane Lallée
 */
#ifndef __WYSIWYD_OPCTAGS_H__
#define __WYSIWYD_OPCTAGS_H__

#define EFAA_OPC_INVALID_ID                 -1

#define EFAA_OPC_ENTITY_TAG                 ("entity")
#define EFAA_OPC_ENTITY_OBJECT              ("object")
#define EFAA_OPC_ENTITY_RTOBJECT            ("rtobject")
#define EFAA_OPC_ENTITY_BODYPART            ("bodypart")
#define EFAA_OPC_ENTITY_AGENT               ("agent")
#define EFAA_OPC_ENTITY_ADJECTIVE           ("adjective")
#define EFAA_OPC_ENTITY_ACTION              ("action")
#define EFAA_OPC_ENTITY_RELATION            ("relation")

#define EFAA_OPC_OBJECT_NAME_TAG            ("name")
#define EFAA_OPC_OBJECT_PRESENT_TAG         ("isPresent")
#define EFAA_OPC_OBJECT_RTID_TAG            ("rt_id")
#define EFAA_OPC_OBJECT_RTPOSX_TAG          ("rt_position_x")
#define EFAA_OPC_OBJECT_RTPOSY_TAG          ("rt_position_y")
#define EFAA_OPC_OBJECT_RTPOSZ_TAG          ("rt_position_z")
#define EFAA_OPC_OBJECT_RTANGLE_TAG         ("rt_angle")
#define EFAA_OPC_OBJECT_RTDIMX_TAG          ("rt_dim_x")
#define EFAA_OPC_OBJECT_RTDIMY_TAG          ("rt_dim_y")
#define EFAA_OPC_OBJECT_RTDIMZ_TAG          ("rt_dim_z")
#define EFAA_OPC_OBJECT_SALIENCY            ("saliency")
#define EFAA_OPC_OBJECT_VALUE               ("value")
#define EFAA_OPC_OBJECT_CURID_TAG           ("cur_id")

#define EFAA_OPC_OBJECT_ROBOTPOS_TAG        ("position_3d")
#define EFAA_OPC_OBJECT_ROBOTPOSX_TAG       ("robot_position_x")
#define EFAA_OPC_OBJECT_ROBOTPOSY_TAG       ("robot_position_y")
#define EFAA_OPC_OBJECT_ROBOTPOSZ_TAG       ("robot_position_z")
#define EFAA_OPC_OBJECT_ROBOTORX_TAG        ("robot_orientation_x")
#define EFAA_OPC_OBJECT_ROBOTORY_TAG        ("robot_orientation_y")
#define EFAA_OPC_OBJECT_ROBOTORZ_TAG        ("robot_orientation_z")

#define EFAA_OPC_OBJECT_GUI_COLOR_R         ("color_r")
#define EFAA_OPC_OBJECT_GUI_COLOR_G         ("color_g")
#define EFAA_OPC_OBJECT_GUI_COLOR_B         ("color_b")
#define EFAA_OPC_OBJECT_GUI_COLOR_ALPHA     ("color_aplha")

// OPC : Frames
#define EFAA_OPC_FRAME_NAME                 ("frameName")
#define EFAA_OPC_FRAME_MATRIX               ("frameMatrix")
#define EFAA_OPC_FRAME_SCALE                ("frameScale")

// OPC : Spatial relations
#define EFAA_OPC_OBJECT_SPATIAL_CONTAINS    ("contains")
#define EFAA_OPC_OBJECT_SPATIAL_CONTAINED   ("isContained")
#define EFAA_OPC_OBJECT_SPATIAL_INTERSECTS  ("intersects")

// defines for talking to tactileInterface
#define EFAA_TACTILEIF_RT2ROBOT_REQ         ("transform-table2icub")

#define EFAA_OPC_ENTITY_AGENT_KINECT_PREFIX         ("KinectAgent00")

// OPC : Kinect Skeleton joints
#define EFAA_OPC_BODY_PART_TYPE_HEAD        ("head")
#define EFAA_OPC_BODY_PART_TYPE_HAND_L      ("handLeft")
#define EFAA_OPC_BODY_PART_TYPE_HAND_R      ("handRight")
#define EFAA_OPC_BODY_PART_TYPE_WRIST_L     ("wristLeft")
#define EFAA_OPC_BODY_PART_TYPE_WRIST_R     ("wristRight")
#define EFAA_OPC_BODY_PART_TYPE_ELBOW_L     ("elbowLeft")
#define EFAA_OPC_BODY_PART_TYPE_ELBOW_R     ("elbowRight")
#define EFAA_OPC_BODY_PART_TYPE_SHOULDER_C  ("shoulderCenter")
#define EFAA_OPC_BODY_PART_TYPE_SHOULDER_L  ("shoulderLeft")
#define EFAA_OPC_BODY_PART_TYPE_SHOULDER_R  ("shoulderRight")
#define EFAA_OPC_BODY_PART_TYPE_SPINE       ("spine")
#define EFAA_OPC_BODY_PART_TYPE_HIP_C       ("hipCenter")
#define EFAA_OPC_BODY_PART_TYPE_HIP_L       ("hipLeft")
#define EFAA_OPC_BODY_PART_TYPE_HIP_R       ("hipRight")
#define EFAA_OPC_BODY_PART_TYPE_KNEE_L      ("kneeLeft")
#define EFAA_OPC_BODY_PART_TYPE_KNEE_R      ("kneeRight")
#define EFAA_OPC_BODY_PART_TYPE_ANKLE_L     ("ankleLeft")
#define EFAA_OPC_BODY_PART_TYPE_ANKLE_R     ("ankleRight")
#define EFAA_OPC_BODY_PART_TYPE_FOOT_L      ("footLeft")
#define EFAA_OPC_BODY_PART_TYPE_FOOT_R      ("footRight")
#define EFAA_OPC_BODY_PART_TYPE_COLLAR_L    ("collarLeft")
#define EFAA_OPC_BODY_PART_TYPE_COLLAR_R    ("collarRight")
#define EFAA_OPC_BODY_PART_TYPE_FT_L        ("fingertipLeft")
#define EFAA_OPC_BODY_PART_TYPE_FT_R        ("fingertipRight")
#define EFAA_OPC_BODY_PART_TYPE_COM         ("CoM")

// OPC: Attentional touching
#define EFAA_OPC_CONTACT_PROPERTY_PART       ("contact_part")
#define EFAA_OPC_CONTACT_PROPERTY_TYPE       ("contact_type")

#define EFAA_OPC_CONTACT_PART_LEFT        ("left")
#define EFAA_OPC_CONTACT_PART_RIGHT       ("right")
#define EFAA_OPC_CONTACT_PART_BOTH        ("both")

#define EFAA_OPC_CONTACT_TYPE_POKE        ("poke")
#define EFAA_OPC_CONTACT_TYPE_GRAB        ("grab")

//Kinect tags
#define EFAA_KINECT_ALL_INFO              ("all_info")
#define EFAA_KINECT_DEPTH                 ("depth")
#define EFAA_KINECT_DEPTH_PLAYERS         ("depth_players")
#define EFAA_KINECT_DEPTH_RGB             ("depth_rgb")
#define EFAA_KINECT_DEPTH_RGB_PLAYERS     ("depth_rgb_players")
#define EFAA_KINECT_DEPTH_JOINTS          ("depth_joints")
#define EFAA_KINECT_CMD_PING              ("ping")
#define EFAA_KINECT_CMD_ACK               ("ack")
#define EFAA_KINECT_CMD_NACK              ("nack")
#define EFAA_KINECT_CMD_GET3DPOINT        ("get3D")
#define EFAA_KINECT_SEATED_MODE           ("seated")
#define EFAA_KINECT_CLOSEST_PLAYER         -1

#endif
