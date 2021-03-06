/*****************************************************************************
 *
 *  (C) Copyright 1998 by National Advanced Driving Simulator and
 *  Simulation Center, the University of Iowa and The University
 *  of Iowa. All rights reserved.
 *
 *  This file has been generated by the hcsm code generator.
 *  ### DO NOT EDIT DIRECTLY ###
 *
 */


#include "genbuttondialinfo.h"
CButtonDialInfo::CButtonDialInfo()
{
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "AudioState";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "AudioState";
			node2.m_dataType = "int";
			node2.m_units = "Audio_bits";
			node2.m_defValue = "16";
			node2.m_optional = false;
			node2.m_comment = "The_audio_sound_to_generate";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "VisualState";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "LightState";
			node2.m_dataType = "int";
			node2.m_units = "bits";
			node2.m_defValue = "16";
			node2.m_optional = false;
			node2.m_comment = "The_state_of_the_light";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "int";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_keep_light_state";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "Mode";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Mode";
			node2.m_dataType = "int";
			node2.m_units = "int";
			node2.m_defValue = "0";
			node2.m_optional = false;
			node2.m_comment = "Mode";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "Dependent";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Dependent";
			node2.m_dataType = "int";
			node2.m_units = "int";
			node2.m_defValue = "0";
			node2.m_optional = false;
			node2.m_comment = "Is_the_dDDO_still_dependent";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "SpeedOverRide";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "SpeedOverRide";
			node2.m_dataType = "double";
			node2.m_units = "double";
			node2.m_defValue = "0";
			node2.m_optional = false;
			node2.m_comment = "Is_the_dDDO_still_dependent";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "DiGuyAction";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Direction";
			node2.m_dataType = "string";
			node2.m_units = "direction";
			node2.m_defValue = "right";
			node2.m_optional = false;
			node2.m_comment = "Direction_of_lane_change";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "TurnOnAnimation";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ddo";
		node.m_name = "TurnOffAnimation";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "SetAnimation";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "SetRotation";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "SetPosition";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "SetStateIndex";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "SetDrawType";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "AttachToObject";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "AttachToLight";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "TurnOnAnimation";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VirtualObject";
		node.m_name = "TurnOffAnimation";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TimeTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "GmtrcPstnTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TrffcLghtTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "RoadPadTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "ExpressionTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "FollowTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TimeToArrvlTrigger";
		node.m_name = "FireTrigger";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "AudioState";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "AudioState";
			node2.m_dataType = "int";
			node2.m_units = "Audio_bits";
			node2.m_defValue = "16";
			node2.m_optional = false;
			node2.m_comment = "The_audio_sound_to_generate";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "int";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_generate_audio";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "ForcedLaneOffset";
		node.m_comment = "Forced_Lane_Offset";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "LaneOffset";
			node2.m_dataType = "double";
			node2.m_units = "feet";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "Offset_from_lane_center";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Urgency";
			node2.m_dataType = "double";
			node2.m_units = "none";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "Urgency_between_0_and_1";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Delay";
			node2.m_dataType = "double";
			node2.m_units = "frames";
			node2.m_defValue = "10";
			node2.m_optional = false;
			node2.m_comment = "start_delay";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MaxSteer";
			node2.m_dataType = "double";
			node2.m_units = "deg_per_sec";
			node2.m_defValue = "129";
			node2.m_optional = false;
			node2.m_comment = "max_steer_rate";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MaxSteerOffset";
			node2.m_dataType = "double";
			node2.m_units = "feet";
			node2.m_defValue = "9";
			node2.m_optional = false;
			node2.m_comment = "max_offset_rate";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MaxSteerForce";
			node2.m_dataType = "double";
			node2.m_units = "feet";
			node2.m_defValue = "3";
			node2.m_optional = false;
			node2.m_comment = "max_force";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "FinishDistance";
			node2.m_dataType = "double";
			node2.m_units = "inches";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "tolerance";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "ForcedVelocity";
		node.m_comment = "Forced_Velocity";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "InhibitLaneChange";
		node.m_comment = "Inhibit_optional_lane_change_behavior";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "double";
			node2.m_units = "seconds";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "No_lane_change_for_this_duration";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "LaneChange";
		node.m_comment = "Lane_Change";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Direction";
			node2.m_dataType = "string";
			node2.m_units = "direction";
			node2.m_defValue = "right";
			node2.m_optional = false;
			node2.m_comment = "Direction_of_lane_change";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Urgency";
			node2.m_dataType = "double";
			node2.m_units = "none";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "Urgency_between_0_and_1";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "TurnSignalDuration";
			node2.m_dataType = "double";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_signal_before_lane_change";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "LaneChangeStatus";
		node.m_comment = "Lane_Change_Status";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Status";
			node2.m_dataType = "int";
			node2.m_units = "Satus";
			node2.m_defValue = "0";
			node2.m_optional = false;
			node2.m_comment = "lane_change_allowed";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "int";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_keep_this_condition";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "MaintainGap";
		node.m_comment = "Maintain_Gap";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "ObjName";
			node2.m_dataType = "string";
			node2.m_units = "none";
			node2.m_defValue = "ExternalDriver";
			node2.m_optional = false;
			node2.m_comment = "Name_of_object_maintain_gap_from";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Type";
			node2.m_dataType = "string";
			node2.m_units = "none";
			node2.m_defValue = "d";
			node2.m_optional = false;
			node2.m_comment = "d_is_for_distance_and_t_is_for_time";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Value";
			node2.m_dataType = "double";
			node2.m_units = "feet_or_seconds";
			node2.m_defValue = "-120";
			node2.m_optional = false;
			node2.m_comment = "Associated_with_mode";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MaxSpeed";
			node2.m_dataType = "double";
			node2.m_units = "mph";
			node2.m_defValue = "100";
			node2.m_optional = false;
			node2.m_comment = "Maximum_velocity";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MinSpeed";
			node2.m_dataType = "double";
			node2.m_units = "mph";
			node2.m_defValue = "20";
			node2.m_optional = false;
			node2.m_comment = "Minimum_velocity";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "DistKp";
			node2.m_dataType = "double";
			node2.m_units = "none";
			node2.m_defValue = "4";
			node2.m_optional = false;
			node2.m_comment = "Distance_gain";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "VelKp";
			node2.m_dataType = "double";
			node2.m_units = "none";
			node2.m_defValue = "4";
			node2.m_optional = false;
			node2.m_comment = "Velocity_gain";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MaxAccel";
			node2.m_dataType = "double";
			node2.m_units = "ms2";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "Maximum_acceleration";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "MaxDecel";
			node2.m_dataType = "double";
			node2.m_units = "ms2";
			node2.m_defValue = "-2";
			node2.m_optional = false;
			node2.m_comment = "Maximum_deceleration";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "PercentAroundTarget";
			node2.m_dataType = "double";
			node2.m_units = "percent";
			node2.m_defValue = "-1";
			node2.m_optional = false;
			node2.m_comment = "Percentage_inside_target_value";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "TimeAroundTarget";
			node2.m_dataType = "double";
			node2.m_units = "seconds";
			node2.m_defValue = "-1";
			node2.m_optional = false;
			node2.m_comment = "Time_to_spend_inside_target_range_before_reset";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "DisableSpeed";
			node2.m_dataType = "double";
			node2.m_units = "mph";
			node2.m_defValue = "-1";
			node2.m_optional = false;
			node2.m_comment = "Reset_when_below_this_speed";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "DisableTime";
			node2.m_dataType = "double";
			node2.m_units = "seconds";
			node2.m_defValue = "-1";
			node2.m_optional = false;
			node2.m_comment = "Reset_when_this_time_expired";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "TargetVelocity";
		node.m_comment = "Change_Target_Velocity";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Speed";
			node2.m_dataType = "double";
			node2.m_units = "MPH";
			node2.m_defValue = "30";
			node2.m_optional = false;
			node2.m_comment = "The_Target_Speed";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "int";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_keep_target_Velocity";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "VisualState";
		node.m_comment = "Change_Light_State";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "LightState";
			node2.m_dataType = "int";
			node2.m_units = "bits";
			node2.m_defValue = "16";
			node2.m_optional = false;
			node2.m_comment = "The_state_of_the_light";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "int";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_keep_light_state";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "ChangeLaneLeft";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "ChangeLaneRight";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "TurnLeft";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "TurnRight";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "Ado";
		node.m_name = "ProjectAndResetLaneOffset";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "StaticObjManager";
		node.m_name = "SetOption1";
		node.m_comment = "Set_Option_1";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Name";
			node2.m_dataType = "string";
			node2.m_units = "none";
			node2.m_defValue = "name";
			node2.m_optional = false;
			node2.m_comment = "name_of_object";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Option";
			node2.m_dataType = "int";
			node2.m_units = "none";
			node2.m_defValue = "Object";
			node2.m_optional = false;
			node2.m_comment = "option_number";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "StaticObjManager";
		node.m_name = "SetOption2";
		node.m_comment = "Set_Option_2";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Name";
			node2.m_dataType = "string";
			node2.m_units = "none";
			node2.m_defValue = "name";
			node2.m_optional = false;
			node2.m_comment = "name_of_object";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Option";
			node2.m_dataType = "int";
			node2.m_units = "none";
			node2.m_defValue = "Object";
			node2.m_optional = false;
			node2.m_comment = "option_number";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "StaticObjManager";
		node.m_name = "AudioState";
		node.m_comment = "int";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "StaticObjManager";
		node.m_name = "VisualState";
		node.m_comment = "int";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "StaticObjManager";
		node.m_name = "AnimationState";
		node.m_comment = "int";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TrafficLightManager";
		node.m_name = "TrafficLight";
		node.m_comment = "Traffic_Light";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Name";
			node2.m_dataType = "string";
			node2.m_units = "string";
			node2.m_defValue = "name";
			node2.m_optional = false;
			node2.m_comment = "Signal_Name";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Singal";
			node2.m_dataType = "string";
			node2.m_units = "string";
			node2.m_defValue = "signal";
			node2.m_optional = false;
			node2.m_comment = "Signal";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Duration";
			node2.m_dataType = "int";
			node2.m_units = "seconds";
			node2.m_defValue = "2";
			node2.m_optional = false;
			node2.m_comment = "How_long_to_keep_this_condition";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TrafficManager";
		node.m_name = "InputSet";
		node.m_comment = "Input_Set";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TrafficManager";
		node.m_name = "MakeTraffic";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TrafficSource";
		node.m_name = "StartStop";
		node.m_comment = "Start_Stop";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Stop";
			node2.m_dataType = "bool";
			node2.m_units = "bool";
			node2.m_defValue = "0";
			node2.m_optional = false;
			node2.m_comment = "StartStop";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "TrafficSource";
		node.m_name = "MakeTraffic";
		node.m_comment = "(null)";
		node.m_isDial = false;

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "VehFail";
		node.m_name = "Failure";
		node.m_comment = "Failure_Text";
		node.m_isDial = true;
		{
			TDialParam node2;
			node2.m_name = "Failure";
			node2.m_dataType = "int";
			node2.m_units = "Failure";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "The_Failure_Catagory";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Type";
			node2.m_dataType = "int";
			node2.m_units = "Type";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "The_Type_of_Failure";
			node.m_params.push_back( node2 );
		}
		{
			TDialParam node2;
			node2.m_name = "Time";
			node2.m_dataType = "int";
			node2.m_units = "Time";
			node2.m_defValue = "1";
			node2.m_optional = false;
			node2.m_comment = "The_Duration_of_Failure";
			node.m_params.push_back( node2 );
		}

		m_buttonDialInfo.push_back( node );
	}
	{
		TButtonDialInfo node;
		node.m_hcsmName = "DriverMirror";
		node.m_name = "TargetVelocity";
		node.m_comment = "Change_Target_Velocity";
		node.m_isDial = true;

		m_buttonDialInfo.push_back( node );
	}
}


CButtonDialInfo::~CButtonDialInfo()
{
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns all dial parameter info for a given HCSM.
//
// Remarks: If the given HCSM is not found then nothing is added to the
//  output vector.  This function does not clear the output vector before
//  writing to it.
//
// Arguments:
//  cHcsmName - The HCSM for which parameter information is needed.
//  output - (output) The output parameter vector.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CButtonDialInfo::GetDialInfo(
			const string& cHcsmName,
			vector<TButtonDialInfo>& output
			)
{
	vector<TButtonDialInfo>::iterator i;
	for( i = m_buttonDialInfo.begin(); i != m_buttonDialInfo.end(); i++ )
	{
		if( i->m_isDial && cHcsmName == i->m_hcsmName )
		{
			output.push_back( *i );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns all button parameter info for a given HCSM.
//
// Remarks: If the given HCSM is not found then nothing is added to the
//  output vector.  This function does not clear the output vector before
//  writing to it.
//
// Arguments:
//  cHcsmName - The HCSM for which parameter information is needed.
//  output - (output) The output parameter vector.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CButtonDialInfo::GetButtonInfo(
			const string& cHcsmName,
			vector<TButtonDialInfo>& output
			)
{
	vector<TButtonDialInfo>::iterator i;
	for( i = m_buttonDialInfo.begin(); i != m_buttonDialInfo.end(); i++ )
	{
		if( !(i->m_isDial) && cHcsmName == i->m_hcsmName )
		{
			output.push_back( *i );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns all parameter info for all HCSMs.
//
// Remarks: This function does not clear the output vector before
//  writing to it.
//
// Arguments:
//  output - (output) The output parameter vector.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void CButtonDialInfo::GetButtonDialInfo( vector<TButtonDialInfo>& output )
{
	vector<TButtonDialInfo>::iterator i;
	for( i = m_buttonDialInfo.begin(); i != m_buttonDialInfo.end(); i++ )
	{
		output.push_back( *i );
	}
}


