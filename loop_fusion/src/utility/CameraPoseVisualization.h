/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "../parameters.h"

class CameraPoseVisualization {
public:
	std::string m_marker_ns;

	// ROS-less marker model (enough to keep current functionality)
	struct ColorRGBA {
		float r = 0.f;
		float g = 0.f;
		float b = 0.f;
		float a = 1.f;
	};
	struct Point {
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};
	struct Marker {
		enum class Type : std::uint8_t {
			LINE_LIST = 0,
			LINE_STRIP = 1,
		};

		std::string ns;
		int id = 0;
		Type type = Type::LINE_STRIP;
		double scale_x = 0.01; // line width

		// Either use per-point colors (same length as points) or a single uniform color.
		bool use_per_point_color = true;
		ColorRGBA color{};
		std::vector<Point> points;
		std::vector<ColorRGBA> colors;
	};

	CameraPoseVisualization(float r, float g, float b, float a);
    
	void setImageBoundaryColor(float r, float g, float b, float a=1.0);
	void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
	void setScale(double s);
	void setLineWidth(double width);

	void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
	void reset();

	// Replacement for ROS publish:
	// - getMarkers(): inspect current markers (camera frustums + edges)
	// - consumeMarkers(): move them out and clear internal list
	const std::vector<Marker>& getMarkers() const;
	std::vector<Marker> consumeMarkers();

	void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
	void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

private:
	double m_scale;
	double m_line_width;
	int LOOP_EDGE_NUM;
	int tmp_loop_edge_num;

	std::vector<Marker> m_markers;
	ColorRGBA m_image_boundary_color;
	ColorRGBA m_optical_center_connector_color;
	Marker image; // kept for parity (currently unused in your .cpp)

	static const Eigen::Vector3d imlt;
	static const Eigen::Vector3d imlb;
	static const Eigen::Vector3d imrt;
	static const Eigen::Vector3d imrb;
	static const Eigen::Vector3d oc  ;
	static const Eigen::Vector3d lt0 ;
	static const Eigen::Vector3d lt1 ;
	static const Eigen::Vector3d lt2 ;
};
