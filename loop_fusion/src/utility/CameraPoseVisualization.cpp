/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "CameraPoseVisualization.h"

const Eigen::Vector3d CameraPoseVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrt = Eigen::Vector3d( 1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imlb = Eigen::Vector3d(-1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrb = Eigen::Vector3d( 1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);

static inline CameraPoseVisualization::Point Eigen2Point(const Eigen::Vector3d& v) {
    CameraPoseVisualization::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

CameraPoseVisualization::CameraPoseVisualization(float r, float g, float b, float a)
    : m_marker_ns("CameraPoseVisualization"), m_scale(0.2), m_line_width(0.01) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
    LOOP_EDGE_NUM = 20;
    tmp_loop_edge_num = 1;
}

void CameraPoseVisualization::setImageBoundaryColor(float r, float g, float b, float a) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
}

void CameraPoseVisualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void CameraPoseVisualization::setScale(double s) {
    m_scale = s;
}
void CameraPoseVisualization::setLineWidth(double width) {
    m_line_width = width;
}
void CameraPoseVisualization::add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    Marker marker;
    marker.ns = m_marker_ns;
    marker.id = static_cast<int>(m_markers.size()) + 1;
    marker.type = Marker::Type::LINE_LIST;
    marker.scale_x = 0.01;
    marker.use_per_point_color = false;
    marker.color = {0.f, 0.f, 1.f, 1.f}; // blue
    marker.points.push_back(Eigen2Point(p0));
    marker.points.push_back(Eigen2Point(p1));
    m_markers.push_back(std::move(marker));
}

void CameraPoseVisualization::add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    Marker marker;
    marker.ns = m_marker_ns;
    marker.id = static_cast<int>(m_markers.size()) + 1;
    marker.type = Marker::Type::LINE_STRIP;
    marker.scale_x = 0.02;
    marker.use_per_point_color = false;
    marker.color = {1.f, 0.f, 0.f, 1.f}; // red
    marker.points.push_back(Eigen2Point(p0));
    marker.points.push_back(Eigen2Point(p1));
    m_markers.push_back(std::move(marker));
}


void CameraPoseVisualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    Marker marker;
    marker.ns = m_marker_ns;
    marker.id = 0;
    marker.type = Marker::Type::LINE_STRIP;
    marker.scale_x = m_line_width;
    marker.use_per_point_color = true;

    const Point pt_lt  = Eigen2Point(q * (m_scale * imlt) + p);
    const Point pt_lb  = Eigen2Point(q * (m_scale * imlb) + p);
    const Point pt_rt  = Eigen2Point(q * (m_scale * imrt) + p);
    const Point pt_rb  = Eigen2Point(q * (m_scale * imrb) + p);
    const Point pt_lt0 = Eigen2Point(q * (m_scale * lt0 ) + p);
    const Point pt_lt1 = Eigen2Point(q * (m_scale * lt1 ) + p);
    const Point pt_lt2 = Eigen2Point(q * (m_scale * lt2 ) + p);
    const Point pt_oc  = Eigen2Point(q * (m_scale * oc  ) + p);

    // image boundaries
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // top-left indicator
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // optical center connector
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    m_markers.push_back(std::move(marker));
}

void CameraPoseVisualization::reset() {
    m_markers.clear();
}

const std::vector<CameraPoseVisualization::Marker>& CameraPoseVisualization::getMarkers() const {
    return m_markers;
}

std::vector<CameraPoseVisualization::Marker> CameraPoseVisualization::consumeMarkers() {
    std::vector<Marker> out;
    out.swap(m_markers);
    return out;
}
