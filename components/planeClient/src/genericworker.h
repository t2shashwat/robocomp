/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>

#include <CommonBehavior.h>

#include <ros/ros.h>
#include <RoboCompPlaneROS/PointXY.h>
#include <RoboCompPlaneROS/Dimensions.h>
#include <RoboCompPlaneROS/Points.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <RoboCompPlaneROS/setHeight.h>
#include <RoboCompPlaneROS/addPoint.h>
#include <RoboCompPlaneROS/setWidth.h>
#include <RoboCompPlaneROS/setName.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


	//class for rosServiceClient
class ServiceClientplaneSetup
{
public:
	ros::ServiceClient srv_setHeight;
	ros::ServiceClient srv_addPoint;
	ros::ServiceClient srv_setWidth;
	ros::ServiceClient srv_setName;
	ServiceClientplaneSetup(ros::NodeHandle *node)
	{
		srv_setHeight = node->serviceClient<RoboCompPlaneROS::setHeight>(node->resolveName("setHeight"), 1000);
		srv_addPoint = node->serviceClient<RoboCompPlaneROS::addPoint>(node->resolveName("addPoint"), 1000);
		srv_setWidth = node->serviceClient<RoboCompPlaneROS::setWidth>(node->resolveName("setWidth"), 1000);
		srv_setName = node->serviceClient<RoboCompPlaneROS::setName>(node->resolveName("setName"), 1000);
	}
	~ServiceClientplaneSetup(){}
	bool setHeight(std_msgs::Int32 height, RoboCompPlaneROS::Dimensions &dims) 
	{
		RoboCompPlaneROS::setHeight srv;
		srv.request.height = height.data;
		if(srv_setHeight.call(srv))
		{
			dims.width = srv.response.dims.width;
			dims.height = srv.response.dims.height;
			return true;
		}
		return false;
	}
	bool addPoint(RoboCompPlaneROS::PointXY point, RoboCompPlaneROS::Points &pts) 
	{
		RoboCompPlaneROS::addPoint srv;
		srv.request.point.x = point.x;
		srv.request.point.y = point.y;
		if(srv_addPoint.call(srv))
		{
			return true;
		}
		return false;
	}
	bool setWidth(std_msgs::Int32 width, RoboCompPlaneROS::Dimensions &dims) 
	{
		RoboCompPlaneROS::setWidth srv;
		srv.request.width = width.data;
		if(srv_setWidth.call(srv))
		{
			dims.width = srv.response.dims.width;
			dims.height = srv.response.dims.height;
			return true;
		}
		return false;
	}
	bool setName(std_msgs::String name, std_msgs::String &lastName) 
	{
		RoboCompPlaneROS::setName srv;
		srv.request.name = name.data;
		if(srv_setName.call(srv))
		{
			lastName.data = srv.response.lastName;
			return true;
		}
		return false;
	}
};


class GenericWorker :
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;




protected:
	QTimer timer;
	int Period;
	ros::NodeHandle node;
	ServiceClientplaneSetup *planesetup_proxy;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
signals:
	void kill();
};

#endif
