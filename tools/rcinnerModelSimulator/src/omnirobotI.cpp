/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#include "omnirobotI.h"
#include "specificworker.h"

OmniRobotI::OmniRobotI(std::shared_ptr<SpecificWorker> _worker, QObject *parent): QThread(parent)
{
	// Pointer to the worker (needed to access the mutex)
	worker = _worker;
	// InnerModel
	innerModel = worker->getInnerModel();
	// Initialize the velocity to 0
	advVelx = advVelz = rotVel = 0;
	// Initialize timing
	gettimeofday(&lastCommand_timeval, NULL);
}


void OmniRobotI::add(std::string id)
{
	std::lock_guard<std::recursive_mutex> guard(innerModel->mutex);

	omniIDs << QString::fromStdString(id);
	node                        = innerModel->getNode<InnerModelOmniRobot>(id).get();
	parent                      = innerModel->getNode<InnerModelTransform>(node->parent->id).get();
	rawOdometryParentNode       = innerModel->newNode<InnerModelTransform>(id+"_raw_odometry_parent\"", "static", 0, 0, 0, 0, 0, 0, 0, 				std::shared_ptr<InnerModelNode>(parent)).get();
	rawOdometryNode             = innerModel->newNode<InnerModelTransform>(id+"_raw_odometry\"", "static",  0, 0, 0, 0, 0, 0, 0, std::shared_ptr<InnerModelNode>(rawOdometryParentNode)).get();
	correctedOdometryParentNode = innerModel->newNode<InnerModelTransform>(id+"_corrected_odometry_parent\"", "static", 0, 0, 0, 0, 0, 0, 0, std::shared_ptr<InnerModelNode>(parent)).get();
	correctedOdometryNode       = innerModel->newNode<InnerModelTransform>(id+"_corrected_odometry\"", "static", 0, 0, 0, 0, 0, 0, 0,  std::shared_ptr<InnerModelNode>(correctedOdometryParentNode)).get();
	movementFutureNode          = innerModel->newNode<InnerModelTransform>(id+"_move\"", "static", 0, 0, 0, 0, 0, 0, 0, std::shared_ptr<InnerModelNode>(node)).get();
}


void OmniRobotI::run()
{
	updateInnerModelPose(true);

	while (true)
	{
		updateInnerModelPose();
		usleep(10000);
	}
}


void OmniRobotI::getBaseState(RoboCompGenericBase::TBaseState& state, const Ice::Current&)
{
	std::lock_guard<std::recursive_mutex> guard(innerModel->mutex);

	{
		QVec retPOSR = innerModel->transform6D(parent->id, node->id+"_raw_odometry\"");
		state.x = retPOSR(0);
		state.z = retPOSR(2);
		state.alpha = retPOSR(4);
	}
	
	{
		QVec retPOSC = innerModel->transform6D(parent->id, node->id+"_corrected_odometry\"");
		state.correctedX = retPOSC(0);
		state.correctedZ = retPOSC(2);
		state.correctedAlpha = retPOSC(4);
	}

	state.isMoving = ( (fabs(advVelx)<0.0001 and fabs(advVelz)<0.0001 and fabs(rotVel)<0.0001));
	
	state.advVx = advVelx;
	state.advVz = advVelz;
	state.rotV  = rotVel;
}


void OmniRobotI::getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current&)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);

	QVec retPOS = innerModel->transform6D(parent->id, node->id+"_raw_odometry\"");
	x = retPOS(0);
	z = retPOS(2);
	alpha = retPOS(4);
}


void OmniRobotI::updateInnerModelPose(bool force)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);

	// Do nothing if the robot isn't moving
	if ( (fabs(advVelx)<0.0001 and fabs(advVelz)<0.0001 and fabs(rotVel)<0.0001) and not force)
	{
		return;
	}

	// Compute idle time
	timeval now;
	gettimeofday(&now, NULL);
	const double msecs = (now.tv_sec - lastCommand_timeval.tv_sec)*1000. +(now.tv_usec - lastCommand_timeval.tv_usec)/1000.;
	lastCommand_timeval = now;

	// Compute estimated increments given velocity and time
	QVec estimatedIncrements = QVec::vec6(advVelx, 0,  advVelz, 0, rotVel, 0).operator*(msecs / 1000.);

	// Update raw odometry using estimated pose increments
	innerModel->getNode<InnerModelTransform>(node->id+"_raw_odometry\"")->update(estimatedIncrements);
	QVec finalRawPose = innerModel->transform6D(parent->id, node->id+"_raw_odometry\"");
	innerModel->getNode<InnerModelTransform>(node->id+"_raw_odometry_parent\"")->update(finalRawPose);
	innerModel->getNode<InnerModelTransform>(node->id+"_raw_odometry\"")->update(QVec::vec6(0,0,0,0,0,0));

	// Update corrected odometry using estimated pose increments
	innerModel->transform6D("root", node->id+"_corrected_odometry_parent\"").print(":updateIMP  correctedP 0");
	innerModel->transform6D("root", node->id+"_corrected_odometry\"").print(       ":updateIMP  corrected  0");
	estimatedIncrements.print("estimatedIncrements");

	innerModel->getNode<InnerModelTransform>(node->id+"_corrected_odometry\"")->update(estimatedIncrements);

	innerModel->transform6D("root", node->id+"_corrected_odometry_parent\"").print(":updateIMP  correctedP 1");
	innerModel->transform6D("root", node->id+"_corrected_odometry\"").print(       ":updateIMP  corrected  1");

	QVec finalCorrectedPose = innerModel->transform6D(parent->id, node->id+"_corrected_odometry\"");
	innerModel->getNode<InnerModelTransform>(node->id+"_corrected_odometry_parent\"")->update(finalCorrectedPose);
	innerModel->getNode<InnerModelTransform>(node->id+"_corrected_odometry\"")->update(QVec::vec6(0,0,0,0,0,0));

	innerModel->transform6D("root", node->id+"_corrected_odometry_parent\"").print(":updateIMP  correctedP 2");
	innerModel->transform6D("root", node->id+"_corrected_odometry\"").print(       ":updateIMP  corrected  2");

	// 	printf("rotvel %f\n", rotVel);
	
	// Compute noisy increments
	const double noise = node->noise;
	float speed = QVec::vec3(advVelx, 0, advVelz).norm2();
	QVec rndmPos = QVec::gaussianSamples(2, 0, noise*msecs*(0.000001*speed + 0.00001*rotVel));
	QVec rndmYaw = QVec::gaussianSamples(1, 0, noise*msecs*(0.00001*speed + 0.00001*rotVel));
	QVec actualIncrements = QVec::vec6(advVelx+rndmPos(0), 0, advVelz+rndmPos(1), 0, rotVel+rndmYaw(0), 0).operator*(msecs / 1000.);
// 	QVec actualIncrements = QVec::vec6(advVelx, 0, advVelz, 0, rotVel, 0).operator*(msecs / 1000.);
// 	actualIncrements.print("actualIncrements");


	QVec backPose = innerModel->transform6D(parent->id, node->id);
// 	backPose.print("backPose");
	innerModel->getNode<InnerModelTransform>(node->id+"_move\"")->update(actualIncrements);
	QVec newPose = innerModel->transform6D(parent->id, node->id+"_move\"");
// 	newPose.print("newPose");
	innerModel->getNode<InnerModelTransform>(node->id)->update(newPose);
	if (not canMoveBaseTo(QString::fromStdString(node->id)))
	{
		innerModel->getNode<InnerModelTransform>(node->id)->update(backPose);
	}
	innerModel->getNode<InnerModelTransform>(node->id+"_move\"")->update(QVec::vec6(0,0,0,0,0,0));
	
}

bool OmniRobotI::canMoveBaseTo(const QString nodeId)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);

	if (not node->collide) return true;

	std::vector<QString> robotNodes;
	std::vector<QString> restNodes;

	recursiveIncludeMeshes(innerModel->getRoot().get(), nodeId, false, robotNodes, restNodes);

	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			if (innerModel->collide(robotNodes[in].toStdString(), restNodes[out].toStdString()))
			{
				return false;
			}
		}
	}

	return true;
}

void OmniRobotI::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);

	if (node->id == robotId.toStdString())
	{
		inside = true;
	}

	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;

	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		for (auto iterator: node->children)
		{
			recursiveIncludeMeshes(iterator.get(), robotId, inside, in, out);
		}
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		if (inside)
		{
			in.push_back(QString::fromStdString(node->id));
		}
		else
		{
			out.push_back(QString::fromStdString(node->id));
		}
	}
}


void OmniRobotI::setSpeedBase(Ice::Float advx, Ice::Float advz, Ice::Float rot, const Ice::Current&)
{
	
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);
	
	gettimeofday(&lastCommand_timeval, NULL);
	advVelx = advx;
	advVelz = advz;
	rotVel = rot;
	
	updateInnerModelPose();
}


void OmniRobotI::stopBase(const Ice::Current&)
{
	setSpeedBase(0., 0., 0.);
}


void OmniRobotI::resetOdometer(const Ice::Current&)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);
	setOdometerPose(0, 0, 0);
}


void OmniRobotI::setOdometer(const RoboCompGenericBase::TBaseState &st, const Ice::Current&)
{
	setOdometerPose(st.x, st.z, st.alpha);
}


void OmniRobotI::setOdometerPose(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);
	innerModel->getNode<InnerModelTransform>(node->id+"_raw_odometry_parent\"")->update(x, 0, z, 0, alpha, 0);
	innerModel->getNode<InnerModelTransform>(node->id+"_raw_odometry\"")->update(0, 0, 0,  0, 0, 0);
}


void OmniRobotI::correctOdometer(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	std::lock_guard<std::recursive_mutex> guard (innerModel->mutex);
	innerModel->getNode<InnerModelTransform>(node->id+"_corrected_odometry_parent\"")->update(x, 0, z, 0, alpha, 0);
	innerModel->getNode<InnerModelTransform>(node->id+"_corrected_odometry\"")->update(0, 0, 0,  0, 0, 0);
	innerModel->transform6D("root", node->id+"_corrected_odometry_parent\"").print(":correctOd  correctedP");
	innerModel->transform6D("root", node->id+"_corrected_odometry\"").print(":correctOd  correctedC");
}

