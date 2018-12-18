/*
 * Copyright 2016 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef INNERMODELCAMERA_H
#define INNERMODELCAMERA_H

#include <innermodel/innermodelnode.h>

class InnerModel;

class InnerModelCamera : public InnerModelNode
{
	public:
		InnerModelCamera(std::string id_, float width_, float height_, float focal_, InnerModel* innermodel_, std::shared_ptr<InnerModelNode> parent_= NULL);
		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		void update();
		virtual std::shared_ptr<InnerModelNode> copyNode(std::map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent);

		Cam camera;
		float width, height, focal;
		float getWidth()  const { return width; }
		float getHeight() const { return height; }
		float getFocal()  const { return focal; }
		float getSize()   const { return getWidth()*getHeight(); }
		void updateValues(float width_, float height_, float focal_);
		
		QVec project(std::string reference, QVec origVec);
		QVec project(const QVec &origVec);
		QVec backProject(const std::string &cameraId, const QVec &coord) ;//const;
		void imageCoordToAngles(const std::string &cameraId, QVec coord, float &pan, float &tilt, const std::string & anglesRefS);
		QVec anglesToImageCoord(const std::string &cameraId, float pan, float tilt, const std::string & anglesRefS);
		QVec imageCoordPlusDepthTo(std::string cameraId, QVec coord, float depth, std::string to);
		QVec projectFromCameraToPlane(const std::string &to, const QVec &coord, const std::string &cameraId, const QVec &vPlane, const float &dist);
		QVec horizonLine(std::string planeId, std::string cameraId, float heightOffset=0.);
		QMat getHomographyMatrix(std::string destCamera, std::string plane, std::string sourceCamera);
		QMat getAffineHomographyMatrix(std::string destCamera, std::string plane, std::string sourceCamera);
		QMat getPlaneProjectionMatrix(std::string virtualCamera, std::string plane, std::string sourceCamera);
		/// Stereo computations
		void updateStereoGeometry( const std::string &firstCam, const std::string & secondCam );
		QVec compute3DPointInCentral(const std::string &firstCamera , const QVec & left, const std::string & secondCamera , const QVec & right);
		QVec compute3DPointInRobot(const std::string &firstCamera , const QVec & left, const std::string & secondCamera , const QVec & right);
		QVec compute3DPointFromImageCoords(const std::string &firstCamera , const QVec & left, const std::string & secondCamera , const QVec & right, const std::string & refSystem);
		QVec compute3DPointFromImageAngles(const std::string &firstCamera , const QVec & left, const std::string & secondCamera , const QVec & right, const std::string & refSystem);
		/// Frustrum
		struct TPlane { QVec n; float d; };
		struct TFrustrum { TPlane left; TPlane top; TPlane right; TPlane down; TPlane near; TPlane far;};
		TFrustrum frustrumLeft, frustrumThird, frustrumRight;
		
		mutable QMutex mutex;
		
		InnerModel *innermodel;
};

#endif // INNERMODELCAMERA_H
