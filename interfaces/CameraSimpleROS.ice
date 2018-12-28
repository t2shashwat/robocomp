//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: IDSLs/CameraSimpleROS.ice
//  Source: IDSLs/CameraSimpleROS.idsl
//  
//****************************************************************** 
#ifndef ROBOCOMPCAMERASIMPLE_ICE
#define ROBOCOMPCAMERASIMPLE_ICE
module RoboCompCameraSimple
{
	sequence <byte> ImgType;
	struct TImage
	{
		 int width;
		 int height;
		 int depth;
		 ImgType image;
	};
	interface CameraSimple
	{
		idempotent void getImage (out TImage im);
	};
};

#endif
